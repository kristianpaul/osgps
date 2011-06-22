#include "SoftOSGPS.h"

/***********************************************************************
  Software GPS RECEIVER (SoftOSGPS)



***********************************************************************/


#include "serport.h"           /* NMEA */
#include "nmea.h"              /* NMEA */
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "file.h"
#include "display.h"

#ifdef __GNUC__
#include <errno.h>
#include <sys/socket.h>
#include <netinet/ip.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/un.h>
#include <getopt.h>
#elif (defined WIN32)
#include <winsock2.h>
#include "ws2tcpip.h"
#include "getopt.h"
typedef int socklen_t;
#endif

#define MAIN
#include  "consts.h"
#include  "structs.h"
#include  "interfac.h"
#include  "gpsfuncs.h"
#include  "globals.h"
#include "rinex.h"
#include "correlator.h"
#undef  MAIN

#ifndef FILENAME_MAX
#define FILENAME_MAX 256
#endif

#define DEFAULT_FILENAME "gnss.bin"

extern long code_ref, carrier_ref;
extern long carrier_ref_cycles;

FILE *output, *debug, *in, *out, *kalm, *data_bits;
FILE *rinex_obs, *rinex_nav, *rinex_par;
char output_file[40],almanac_file[40],ephemeris_file[40],receiver_file[40];
char location_file[40],ion_utc_file[40],data_bits_file[40],kalman_file[40];
char rinex_obs_file[40],rinex_nav_file[40],rinex_par_file[40],debug_file[40];

/* Internal functions */
static void read_rcvr_par (void);
static void chan_allocate (void);
static void cold_allocate (void);
static void read_rinex_par (void);
static void read_filenames (void);
static void unpack(char *,char *,int);

/* External functions */
extern void nav_fix (void);
extern void Interrupt_Install (void);
extern void Interrupt_Remove  (void);


#ifdef __GNUC__
#define UNUSED __attribute__ ((unused))
#else
#define UNUSED /* */
#endif

void generate_prn_codes(void);
void create_wave(float [], int, double, double, double, int);
void gensig(float [], int, int, double, double, double, int);
void Sim_GP2021(char IF[],long nsamp,long *tic);
void Sim_GP2021_int (char *IF, long nsamp);
void gpsisr (void);


struct stat buffer;
int         status;
time_t utctime;

long  tic_counter;



/**
 * Tracking constants for 2nd order tracking loops 
 *
 *  Variable definitions:                                      
 *   On   = Natural frequency (Hz)                             
 *   zeta = Damping coefficient (unitless) set to 0.7071       
 *   BWn  = Noise bandwidth (Hz)                               
 *   Ts   = Integration time (s)                               
 *   Ko   = Discriminator gain  chip (code) /cycle (carrier)   
 *        = 1 for code loop                                    
 *        = 2*pi for carrier loop                              
 *   Kn   = NCO resolution (Hz)                                
 *        = 0.042575 Hz or 40e6/7/2^27                         
 *
 *  Equations:                                                 
 *   On = 8*zeta*BWn/(4*zeta^2+1)                              
 *   C1 = (1/(Ko*Kn))*8*zeta*On*Ts/(4+4*zeta*On*Ts+(On*Ts)^2)  
 *   C2 = (1/(Ko*Kn))*4*(On*Ts)^2/(4+4*zeta*On*Ts+(On*Ts)^2)   
 **/
static void
get_second_order_tracking_const (int *C1, int *C2,
				 double zeta, /* Damping coefficient (no unit)*/
				 double BWn,  /* Noise bandwidth (Hz) */
				 double Ts,   /* Integration Time (seconds) */
				 double Ko,   /* Discriminator gain */
				 double Kn)   /* NCO resolution (Hz) */
{
  double On = ((8.0 * zeta * BWn) / (4.0 * zeta * zeta + 1.0));
  double KoKn = Ko * Kn;
  double OnTs = On * Ts;
  double denom = (4.0 + 4.0 * zeta * OnTs + OnTs * OnTs);

  *C1=(int) ((1.0 / KoKn * 8.0 * zeta * On) / denom + 0.5);
  *C2=(int) ((1.0 / KoKn * 4.0 * OnTs * On) / denom + 0.5);
}


/* XXX FIX ME: This should be a structure **/
extern int trk_carr_C1, trk_carr_C2;
extern int trk_code_C1, trk_code_C2;
extern int pull_carr_C1, pull_carr_C2;
extern int pull_code_C1, pull_code_C2;
double Carrier_DCO_Delta, Code_DCO_Delta;
extern int d_freq;
extern int acq_thresh;

static int
get_acq_thresh (double samp_rate)
{
  double acq_thres_db = 35;  /* dB-Hz, this a generally accepted value */
  double acq_thres_value = pow(10.0, acq_thres_db/10.0);

  /**
   * C / No = (1/2) * (A/sigma_n)^2 * (1 / T); in Hz 
   * T = 0.001 seconds
   **/
  double acq_thres_ratio = sqrt(2.0 * acq_thres_value / 1000);
  int carrier_dco_levels[8] = {-1, 1, 2, 2, 1, -1, -2, -2};
  int i;

  double sigma_n;
  double sum_sq = 0, mean_sum_sq;
/*   double hack = samp_rate / (40e6/7.0); */
  double hack = 1;
  for (i = 0; i < 8; i++)
    sum_sq += (carrier_dco_levels[i] * carrier_dco_levels[i]);
  mean_sum_sq = sum_sq / 8;

  /* We're at level "3" of the IF 30% of the time and at level "1" 70%
     of the time so... */

  sigma_n = sqrt (2.0 *((mean_sum_sq * 3.0 * 3.0) * 0.3 
			+ (mean_sum_sq * 1.0 * 1.0) * 0.7) 
		  * samp_rate * hack * 0.001);

  return (int) (sigma_n * acq_thres_ratio + 0.5);
}

static void
tracking_init()
{
  double zeta = 1.0 / sqrt(2.0);

  /* pull-in is all 1 ms    */
  /* tracking loop constants for pull-in state  */

  get_second_order_tracking_const (&pull_carr_C1, &pull_carr_C2,
				   zeta, 25.0, 1e-3, 2.0 * M_PI, 
				   Carrier_DCO_Delta);
  get_second_order_tracking_const (&pull_code_C1, &pull_code_C2,
				   zeta, 25.0, 1e-3,        1.0,    
				   Code_DCO_Delta);

  /* 25 Hz, 1 ms */
  get_second_order_tracking_const (&trk_carr_C1, &trk_carr_C2,
				   zeta, 15.0, 1e-3, 2.0 * M_PI, 
				   Carrier_DCO_Delta);

  /* 1 Hz, 20 ms */
  get_second_order_tracking_const (&trk_code_C1, &trk_code_C2,
				   zeta,  1.0, 0.02,        1.0,    
				   Code_DCO_Delta);

}

/******************************************************************************
FUNCTION main()
RETURNS  None.

PARAMETERS None.

PURPOSE
	This is the main program to control the GPS receiver

WRITTEN BY
	Clifford Kelley

******************************************************************************/
int
main (int argc UNUSED, char *argv[] UNUSED)
{
  FILE *ifdata;
  char IF[16368];   /* reserve enough space for 1 ms at 16.368 MHz  */
  char IFP[16368];  /* reserve enough space for unpacking 1 ms of data at 16.368 MHz  */
  int i,ch;
  double simtime;
  int wrd;
  char wrdh, wrdl;
  char IF_Filename[FILENAME_MAX];
  int idispcnt = 0;
  double freq_bin_width = 200; /* Hertz */
  double tic_period = 0.1;     /* Seconds */

  /* for the SiGe these are the parameters */
  double samp_rate=16.3676e6; /*  sampling rate */
  double carrier_IF=4.1304e6; /*  carrier Intermediate Frequency */

  /* number of samples per interrupt */
  long nsamp;
  int subsample3 = 0;
  int pack4 = 0;
  int socket_flag = 0;
  int tcp_port = 0;
  int opt;

  strncpy (IF_Filename, DEFAULT_FILENAME, 8);
  while ((opt = getopt(argc, argv, "3pf:t:ua")) != -1) {
    switch (opt) {
    case '3':
      subsample3 = 1;
      break;
    case 'p':
      pack4 = 1;
      break;
    case 'f':
      strncpy (IF_Filename, optarg, FILENAME_MAX);
      break;
     case 't':
       sscanf (optarg, "%d", &tcp_port);
       socket_flag = 1;
       break;
    case 'u':
      socket_flag = 1;
      break;
    case 'a':
/* for the GPS1A these are the parameters */
      samp_rate=16.367666666e6; /*  sampling rate */
      carrier_IF=4.123968e6; /*  carrier Intermediate Frequency */
      break;
    default: /* '?' */
      fprintf (stderr, "Usage: %s [-3] [-p] [-a] [-t TCP_port] [-u] [-f filename]\n",
	       argv[0]);
      exit (EXIT_FAILURE);
    }
  }

  if (subsample3) {
    samp_rate=samp_rate / 3.0;
    carrier_IF = samp_rate - carrier_IF;
    dop_sign = 1;
  }
  else
    dop_sign = -1;

  /* for Integer GP2021 emulation use 32 bits */
  Carrier_DCO_Delta = samp_rate / pow(2.0,32);
  Code_DCO_Delta    = samp_rate / pow(2.0,32);

  correlator_init (samp_rate, tic_period);
  tracking_init();
  acq_thresh = get_acq_thresh (samp_rate);
  if (!subsample3) acq_thresh*=3;

  /* number of samples per interrupt */
  nsamp = samp_rate * interr_int / 1.0e6;

  if (subsample3 & pack4)
    nsamp=(nsamp/12)*12;

  d_freq = (int) freq_bin_width / Carrier_DCO_Delta;
  carrier_ref_cycles = carrier_IF * tic_period * 1024;

  out=fopen("test.dat","w+");

  read_filenames();

  code_ref = 1023000.0/Code_DCO_Delta;
  carrier_ref = carrier_IF/Carrier_DCO_Delta;

  simtime=0.0;

  if (socket_flag) {
    int unit;
    struct sockaddr *sp = NULL;
    socklen_t slen = 0;
    struct sockaddr_in sin;
#ifndef WIN32
    struct sockaddr_un sun;
#endif

    if (tcp_port) {
      char host[256];
      struct hostent *hp;
	
      /* Open a TCP socket */
      if ((unit=socket(PF_INET, SOCK_STREAM, 0)) < 0) {
	perror ("Unable to open IP socket");
	exit (EXIT_FAILURE);
      }
	
      /* Start off assuming they want to run off the local host */
      strcpy (host, "localhost");
	
      /* If user set a file name assume it's a host name */
      if (strcmp (DEFAULT_FILENAME, IF_Filename) != 0)
	strcpy (host, IF_Filename);
	
      /* Look up host by name */
      if ((hp=gethostbyname(host)) == NULL) {
	fprintf (stderr, "Unable to find a DNS entry for a host named %s.\n",
		 host);
	exit (EXIT_FAILURE);
      }
	
      /* Set up the server internet socket address structure */
      memset ((char *) &sin, 0, sizeof(sin));
      sin.sin_family = PF_INET;
      sin.sin_port = htons(tcp_port);
#ifdef WIN32
      memcpy (&sin.sin_addr, hp->h_addr_list[0], 4);
#else
      if (inet_pton(PF_INET, hp->h_addr_list[0], &sin.sin_addr) < 0) {
	perror ("Unable to convert IP address");
	exit (EXIT_FAILURE);
      }
#endif
      sp = (struct sockaddr *) &sin;
      slen = sizeof(sin);
    }
#ifndef WIN32
    else { /* UNIX Domain Socket */
      /* Open a UNIX socket */
      if ((unit=socket(PF_UNIX, SOCK_STREAM, 0)) < 0) {
	perror ("Unable to open UNIX socket");
	exit (EXIT_FAILURE);
      }

      /* Set up the server UNIX domain socket address structure */
      memset ((char *) &sun, 0, sizeof(sun));
      sun.sun_family = PF_UNIX;
      strcpy (sun.sun_path, IF_Filename);
      slen = sizeof(sun.sun_family) + strlen(sun.sun_path);
      sp = (struct sockaddr *) &sun;
    }
#endif /* Not WIN32 */
    /* Attempt to connect to server */
    if (connect(unit, sp, slen) < 0) {
      perror ("Unable to connect");
      exit (EXIT_FAILURE);
    }

    /* Open stream so input will be buffered and we can use fread() */
    if ( (ifdata = fdopen (unit, "rb")) == NULL) {
      perror ("Unable to open socket stream");
      exit (EXIT_FAILURE);
    }

    /* Set time to computer's time */
    time (&utctime);
  }
  else {

    /* Open input from a standard file */
    if ((ifdata=fopen(IF_Filename,"rb")) == NULL) {
      fprintf (stderr, "Error: Unable to open IF file %s: %s\n",
	       IF_Filename, strerror (errno));
      fprintf (stderr, "Exiting...\n");
      exit (EXIT_FAILURE);
    }
      
    /* Set the time to modification time of the input file */
    status = stat(IF_Filename, &buffer);
    utctime=buffer.st_mtime ; /* mtime is at the end of the file             */
                              /* we need to adjust for the start of the file */
                              /* important when using large if files         */
    if (pack4) utctime-=buffer.st_size/samp_rate*4;
    else       utctime-=buffer.st_size/samp_rate;
  }

  tic_counter=0;

  read_rcvr_par ();

  rec_pos_xyz.x = 0.0;
  rec_pos_xyz.y = 0.0;
  rec_pos_xyz.z = 0.0;

  if (out_kalman == 1)
    kalm = fopen (kalman_file, "w+");
  if (out_rinex == 1)
    {
      rinex_obs = fopen (rinex_obs_file, "w+");
      rinex_nav = fopen (rinex_nav_file, "w+");
      read_rinex_par();
      write_rinex_obs_head = 1;
      write_rinex_nav_head = 1;
    }
  if (out_pos == 1 || out_vel == 1 || out_time == 1)
    output = fopen (output_file, "w+");
  if (out_pos == 1)
    fprintf (output, "time (seconds), latitude (degrees), "
 	     "longitude (degrees), hae (meters), ");
  if (out_vel == 1) 
    fprintf (output, "velocity north, velocity east, velocity up, ");
  if (out_time == 1)
    fprintf (output, "clock offset, ");
  if (out_pos || out_vel || out_time)
    fprintf (output, "hdop, vdop, tdop\n");

  if (out_debug == 1)
    debug = fopen (debug_file, "w+");
  if (out_data == 1)
    data_bits = fopen (data_bits_file, "w+");
  read_initial_data ();
  current_loc = receiver_loc ();
  rec_pos_llh.lon = current_loc.lon;
  rec_pos_llh.lat = current_loc.lat;
  rec_pos_llh.hae = current_loc.hae;
  nav_tic = nav_up;

  setup_term();

  if (status != cold_start)
    chan_allocate ();
  else if (status == cold_start)
    cold_allocate ();
  m_time[1] = clock_tow;

  /* Initialize IODE and IODC to the invalid value of -1. */
  for (i = 0; i <= 32; i++) {
    gps_eph[i].iode = -1;
    gps_eph[i].iodc = -1;
  }
  read_ephemeris ();
  {
    /* int err; */
    /*      open_com (0, Com0Baud, 0, 1, 8, &err); */       /* NMEA */
  } 
  while (!feof(ifdata))
    { 
      if ( pack4 ) {
	 /*  Simulate an interrupt every nsamp-les  */
	fread(&IFP,sizeof(char),nsamp/4,ifdata);
	unpack(IF,IFP,nsamp/4);
      }
      else 
	/*  Simulate an interrupt every nsamp-les  */
	fread(&IF,sizeof(char),nsamp,ifdata);

      simtime += interr_int / 1e6;
      Sim_GP2021_int(IF, nsamp);
      gpsisr();   /* Do the interrupt routine stuff (tracking loops etc.)   */
      if (idispcnt > 2000) {     /* display only about once per second to save time */
	if (display()!=0)
	  break;
	printf("time = %f\n",simtime);
	idispcnt = 0;
      }
      else
	idispcnt++;
      if (data_frame_ready == 1) 
	{
	  for (ch = 0; ch < N_channels; ch++) 
	    {
	      if ((ichan[ch].state == track) && ichan[ch].tow_sync) 
		{
		  /* decode the navigation message for this channel */
		  navmess (ichan[ch].prn, ch);
		}
	    }
	  if (out_data)
	    {
	      for (ch = 0; ch < N_channels; ch++) 
		{
		  wrdl = ichan[ch].prn;
		  fputc(wrdl,data_bits);
		}
	      for (i=0;i<1500;i++)
		{
		  wrd= data_message[i];
		  wrdh= (wrd & 0xff00) >> 8;
		  wrdl= wrd & 0xff;
		  fputc(wrdh,data_bits);
		  fputc(wrdl,data_bits);
		}
	      fprintf(data_bits,"\n");
	    }
	  data_frame_ready = 0;   
	}

      if (sec_flag == 1)
	{
	  /*          SendNMEA ();  */
	  nav_fix();  
	  /* Do the main routine stuff (display, position fix etc.) */
          utctime++;
          clock_tow = (clock_tow+1) % 604800L;
          time_on++;
          sec_flag = 0;
          for (ch = 0; ch < N_channels; ch++)
	    {
              if (ichan[ch].state == track)
		{
                  if (ichan[ch].CNo < 33)
		    {
                      /* calculate carrier clock and doppler correction */
		      long carrier_corr = -(xyz[ichan[ch].prn].doppler +
					   clock_offset * 1575.42)*dop_sign /
                        Carrier_DCO_Delta;
                      /* calculate code clock and doppler correction */
		      long code_corr =(xyz[ichan[ch].prn].doppler +
					   clock_offset * 1575.42) / 1540 /
                        Carrier_DCO_Delta;

		      setup_channel (ch, ichan[ch].prn, code_corr, carrier_corr);
		    }
		}
	    }
	}
        if (min_flag == 1)
       {
            chan_allocate();
            min_flag=0;
       }
    }


  restore_term ();

  /* Update the Almanac Data file */
  write_almanac ();

  /* Update the Ephemeris Data file */
  write_ephemeris ();

  /* Update the ionospheric model and UTC parameters */
  write_ion_utc ();

  /* Update the curloc file for the next run */
  if (status == navigating)
    {
      out = fopen (location_file, "w+");
      fprintf (out, "latitude  %f\n", rec_pos_llh.lat * r_to_d);
      fprintf (out, "longitude %f\n", rec_pos_llh.lon * r_to_d);
      fprintf (out, "hae       %f\n", rec_pos_llh.hae);
      fclose (out);
    }
  return 0;
}

/******************************************************************************
FUNCTION cold_allocate()
RETURNS  None.

PARAMETERS None.

PURPOSE To allocate the PRNs to channels for a cold start, start by
	  searching for PRN 1 through 12 and cycling through all PRN
	  numbers skipping channels that are tracking

WRITTEN BY
	Clifford Kelley

******************************************************************************/
void
cold_allocate ()
{
  int ch, i, alloc;
   /* search 200 Hz intervals */
  search_max_f = 50;            /* widen the search for a cold start */
  satfind (0);
  /* almanac_valid=1; */
  for (i = 1; i <= 32; i++)
    {
      if (gps_alm[i].inc > 0.0 && gps_alm[i].week != gps_week % 1024)
        almanac_valid = 0;
    }
  if (al0 == 0.0 && b0 == 0.0)
    almanac_valid = 0;
  for (ch = 0; ch < N_channels; ch++)   /* if no satellite is being tracked */
    /* turn the channel off */
    {
      if (ichan[ch].CNo < 30)  /* if C/No is too low turn the channel off */
	channel_off (ch);
    }
  for (i = 0; i <= chmax; i++)
    {
      alloc = 0;
      for (ch = 0; ch < N_channels; ch++)
        {
          if (ichan[ch].prn == cold_prn)
            {
              alloc = 1;        /* satellite is already allocated a channel */
	      cold_prn = cold_prn % 32 + 1;
              break;
            }
        }
      if (alloc == 0)           /* if not allocated find an empty channel */
        {
          for (ch = 0; ch < N_channels; ch++)
            {
              if (ichan[ch].state == off)
                {
                  long carrier_corr =  - clock_offset * 1575.42 * dop_sign / Carrier_DCO_Delta;
		          setup_channel (ch, cold_prn, 0, carrier_corr); /* No code correction */
                  cold_prn = cold_prn % 32 + 1;
                  break;
                }
            }
        }
    }
}

/******************************************************************************
FUNCTION read_rcvr_par(void)
RETURNS  None.

PARAMETERS None.

PURPOSE   To read in from the rcvr_par file the receiver parameters that
			control acquisition, tracking etc.

WRITTEN BY
	Clifford Kelley

******************************************************************************/
void
read_rcvr_par (void)
{
  char intext[40];
  if ((in = fopen (receiver_file, "r")) == NULL)
    {
      printf ("Cannot open rcvr_par.dat file.\n");
      exit (0);
    }
  else
    {
      fscanf (in, "%s %s", intext, tzstr);
      fscanf (in, "%s %f", intext, &mask_angle);
      mask_angle = mask_angle / r_to_d;
      fscanf (in, "%s %f", intext, &clock_offset);
      fscanf (in, "%s %u", intext, &interr_int);
      fscanf (in, "%s %d", intext, &cold_prn);
      fscanf (in, "%s %d", intext, &ICP_CTL);
      fscanf (in, "%s %f", intext, &nav_up);
      fscanf (in, "%s %d", intext, &out_pos);
      fscanf (in, "%s %d", intext, &out_vel);
      fscanf (in, "%s %d", intext, &out_time);
      fscanf (in, "%s %d", intext, &out_kalman);
      fscanf (in, "%s %d", intext, &out_debug);
      fscanf (in, "%s %d", intext, &out_data);
      fscanf (in, "%s %d", intext, &m_tropo);
      fscanf (in, "%s %d", intext, &m_iono);
      fscanf (in, "%s %d", intext, &align_t);
      fscanf (in, "%s %u", intext, &Com0Baud);  /* NMEA */
      fscanf (in, "%s %u", intext, &Com1Baud);  /* NMEA */
      fscanf (in, "%s %u", intext, &GPGGA);     /* NMEA */
      fscanf (in, "%s %u", intext, &GPGSV);     /* NMEA */
      fscanf (in, "%s %u", intext, &GPGSA);     /* NMEA */
      fscanf (in, "%s %u", intext, &GPVTG);     /* NMEA */
      fscanf (in, "%s %u", intext, &GPRMC);     /* NMEA */
      fscanf (in, "%s %u", intext, &GPZDA);     /* NMEA */
	  fscanf (in, "%s %d", intext, &out_rinex); /* RINEX data logging */
    }
  fclose (in);
}

/******************************************************************************
FUNCTION read_rinex_par(void)
RETURNS  None.

PARAMETERS None.

PURPOSE   Reads in the RINEX file parameters from the rinexpar.dat file

WRITTEN BY
	Jonathan J. Makela

******************************************************************************/
void
read_rinex_par (void)
{
  char intext[40];
  char *p;

  if ((in = fopen (rinex_par_file, "r")) == NULL)
    {
      printf ("Cannot open %s file.\n",rinex_par_file);
      exit (0);
    }
  else
    {
      fscanf (in, "%s %s\n", intext, system_type);
      fscanf (in, "%s %s\n", intext, program_name);
      fscanf (in, "%s %s\n", intext, agency_name);
      fscanf (in, "%s %s\n", intext, marker_name);
      fscanf (in, "%s\t", intext);
      fgets(observer_name, 20, in);
      /* Remove newline character if it is there */
      p = strchr(observer_name, '\n');
      if(p != NULL)
	*p = '\0';
      else
	fscanf (in, "%[ a-z A-Z 0-9]\n", intext);
      fscanf (in, "%s %s\n", intext, receiver_number);
      fscanf (in, "%s %s\n", intext, receiver_type);
      fscanf (in, "%s %s\n", intext, receiver_version);
      fscanf (in, "%s %s\n", intext, antenna_number);
      fscanf (in, "%s %s\n", intext, antenna_type);
      fscanf (in, "%s %lf\n", intext, &loc_x);
      fscanf (in, "%s %lf\n", intext, &loc_y);
      fscanf (in, "%s %lf\n", intext, &loc_z);
      fscanf (in, "%s %lf\n", intext, &delx);
      fscanf (in, "%s %lf\n", intext, &dely);
      fscanf (in, "%s %lf\n", intext, &delz);
      fscanf (in, "%s %d\n", intext, &lamda_factor_L1);
      fscanf (in, "%s %d\n", intext, &lamda_factor_L2);
      fscanf (in, "%s %d\n", intext, &n_obs);
      fscanf (in, "%s %s\n", intext, obs1);
      fscanf (in, "%s %s\n", intext, obs2);
      fscanf (in, "%s %s\n", intext, time_system);
    }
  fclose (in);
}


/******************************************************************************
FUNCTION chan_allocate()
RETURNS  None.

PARAMETERS None.

PURPOSE
	This function allocates the channels with PRN numbers

WRITTEN BY
	Clifford Kelley

******************************************************************************/

void
chan_allocate ()
{
  int ch, prnn, alloc;
  int i;
  /*       almanac_valid=1; */
  for (prnn = 1; prnn <= 32; prnn++)
    {
      xyz[prnn] = satfind (prnn);
      if (gps_alm[prnn].inc > 0.0 && gps_alm[prnn].week != gps_week % 1024)
        {
          almanac_valid = 0;
        }
    }
  if (al0 == 0.0 && b0 == 0.0)
    almanac_valid = 0;
  for (ch = 0; ch < N_channels; ch++)
    /* if the sat has dropped below mask angle turn the channel off */
    {
      if (ichan[ch].CNo < 30)
        {
          memset (&satellite[ichan[ch].prn], 0, sizeof(struct satellite));
	       channel_off (ch);
          for (i = 0; i < 5; i++)
            schan[ch].word_error[i] = 0;
        }
    }
  for (prnn = 1; prnn <= 32; prnn++)
    {
      if (xyz[prnn].elevation > mask_angle && gps_alm[prnn].health == 0 &&
          gps_alm[prnn].ety != 0.00)
        {
          alloc = 0;
          for (ch = 0; ch < N_channels; ch++)
            {
              if (ichan[ch].prn == prnn)
                {
                  alloc = 1;    /* satellite already allocated a channel */
                  break;
                }
            }
          if (alloc == 0)       /* if not allocated find an empty channel */
            {
	      for (ch = 0; ch < N_channels; ch++)
                {
                  if (ichan[ch].state == off)
                    {
                      /* calculate carrier clock and doppler correction */
                      long carrier_corr = (long)(-( xyz[prnn].doppler + clock_offset * 1575.42) * dop_sign /
                        Carrier_DCO_Delta);
                      /* calculate code clock and doppler correction */
                      long code_corr = (long)  ((xyz[prnn].doppler + clock_offset * 1575.42)
                                       / 1540 / Code_DCO_Delta );
 
		      setup_channel (ch, prnn, code_corr, carrier_corr);  
                      break;
                    }
                }
            }
        }
    }
}

int
pcifind (void)
{
  return 0;
}

void read_filenames(void)
{
  char txt1[40],txt2[40];
  FILE *filenames;
  filenames = fopen("SW_files.def","r");
  if (filenames == NULL) {
    fprintf (stderr, "Error: open file names file: %s\n", 
	     strerror (errno));
    exit (EXIT_FAILURE);
  }
  fscanf(filenames,"%s %s",txt1,txt2);
  sprintf(almanac_file,"%s",txt2);
  fscanf(filenames,"%s %s",txt1,txt2);
  sprintf(ephemeris_file,"%s",txt2);
  fscanf(filenames,"%s %s",txt1,txt2);
  sprintf(receiver_file,"%s",txt2);
  fscanf(filenames,"%s %s",txt1,txt2);
  sprintf(location_file,"%s",txt2);
  fscanf(filenames,"%s %s",txt1,txt2);
  sprintf(ion_utc_file,"%s",txt2);
  fscanf(filenames,"%s %s",txt1,txt2);
  sprintf(output_file,"%s",txt2);
  fscanf(filenames,"%s %s",txt1,txt2);
  sprintf(kalman_file,"%s",txt2);
  fscanf(filenames,"%s %s",txt1,txt2);
  sprintf(data_bits_file,"%s",txt2);
  fscanf(filenames,"%s %s",txt1,txt2);
  sprintf(rinex_par_file,"%s",txt2);
  fscanf(filenames,"%s %s",txt1,txt2);
  sprintf(rinex_obs_file,"%s",txt2);
  fscanf(filenames,"%s %s",txt1,txt2);
  sprintf(rinex_nav_file,"%s",txt2);
  fscanf(filenames,"%s %s",txt1,txt2);
  sprintf(debug_file,"%s",txt2);

  fclose(filenames);
}

static void 
unpack(char *IF,char *IFP,int nsamp)
{
  int i;
  const char val[4]={1,3,-1,-3};
  for (i = 0; i < nsamp; i++) {
    IF[(i<<2)  ] = val[ IFP[i]&0x3     ];
    IF[(i<<2)+1] = val[(IFP[i]&0xc )>>2];
    IF[(i<<2)+2] = val[(IFP[i]&0x30)>>4];
    IF[(i<<2)+3] = val[(IFP[i]&0xc0)>>6];
  }
}
