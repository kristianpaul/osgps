/***********************************************************************
  GPS RECEIVER (GPSRCVR) Ver. 2.10

  12 Channel All-in-View GPS Receiver Program based on Mitel GP2021
  chipset
  Clifford Kelley <cwkelley@earthlink.net>
  This program is licensed under GNU GENERAL PUBLIC LICENSE
  This LICENSE file must be included with the GPSRCVR code.

  I would like to thank the following people who have contributed to
  OpenSource GPS.  If I have left someone out I appologize and ask you
  to correct my oversight.

  Rick Niles
  Joel Barnes        University of New South Wales, Australia
  Jingrong Cheng     University of California Riverside
  Georg Beyerle      GFZ, Germany
  Alberto Perez
  Phil Bender
  Doug Baker
  Elmer Thomas       University of California Riverside
  Andrew Greenberg   Portland State University
  Takuji Ebinuma
  Serguei Miridonov
  Goetz Kappen


***********************************************************************/


#include  "serport.h"           /* NMEA */
#include  "nmea.h"              /* NMEA */
#include  <time.h>
#include  <stdio.h>
#include  <stdlib.h>
#include  <math.h>
#include  <string.h>

#ifdef __linux__
#include <termios.h>
#include <unistd.h>
#include <sys/io.h>   /* for iopl() */
#include <errno.h>
extern void restore_term (void);
#endif

#define MAIN
#include  "consts.h"
#include  "structs.h"
#include  "interfac.h"
#include  "gpsfuncs.h"

#include  "globals.h"
#include "rinex.h"
#undef  MAIN
FILE *output, *debug, *in, *out, *kalm, *data_bits;
FILE *rinex_obs, *rinex_nav, *rinex_par;
char output_file[40],almanac_file[40],ephemeris_file[40],receiver_file[40];
char location_file[40],ion_utc_file[40],data_bits_file[40],kalman_file[40];
char rinex_obs_file[40],rinex_nav_file[40],rinex_par_file[40],debug_file[40];
time_t utctime;

/* Internal functions */
static void read_rcvr_par (void);
static void display       (void);
static void chan_allocate (void);
static void cold_allocate (void);
static void read_rinex_par (void);
static void read_filenames (void);

/* External functions */
extern void nav_fix (void);
extern void Interrupt_Install (void);
extern void Interrupt_Remove  (void);

#if (defined VCPP)
#define clear_screen()              _clearscreen (_GCLEARSCREEN);
#define goto_screen_location(x, y)  _settextposition(x, y)
#define check_for_keyboard_press()  (kbhit() ? getch() : '\0')

#elif ((defined BCPP) || (defined __TURBOC__))

#include <conio.h> /* For getch()/kbhit() */
#define clear_screen()              clrscr ()
#define goto_screen_location(x, y)  gotoxy(x, y)
#define check_for_keyboard_press()  (kbhit() ? getch() : '\0')

#elif (defined __linux__)

/* Basic terminal escape sequence (note: ASCII 0x1b == ESC) */
#define clear_screen()              printf ("\n%c[2J", 0x1b);
#define goto_screen_location(x, y)  printf ("%c[%d;%dH", 0x1b, x, y)
#define check_for_keyboard_press()  getchar()

#endif


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
main (int argc, char *argv[])
{
  int ch;
  int i,wrd;
  char wrdl,wrdh;
  unsigned short int bldraddr = 0;

  /* Read the GPSBuilder2 address from the command line */
  if (argc == 2) {
    sscanf (argv[1], "bldraddr=%hx", &bldraddr);
  }
  data_frame_ready = 0;
  /* Find the card, and quit if we can not find it. */
  if (gp2021_detect (bldraddr)) {
    fprintf (stderr, "Could not find a GP2021 card.\n"
	     "If this is a GPSBuilder2 type card try:\n"
	     "%s bldraddr=0xNNN\n"
	     " where 0xNNN is the I/O address\n",
	     argv[0]);
    return EXIT_FAILURE;
  }

  /* Initialize the correlator */
  gp2021_init ();

  read_filenames();
  /* Read the rcvr_par.dat file */
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
  nav_tic = nav_up * 10;
/* program_TIC(TIC_cntr); */
  for (ch = 0; ch < N_channels; ch++)
    {
      for (i = 0; i < 5; i++)
        schan[ch].word_error[i] = 0;
    }
  time (&thetime); /* set up thetime so it can be taken over by this program */
  thetime += dtls;
#ifdef VCPP
  _setbkcolor (1);
  _displaycursor (_GCURSOROFF);


  /**
   * Get the current system time.  MS_TIME
   **/
  struct _dostime_t gDosTime;
  struct _dosdate_t gDosDate;
  _dos_gettime (&gDosTime);
  _dos_getdate (&gDosDate);

#endif

  clear_screen ();

  if (status != cold_start)
    chan_allocate ();
  else if (status == cold_start)
    cold_allocate ();
  m_time[1] = clock_tow;

  /* Initialize IODE and IODC to the invalid value of -1. */
  for (i = 0; i < 32; i++) {
    gps_eph[i].iode = -1;
    gps_eph[i].iodc = -1;
  }
  read_ephemeris ();
  {
    int err;
    open_com (0, Com0Baud, 0, 1, 8, &err);        /* NMEA */
  }

#ifndef __linux__
  Interrupt_Install ();
#endif

  do
    {
      check_for_new_data ();
      key = check_for_keyboard_press ();
      if (data_frame_ready == 1) {
	for (ch = 0; ch < N_channels; ch++) {
	  if ((ichan[ch].state == track) && ichan[ch].tow_sync) {
	    /* decode the navigation message for this channel */
	    navmess (ichan[ch].prn, ch);
	  }
	}
        if (out_data)
	  {
	    /* for (ch = 0;ch < N_channels; ch++)
	    {
		wrdl=ichan[ch].prn;
                fputc(wrdl,data_bits);
		}  */
	    for (i=0;i<1500;i++)
	    {
		wrd=data_message[i];
                wrdh=(wrd & 0xff00)>>8;
                wrdl=wrd & 0x0ff;
                fputc(wrdh,data_bits);
                fputc(wrdl,data_bits);
	    }
	  }
	data_frame_ready = 0;   
      }
      if (sec_flag == 1)
        {
          SendNMEA ();
          thetime++;
#if ((defined BCPP) || (defined __TURBOC__))
	  {
	    time_t utctime = thetime - dtls;
	    stime (&utctime);
	  }
#endif



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
                      long carrier_corr = (-xyz[ichan[ch].prn].doppler -
                                               clock_offset * 1575.42) /
                        42.57475e-3;
                      /* calculate code clock and doppler correction */
                      long code_corr =
                        clock_offset * 24. + xyz[ichan[ch].prn].doppler / 65.5;

		      setup_channel (ch, ichan[ch].prn, code_corr, carrier_corr);
                    }
                }
            }

        }
      /* nav fix once every X seconds */
      if (nav_flag == 1)
        {
          nav_fix ();
          nav_flag = 0;
        }
      /* channel allocation once every minute */
      if (min_flag == 1)
        {
          if (status != cold_start)
            chan_allocate ();
          else if (status == cold_start)
            cold_allocate ();
          min_flag = 0;
	  clear_screen ();
        }
      display ();
      if (key == 'p' || key == 'P')
        {
          display_page++;
          display_page = display_page % 5;
	  clear_screen ();
        }
      /* Allow the user to cycle backwards through the display pages */
      if (key == 'b' || key == 'B')
	{
	  display_page--;
	  display_page = (display_page + 5) % 5;
	  clear_screen();
	}
    }
  while (key != 'x' && key != 'X'); /*Stay in loop until 'X' key is pressed. */

#ifdef VCPP                     /* PGB */

  /**
   * There may be a better way????
   **/

  struct tm *TempTime;
  thetime -= dtls;
  TempTime = localtime (&thetime);

  /**
   * Update system time.
   **/
  gDosTime.hour = TempTime->tm_hour;
  gDosTime.minute = TempTime->tm_min;
  gDosTime.second = TempTime->tm_sec;

  gDosDate.month = TempTime->tm_mon + 1;
  gDosDate.day = TempTime->tm_mday;
  /**
   * tm_year is the current year - 1900
   **/
  gDosDate.year = 1900 + TempTime->tm_year;

  gDosDate.dayofweek = TempTime->tm_wday;

  _dos_settime (&gDosTime);     /*  TODO */
  _dos_setdate (&gDosDate);     /*  TODO */

#endif

#ifdef __linux__
  restore_term ();
#endif

  /**
   * Remove our interrupt and restore the old one
   **/
#ifndef __linux__
  Interrupt_Remove ();
#endif

  close_com ();               /* NMEA */

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

#ifdef DEBUG
  out=fopen(debug_file,"w+");
  fprintf(out,"carrier=%ld, code=%ld\n",store_carrier,store_code);
  for (ch=0;ch<6;ch++)
    {
      fprintf(out," ch=%d  offset=%d  \n",ch,chan[ch].offset);
      for ((int)i=0;i<1500;i++)
	{
	  fprintf(out," %d %d, %ld, %ld, %ld, %ld\n",
		  ch, i,qdither[ch][i],qprompt[ch][i],idither[ch][i],
		  iprompt[ch][i]);
	}
    }
  for (i=0;i<30000;i++)
    {
      fprintf(out," %d, %d, %d, %d, %d\n",
	      i,qdither0[i],qprompt0[i],idither0[i],iprompt0[i]);
    }
#endif /* DEBUG */
  
  /* fcloseall(); */
  return 0;
}


/******************************************************************************
FUNCTION display()
RETURNS  None.

PARAMETERS None.

PURPOSE
	This function displays the current status of the receiver on the
	computer screen.  It is called when there is nothing else to do

WRITTEN BY
	Clifford Kelley

******************************************************************************/
void
display (void)
{
  int ch;
  time_t utctime = thetime - dtls;
  goto_screen_location(1, 1);

  printf ("                   OpenSource GPS Software Version 2.30\n");
  printf ("%s", ctime (&utctime));
  printf ("TOW  %6ld\n", clock_tow);
  printf ("meas time %f  error %f  delta %f\n", m_time[1], m_error,
          delta_m_time);
  cur_lat.deg = rec_pos_llh.lat * r_to_d;
  cur_lat.min = (rec_pos_llh.lat * r_to_d - cur_lat.deg) * 60;
  cur_lat.sec =
    (rec_pos_llh.lat * r_to_d - cur_lat.deg - cur_lat.min / 60.) * 3600.;
  cur_long.deg = rec_pos_llh.lon * r_to_d;
  cur_long.min = (rec_pos_llh.lon * r_to_d - cur_long.deg) * 60;
  cur_long.sec =
    (rec_pos_llh.lon * r_to_d - cur_long.deg - cur_long.min / 60.) * 3600.;
  printf ("   latitude    longitude          HAE      clock error (ppm)\n");
  printf ("  %4d:%2d:%5.2f  %4d:%2d:%5.2f  %10.2f  %f\n",
          cur_lat.deg, abs (cur_lat.min), fabs (cur_lat.sec), cur_long.deg,
          abs (cur_long.min), fabs (cur_long.sec), rec_pos_llh.hae,
          clock_offset);
  printf (" Speed     Heading      TIC_dt\n");
  printf (" %f   %f   %f\n", speed, heading * r_to_d, TIC_dt);
  printf ("   \n");
  printf
    ("tracking %2d status %1d almanac valid %1d gps week %4d alm_page %2d\n",
     n_track, status, almanac_valid, gps_week % 1024, alm_page);
  if (display_page == 0)
    {
      printf
        (" ch prn state n_freq az el doppler t_count n_frame sfid ura page missed CNo\n");
      for (ch = 0; ch < N_channels; ch++)
        {
          printf
            (" %2d %2d  %2d  %3d   %4.0f  %3.0f   %6.0f   %4d  %4d  %2d  %3d  %3d%5d     %2d\n",
             ch, ichan[ch].prn, ichan[ch].state, ichan[ch].n_freq,
             xyz[ichan[ch].prn].azimuth * 57.3,
             xyz[ichan[ch].prn].elevation * 57.3, xyz[ichan[ch].prn].doppler,
             ichan[ch].frame_bit % 1500, ichan[ch].n_frame, ichan[ch].sfid,
             gps_eph[ichan[ch].prn].ura, schan[ch].page5, ichan[ch].missed,
             ichan[ch].CNo);
        }
      printf (" GDOP=%6.3f  HDOP=%6.3f  VDOP=%6.3f  TDOP=%6.3f\n", gdop, hdop,
	      vdop, tdop);
    }
  else if (display_page == 1)
    {
      printf (" ch prn state TLM      TOW  Health  Valid  TOW_sync offset\n");
      for (ch = 0; ch < N_channels; ch++)
        {
          printf (" %2d %2d  %2d  %6ld   %6ld   %2d     %2d     %2d   %4d\n",
                  ch, ichan[ch].prn, ichan[ch].state, ichan[ch].TLM,
                  ichan[ch].TOW, gps_eph[ichan[ch].prn].health,
                  gps_eph[ichan[ch].prn].valid, ichan[ch].tow_sync, 0);
        }
    }
  else if (display_page == 2)
    {
      printf (" ch prn state n_freq az  el        tropo        iono\n");
      for (ch = 0; ch < N_channels; ch++)
        {
          printf (" %2d %2d  %2d  %3d   %4.0f  %3.0f   %10.4f   %10.4f\n",
                  ch, ichan[ch].prn, ichan[ch].state, ichan[ch].n_freq,
                  xyz[ichan[ch].prn].azimuth * 57.3,
                  xyz[ichan[ch].prn].elevation * 57.3, 
		  satellite[ichan[ch].prn].Tropo * c,
                  satellite[ichan[ch].prn].Iono * c);
        }
    }
  else if (display_page == 3)
    {
      printf (" ch prn state      Pseudorange     delta Pseudorange\n");
      for (ch = 0; ch < N_channels; ch++)
        {
          printf (" %2d %2d  %2d  %20.10f   %15.10f\n",
                  ch, ichan[ch].prn, ichan[ch].state, 
		  satellite[ichan[ch].prn].Pr,
                  satellite[ichan[ch].prn].dPr);
        }
    }
  else if (display_page == 4)   /* can be used for debugging purposes */
    {
      printf (" ch prn state sfid page SF1  SF2  SF3  SF4  SF5\n");
      for (ch = 0; ch < N_channels; ch++)
        {
          printf (" %2d %2d   %2d   %2d  %2d  %3x  %3x  %3x  %3x  %3x\n",
                  ch, ichan[ch].prn, ichan[ch].state, ichan[ch].sfid,
                  schan[ch].page5, schan[ch].word_error[0],
                  schan[ch].word_error[1], schan[ch].word_error[2],
                  schan[ch].word_error[3], schan[ch].word_error[4]);
        }
    }

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
                      long carrier_corr = (-xyz[prnn].doppler -
					   clock_offset * 1575.42) /
                        42.57475e-3;
                      /* calculate code clock and doppler correction */
                      long code_corr =
                        clock_offset * 24. + xyz[prnn].doppler / 65.5;

		      setup_channel (ch, prnn, code_corr, carrier_corr);
                      break;
                    }
                }
            }
        }
    }
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
                  long carrier_corr = -clock_offset * 1575.42 / 42.57475e-3;
		  setup_channel (ch, cold_prn, 0, carrier_corr);
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
      printf ("Cannot %s file.\n",receiver_file);
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

void read_filenames(void)
{
  char txt1[40],txt2[40];
  FILE *filenames;
  filenames=fopen("HW_files.def","r+");
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
