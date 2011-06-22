/**
 * This is replacement module for interfac.c in the Userland linux
 * version
 *
 * This is a file in the OSGPS project covered by the GNU GPL.
 **/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/select.h>
#include <sys/time.h>
#include <unistd.h>
#include <errno.h>
#include <termios.h>

#include "interfac.h"

#define MAX(x,y) ((x) > (y) ? (x) : (y))

struct interface_channel ichan[N_channels];
struct debug_struct debug_u;
struct measurement_set measurements;
int ICP_CTL = DOP_ICP;

/* These three flags indicate that one second, one minute and one
   navigation TIC has occured respectively. */
int sec_flag, min_flag, nav_flag;

/* Navigation TIC period */
int nav_tic;

/**
 * Number of search bins 
 *
 * This should be moved to the ISR-only side, with maybe "hints" from
 * the user side.
 **/
int search_max_f = 20;

/* This is the internal representation of time */
time_t thetime;

/* This is time of the week to decide if the input stream is sane */
unsigned long clock_tow = 0;

int data_frame_ready;
uint16_t data_message[1500];

/* Linux-only variables */
int fd_status, fd_measurement, fd_data;
static struct termios oldt;

int
self_test (void)
{
  /* Stub */
  return 0;
}

/**
 * This is the standard method of getting a raw terminal and stores
 * the original terminal setting in a global variable that will be
 * restored before we exit.
 **/
void
setup_term (void)
{
  struct termios newt;
  tcgetattr( STDIN_FILENO, &oldt );
  newt = oldt;
  newt.c_lflag &= ~( ICANON | ECHO );
  newt.c_cc[VMIN]=0;
  tcsetattr( STDIN_FILENO, TCSANOW, &newt );
  
  setbuf(stdin, NULL);
}

void
restore_term (void)
{
 tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
}

/**
 * Linux specific setup
 *
 * (1) Open device files
 * (2) Get initial data
 * (3) Set up terminal
 **/
void
gp2021_init (void)
{
  /* open device files */
  if ((fd_status = open ("/dev/gps/status", O_RDWR)) == -1)
    perror ("open of GPS status device");
  if ((fd_measurement = open ("/dev/gps/measurement", O_RDWR)) == -1)
    perror ("open of GPS measurement device");
  if ((fd_data = open ("/dev/gps/data", O_RDONLY)) == -1)
    perror ("open of GPS data message device");
  if ((fd_status == -1) || (fd_measurement == -1) || (fd_data == -1))
    exit (EXIT_FAILURE);

  check_for_new_data ();

  setup_term ();
}

/**
 * Function to set a channel to a particular PRN and set an initial
 * Doppler shift.
 *
 * This is done by writing ASCII text to the status device file.
 **/
void
setup_channel (int ch, int prn,
               long code_corr __attribute__ ((unused)),
               long carrier_corr __attribute__ ((unused)))
{
  char outstr[80];

  sprintf (outstr, "%d %d %d\n", ch, prn, (int) carrier_corr * -42);
  write (fd_status, outstr, strlen (outstr));

  memset (&ichan[ch], 0, sizeof (struct interface_channel));
  ichan[ch].state = acquisition;
  ichan[ch].prn = prn;
}

/**
 * Turn off a channel.
 *
 * Set the particular channel to PRN 0, by sending a ASCII text
 * message to the status device file.
 **/
void
channel_off (int ch)
{
  char outstr[80];

  sprintf (outstr, "%d 0 0\n", ch);
  write (fd_status, outstr, strlen (outstr));

  memset (&ichan[ch], 0, sizeof (struct interface_channel));
}

/**
 * This is done in the device driver in Linux so this function is just
 * a stub.
 **/
int
gp2021_detect (unsigned short int address __attribute__ ((unused)))
{
  /* Stub */
  return 0;
}

/**
 * We don't actually need this, but it could be implemented if needed.
 * However, I'm considering removing this feature so I might as well
 * not implement it now.
 **/
void
set_TIC (long TIC_in __attribute__ ((unused)))
{
  /* Stub. */

  /* Need to write TIC_in to measurement device file as a two-byte
     (short) integer */
}

/**
 * Get the time-of-week (TOW) using the real-time clock.  Note this
 * function is repeated in the device driver.  We probably don't need
 * it in both places.
 **/
long
get_tow (void)
{
  int day_of_week;
  long days_from_1970;
  long today_seconds;
  const long SECS_PER_DAY = (3600 * 24);
  const long leap_seconds = 13;
  struct timeval tv;

  gettimeofday (&tv, NULL);

  days_from_1970 = (tv.tv_sec / SECS_PER_DAY);

  today_seconds = tv.tv_sec - days_from_1970 * SECS_PER_DAY;

  /* January 1, 1970 was a Thursday (day 4) */
  day_of_week = (days_from_1970 + 4) % 7;

  return day_of_week * SECS_PER_DAY + today_seconds + leap_seconds;
}


void
check_for_new_data (void)
{
  int i, n;
  fd_set readfds;
  struct timeval timeout;
  struct tracking_status status[N_channels];
  static int navcnt = 0;

  timeout.tv_sec = 0;
  timeout.tv_usec = 500000;

  FD_ZERO (&readfds);
  FD_SET (fd_measurement, &readfds);
  FD_SET (fd_data, &readfds);
  FD_SET (STDIN_FILENO, &readfds);

  /* The select() call will tell me which device files have data
     available. Note: the status device is left out since it's data is
     created on the fly and thus is always "ready". */
  n =
    select (MAX (fd_measurement, fd_data) + 1, &readfds, NULL, NULL,
            &timeout);

  /* Can we live without this? */
  clock_tow = get_tow ();

  if (n) {
    if (FD_ISSET (fd_measurement, &readfds)) {
      /* Read new measurement */
      memset (&measurements, 0, sizeof (struct measurement_set));
      read (fd_measurement, &measurements, sizeof (struct measurement_set));
      navcnt = (navcnt + 1) % 60;
      nav_flag = 1;
      sec_flag = 1;
      if (navcnt == 0) {
        min_flag = 1;
      }
    }
    if (FD_ISSET (fd_data, &readfds)) {
      extern int out_data;
      extern FILE *data_bits;
      /* Read new data message */
      read (fd_data, data_message, 1500 * sizeof (short));
      if (out_data)
	fwrite (data_message, 1, 1500 * sizeof (short), data_bits);
      data_frame_ready = 1;
    }
  }

  /* read status */
  read (fd_status, status, sizeof (struct tracking_status) * N_channels);

  for (i = 0; i < N_channels; i++) {
    ichan[i].prn      =  status[i].prn;
    ichan[i].state    =  status[i].stat & 0x7;
    ichan[i].tow_sync = (status[i].stat >> 3) & 0x1;
    ichan[i].sfid     = (status[i].stat >> 4) & 0x7;
    ichan[i].n_freq   =  status[i].n_freq;
    ichan[i].CNo      =  status[i].CNo;
    ichan[i].missed   =  status[i].missed;
    ichan[i].n_frame  =  status[i].n_frame;
  }
}

/**
 * NMEA serial port stuff
 **/
static int nmea_fd;
static struct termios original_nmea_term;

int
ComPortWrite (unsigned char *str,      /* string to send out com port */
              int NumberOfBytes)
{
  return (write (nmea_fd, str, NumberOfBytes) > 0);
}

void 
open_com (int Cport, unsigned baud, int parity, int stopbits,
               int numbits, int *err_code)
{
  char devname[80];
  struct termios nmea_term;

  sprintf (devname, "/dev/ttyS%d", Cport);
  sprintf (devname, "nmeatest.out");

  if ( (nmea_fd = open (devname, O_WRONLY | O_CREAT, 0644)) == -1 ) {
    fprintf (stderr, "Error opening NMEA device %s: %s\n", 
	     devname, strerror (errno));
    exit (EXIT_FAILURE);
  }

  tcgetattr ( nmea_fd, &original_nmea_term);
  nmea_term = original_nmea_term;

  /* Only doing the most common baud rates here... */
  switch (baud) {
  case 300:
    cfsetispeed( &nmea_term, B300);
    break;
  case 1200:
    cfsetispeed( &nmea_term, B1200);
    break;
  case 9600:
    cfsetispeed( &nmea_term, B9600);
    break;
  case 19200:
    cfsetispeed( &nmea_term, B19200);
    break;
  case 38400:
    cfsetispeed( &nmea_term, B38400);
    break;
  case 57600:
    cfsetispeed( &nmea_term, B57600);
    break;
  case 115200:
    cfsetispeed( &nmea_term, B115200);
    break;
  case 230400:
    cfsetispeed( &nmea_term, B230400);
    break;
  default:
    fprintf (stderr, "Error: unknown baud rate %d\n", baud);
    exit (EXIT_FAILURE);
  }
  switch (parity) {
  case 0:
    /* none */
    nmea_term.c_cflag |= ~PARENB;
    break;
  case 1:
    /* even */
    nmea_term.c_cflag |= PARENB;
    nmea_term.c_cflag &= ~PARODD;
    break;
  case 2:
    /* odd */
    nmea_term.c_cflag |= (PARENB & PARODD);
    break;
  default:
    fprintf (stderr, "Error: unknown parity: %d\n", parity);
    exit (EXIT_FAILURE);
  }

  if (stopbits == 2)
    nmea_term.c_cflag |= CSTOPB;
  else
    nmea_term.c_cflag &= ~CSTOPB;

  switch (numbits) {
  case 7:
    nmea_term.c_cflag &= ~CSIZE;
    nmea_term.c_cflag |= CS7;
    break;
  case 8:
    nmea_term.c_cflag &= ~CSIZE;
    nmea_term.c_cflag |= CS8;
    break;
  }

/*   cfsetattr (nmea_fd, TCSANOW, &nmea_term); */
  *err_code = 0;              /*Set error code to 0 */

}

void 
close_com (void)
{
  tcsetattr ( nmea_fd, TCSANOW, &original_nmea_term);
  close (nmea_fd);
}
