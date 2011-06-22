#include <stdio.h>
#include <time.h>
#include <math.h>
#include <stdlib.h>

#include "interfac.h"
#include "globals.h"
#include "consts.h"

extern time_t utctime;

#if (defined WIN32) /* Microsoft Visual C++ 2008 */
#include <windows.h>
void 
clear_screen(void)
{
  HANDLE hStdOut = GetStdHandle(STD_OUTPUT_HANDLE);
  COORD coord = {0, 0};
  DWORD count;

  CONSOLE_SCREEN_BUFFER_INFO csbi;
  GetConsoleScreenBufferInfo(hStdOut, &csbi);

  FillConsoleOutputCharacter(hStdOut, ' ', csbi.dwSize.X * csbi.dwSize.Y,
			     coord, &count);

  SetConsoleCursorPosition(hStdOut, coord);
}

void 
goto_screen_location(int x, int y)
{
  HANDLE hStdOut = GetStdHandle(STD_OUTPUT_HANDLE);
  COORD coord = {x, y};
  SetConsoleCursorPosition(hStdOut, coord);
}

#define check_for_keyboard_press()  (kbhit() ? getch() : '\0')
void setup_term() { clear_screen ()}
void restore_term() { clear_screen ()}

#elif (defined __GNUC__)
#include <termios.h>
#include <unistd.h>

/* Basic terminal escape sequence (note: ASCII 0x1b == ESC) */
#define clear_screen()              printf ("\n%c[2J", 0x1b);
#define goto_screen_location(x, y)  printf ("%c[%d;%dH", 0x1b, x, y)
#define check_for_keyboard_press()  getchar()
static struct termios oldt;

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
  clear_screen ();
}

void
restore_term (void)
{
  tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
}

#endif

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
int
display (void)
{
  int ch;
  /*   utctime -= dtls; */
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

  key = check_for_keyboard_press ();
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
  if (key == 0x1b) { /* ESC */
    if (check_for_keyboard_press () == '[') {
      key = check_for_keyboard_press ();
      if (key == '5') {
	display_page--;
	display_page = (display_page + 5) % 5;
	clear_screen ();
      }
      else {
	display_page++;
	display_page = display_page % 5;
	clear_screen ();
      }
    }
  }
  if (key == 'x' || key == 'X' || key == 'q' || key == 'Q') 
    return -1;

  return 0;
}
