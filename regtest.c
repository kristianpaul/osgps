#include  <stdio.h>
#include  <stdlib.h>
#include  <math.h>
#include  <string.h>

#ifdef __linux__
#include <termios.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>
#include <sys/io.h>
#define outpw(p, v) outw_p(v, p)
#define inpw(p)     inw_p(p)
#define gotoxy(x,y) (printf ("%c[%d;%dH", 0x1b,x,y))
#define clrscr()    (printf ("%c[2J", 0x1b))
#else
/* DOS Headers */
#include  <conio.h>
#include  <io.h>
#include  <Dos.h>
#endif

#ifdef __TURBOC__
#define outpw(p, v) outport(p, v)
#define inpw(p)     inport(p)
#endif

static void
to_gps (int add, int data)
{
  outpw (0x304, add);
  outpw (0x308, data);
} 

static int
from_gps (int add)
{
  outpw (0x304, add);
  return (inpw (0x308));
}

#ifdef __linux__
int 
set_raw_mode(void)
{
  int fd = 0;
  struct termios t;
  
  if (tcgetattr(fd, &t) < 0)
    {
      perror("tcgetattr");
      return -1;
    }
  
  t.c_lflag &= ~ICANON;
  
  if (tcsetattr(fd, TCSANOW, &t) < 0)
    {
      perror("tcsetattr");
      return -1;
	}
  
  setbuf(stdin, NULL);
  
  return 0;
}

static void
delay (int n) {
   struct timespec req;
   req.tv_sec = 0;
   req.tv_nsec = 1000000L * (long) n;
   nanosleep (&req, NULL);
 }

#endif


static int
check_for_keyboard_press (void)
{
  int key = '\0';

#ifdef __linux__
#include <sys/poll.h>
#include <unistd.h>
  {
    struct pollfd ufds;
    char c;

    ufds.fd = 0;                /* stdin */
    ufds.events = POLLIN;
    ufds.revents = 0;
    poll (&ufds, 1, 0);
    if (ufds.revents & POLLIN)
      {
        read (0, &c, 1);
        key = (int) c;
      }
  }
#else
  if (kbhit ())
    key = getch ();
#endif

  return key;
}

int
main (void)
{
  unsigned char addlo, addhi, address;
  int key, i, amin, amax;
  unsigned int result, cycle;
  FILE *error_file;
  unsigned short int data[256] = { 
    0xa3f6, 0x0000, 0x0000, 0x01f7, 0xb1b9, 0x016e, 0xa4a8, 0x0000,/*0 cntl 0*/
    0xa3ec, 0x0000, 0x0000, 0x01f7, 0xb1b9, 0x016e, 0xa4a8, 0x0000,/*0 cntl 1*/
    0xa3d8, 0x0000, 0x0000, 0x01f7, 0xb1b9, 0x016e, 0xa4a8, 0x0000,/*1 cntl 2*/
    0xa3b0, 0x0000, 0x0000, 0x01f7, 0xb1b9, 0x016e, 0xa4a8, 0x0000,/*1 cntl 3*/
    0xa04b, 0x0000, 0x0000, 0x01f7, 0xb1b9, 0x016e, 0xa4a8, 0x0000,/*2 cntl 4*/
    0xa096, 0x0000, 0x0000, 0x01f7, 0xb1b9, 0x016e, 0xa4a8, 0x0000,/*2 cntl 5*/
    0xa2cb, 0x0000, 0x0000, 0x01f7, 0xb1b9, 0x016e, 0xa4a8, 0x0000,/*3 cntl 6*/
    0xa196, 0x0000, 0x0000, 0x01f7, 0xb1b9, 0x016e, 0xa4a8, 0x0000,/*3 cntl 7*/
    0xa32c, 0x0000, 0x0000, 0x01f7, 0xb1b9, 0x016e, 0xa4a8, 0x0000,/*4 cntl 8*/
    0xa3ba, 0x0000, 0x0000, 0x01f7, 0xb1b9, 0x016e, 0xa4a8, 0x0000,/*4 cntl 9*/
    0xa374, 0x0000, 0x0000, 0x01f7, 0xb1b9, 0x016e, 0xa4a8, 0x0000,/*5 cntl10*/
    0xa1d0, 0x0000, 0x0000, 0x01f7, 0xb1b9, 0x016e, 0xa4a8, 0x0000,/*5 cntl11*/
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,/*6 cntl Multi */
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0008, 0x0000, 0xb823,/*6 TIC */
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,/*7 cntl All */
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x1fff,/*7 reset */
    0x0000, 0x0000, 0x0000, 0x0000,	/*8 Status */
    0x0000, 0x0000, 0x0000, 0x0000,	/*8 accum 0 */
    0x0000, 0x0000, 0x0000, 0x0000,	/*8 accum 1 */
    0x0000, 0x0000, 0x0000, 0x0000,	/*8 accum 2 */
    0x0000, 0x0000, 0x0000, 0x0000,	/*9 accum 3 */
    0x0000, 0x0000, 0x0000, 0x0000,	/*9 accum 4 */
    0x0000, 0x0000, 0x0000, 0x0000,	/*9 accum 5 */
    0x0000, 0x0000, 0x0000, 0x0000,	/*9 accum 6 */
    0x0000, 0x0000, 0x0000, 0x0000,	/*a accum 7 */
    0x0000, 0x0000, 0x0000, 0x0000,	/*a accum 8 */
    0x0000, 0x0000, 0x0000, 0x0000,	/*a accum 9 */
    0x0000, 0x0000, 0x0000, 0x0000,	/*a accum 10 */
    0x0000, 0x0000, 0x0000, 0x0000,	/*b accum 11 */
    0x0000, 0x0000, 0x0000, 0x0000,	/*b accum Multi */
    0x0000, 0x0000, 0x0000, 0x0000,	/*b accum ALL */
    0x0000, 0x0000, 0x0000, 0x0000,	/*b */
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,/*c RTC */
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,/*c RTC */
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,/*d DUART */
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,/*d DUART */
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,/*e System cntl */
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,/*e */
    0x0301, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,/*f General cntl */
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000
  };				/*f */
  error_file = fopen ("reg_err.dat", "w+");

#ifdef __linux__
  /* Enable access to raw ioports (requires root permission) */
  if (iopl (3) != 0) {
    fprintf (stderr, "Unable to access I/O ports: %s\n", strerror (errno));
    exit (EXIT_FAILURE);
  }

  /* Set terminal for raw mode */
  set_raw_mode();
#endif

  clrscr ();
  to_gps (0x7f, 0);		/*reset control */
  delay (100);
  to_gps (0x7f, 0x1fff);	/*reset control */
  to_gps (0xf0, 0x0301);	/*io config */
  to_gps (0x7c, 0);		/*test control */
  to_gps (0x7e, 0x200);		/*system setup */
  cycle = 1;
  amin = 0;
  amax = 255;
  for (i = amin; i <= amax; i++) {
    if (i < 0x70 || i > 0x77)
      to_gps (i, data[i]);
  }
  do {
    key = check_for_keyboard_press ();
    gotoxy (1, 3);
    printf ("cycle %d\n", cycle);
    to_gps (0x80, 0);		/* latch status bits */
    for (addhi = 0; addhi < 16; addhi++) {
      for (addlo = 0; addlo < 16; addlo++) {
	result = 0;
	address = addhi * 16 + addlo;
	result = from_gps (address);
	printf ("%4x ", result);
      }
      printf ("\n");
    }
    data[0xf2]++;
    to_gps (0xf2, data[0xf2]);	/* data bus test */
    data[0xe4]++;
    to_gps (0xe4, data[0xe4]);	/* data retent */
    result = from_gps (0xf2);
    if (result != data[0xf2]) {
      printf ("data bus problem  w=%4x r=%4x\n", data[0xf2], result);
      fprintf (error_file, "data bus problem  w=%4x r=%4x\n", data[0xf2],
	       result);
    }
    if (data[0xf2] == 0xffff) {
      fprintf (error_file, "completed cycle %d\n", cycle);
      cycle++;
    }
  } while (key != 'x' && key != 'X');	/*Stay in loop until 'X' key is pressed. */

  return 0;
}
