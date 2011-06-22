/***********************************************************************
  GPS RECEIVER (GPSRCVR) Ver. 1.18
  See comments in GPSRCVR.CPP
  12 Channel All-in-View GPS Receiver Program based on Mitel GP2021
  chipset
  Clifford Kelley <cwkelley@earthlink.net>
  This program is licensed under GNU GENERAL PUBLIC LICENSE
  This LICENSE file must be included with the GPSRCVR code.
***********************************************************************/


#ifdef SOFT
# include "SoftOSGPS.h"
int inpwd(unsigned short int add)
{
  return (REG_read[add]);
}

void outpwd(unsigned short int add, unsigned short int data)
{
  REG_write[add]=data;
}
#else /* SOFT */
# ifdef __linux__
#  ifdef __KERNEL__
#   include <asm/io.h>
#  else /* __KERNEL__ */
/* #include <sys/io.h> */
#   include "SoftOSGPS.h"
#  endif /* __KERNEL__ */
# define outpwd(p, v) outw_p(v, p)
# define inpwd(p)     inw_p(p)
# else /* __linux__ */
#  include <dos.h>  /* For Port Memory I/O */
# endif /* __linux__ */
#endif /* SOFT */

extern int Base_address, Register_address, Data_address;
static int ch_status = 1;

void
to_gps (int add, int data)
{
  
#ifdef SOFT
  outpwd (add, data);
#else
  if (Base_address)
    outpwd (Base_address + ((add & 0xfc) << 8) + ((add & 0x3) << 3), data);
  else {
    outpwd (Register_address, add);
    outpwd (Data_address, data);
  }
#endif
}

short int
from_gps (int add)
{
#ifdef SOFT
  return inpwd (add);
#else
  if (Base_address)
    return (inpwd (Base_address + ((add & 0xfc) << 8 ) + ((add & 0x3) << 3 )));
  else {
    outpwd (Register_address, add);
    return (inpwd (Data_address));
  }
#endif
}

void
gp2021_latch (void)
{
  to_gps (0x80, 0);             /* tell 2021 to latch the correllators */
}

int
accum_status (void)
{
  return (from_gps (0x82));
}

short int
gp2021_missed (void)
{
  return from_gps(0x83);
}

unsigned int
ch_epoch_chk (int ch)
{
  return (from_gps ((ch << 3) + 7));
}

void
ch_cntl (int ch, int data)
{
  /*  printf("ch=%d  port=%x\n",ch,port(ch<<3)); */
  to_gps (ch << 3, data);
}


void
ch_accum_reset (int ch)
{
  to_gps ((ch << 2) + 0x85, 0);
}

void
ch_code_slew (int ch, int data)
{
  to_gps ((ch << 2) + 0x84, data);
}

void
program_TIC (long data)
{
  unsigned int high, low;
  high = ((int) (data >> 16));
  low = ((int) (data & 0xffff));
  to_gps (0x6d, high);
  to_gps (0x6f, low);
}

void
reset_cntl (int data)
{
  to_gps (0x7f, data);
  /*  fprintf(out,"reset data=%x\n",data); */
}

void
ch_carrier (int ch, long freq)
{
  int freq_hi, freq_lo;
  unsigned int add;
  freq_hi = ((int) (freq >> 16));
  freq_lo = ((int) (freq & 0xffff));
  add = (ch << 3) + 3;
  to_gps (add, freq_hi);
  add++;
  to_gps (add, freq_lo);
}

void
ch_code (int ch, long freq)
{
  int freq_hi, freq_lo;
  unsigned int add;
  freq_hi = (int) (freq >> 16);
  freq_lo = (int) (freq & 0xffff);
  add = (ch << 3) + 5;
  to_gps (add, freq_hi);
  add++;
  to_gps (add, freq_lo);
}

void
ch_epoch_load (int ch, unsigned int data)
{
  to_gps ((ch << 3) + 7, data);
}

void
ch_on (int ch)
{
  ch_status = ch_status | (0x1 << (ch+1));
  reset_cntl (ch_status);
}

void
system_setup (int data)
{
  to_gps (0x7e, data);
}

void
test_control (int data)
{
  to_gps (0x7c, data);
}

void
io_config (int data)
{
  to_gps (0xf0, data);
}

void
data_retent_w (int data)
{
  to_gps (0xe4, data);
}

int
data_retent_r (void)
{
  return (from_gps (0xe4));
}

void
data_bus_test_w (int data)
{
  to_gps (0xf2, data);
}

int
data_bus_test_r (void)
{
  return (from_gps (0xf2));
}

int
self_test (void)
{
  unsigned short int indataaax, indata55x, indataaay, indata55y, error;
  error = 0;
  data_retent_w (0x5500);
  data_bus_test_w (0xaa55);
  indata55x = data_retent_r ();
  indataaax = data_bus_test_r ();
  data_retent_w (0xaa00);
  data_bus_test_w (0x55aa);
  indataaay = data_retent_r ();
  indata55y = data_bus_test_r ();
  if ((indata55x != 0x5500) || (indataaax != 0xaa55)
      || (indataaay != 0xaa00) || (indata55y != 0x55aa))
    {
      error = 1;
/* #define DEBUG 1 */
#ifdef DEBUG
#include <stdio.h>
      printf ("data line error\n");
      printf ("indata55x=%x , indataaax=%x\n", indata55x, indataaax);
      printf ("indataaay=%x , indata55y=%x\n", indataaay, indata55y);
#endif
    }
  return error;
}

int
ch_carrier_DCO_phase (int ch)
{
  return (from_gps ((ch << 3) + 0x3));
}

long
ch_carrier_cycle (int ch)
{
  long result;
  result = from_gps ((ch << 3) + 6);
  result = result << 16;
  result = result + from_gps ((ch << 3) + 2);
  return (result);
}

int
ch_code_phase (int ch)
{
  return (from_gps ((ch << 3) + 0x1));
}

unsigned int
ch_epoch (int ch)
{
  return (from_gps ((ch << 3) + 4));
}

int
ch_code_DCO_phase (int ch)
{
  return (from_gps ((ch << 3) + 5));
}

int
ch_i_track (int ch)
{
  return (from_gps ((ch << 2) + 0x84));
}

int
ch_q_track (int ch)
{
  return (from_gps ((ch << 2) + 0x85));
}

int
ch_i_prompt (int ch)
{
  return (from_gps ((ch << 2) + 0x86));
}

int
ch_q_prompt (int ch)
{
  return (from_gps ((ch << 2) + 0x87));
}


#ifdef UNUSED
void
data_tst (int data)
{
  to_gps (0xf2, data);
}

void
ch_code_incr_hi (int ch, int data)
{
  to_gps ((ch << 3) + 0x5, data);
}

void
ch_code_incr_lo (int ch, int data)
{
  to_gps ((ch << 3) + 0x6, data);
}

void
carr_incr_hi (int ch, int data)
{
  to_gps ((ch << 3) + 0x3, data);
}

void
carr_incr_lo (char ch, int data)
{
  to_gps ((ch << 3) + 0x4, data);
}

void
all_cntl (int data)
{
  to_gps (0x70, data);

}

void
multi_cntl (int data)
{
  to_gps (0x60, data);

}

void
all_code_slew (int data)
{
  to_gps (0x70, data);
}

int
meas_status (void)
{
  return (from_gps (0x81));
}


void
ch_off (int ch)
{
  ch_status = ch_status & !(0x1 << (ch+1));
  reset_cntl (ch_status);
}

void
status_latch (void)
{
  to_gps (0x80, 0);
}

#endif /* UNUSED */
