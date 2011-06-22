#include "interfac.h"
#include "gp2021.h"
#include "gpsisr.h"

extern long code_ref, carrier_ref;
extern int  d_freq;
int dop_sign = 1;

struct debug_struct debug_data;
int debug_ready;

struct interface_channel ichan[N_channels];
struct measurement_set measurements;
int ICP_CTL = DOP_ICP;

/* These three flags indicate that one second, one minute and one
   navigation TIC has occured respectively. */
int sec_flag, min_flag, nav_flag;

/* Navigation TIC period (warning there's a few places in the code
   that current depend on this being exactly 10.) */
int nav_tic = 10;

/**
 * Number of search bins 
 *
 * This should be moved to the ISR-only side, with maybe "hints" from
 * the user side.
 **/
int search_max_f = 5;

/* This is time of the week to decide if the input stream is sane */
unsigned long clock_tow = 0;

/* These are the ISA defaults, but will get overridden by a call to
   set_gp2021_address() */
int Base_address     = 0;
int Register_address = 0x304;
int Data_address     = 0x308;

int data_frame_ready;
uint16_t data_message[1500];

void
gp2021_init (void)
{
  io_config (0x301);
  test_control (0);
  system_setup (0);
  reset_cntl (0x0);
  reset_cntl (0x1fff);
}

void
set_TIC (long TIC_in)
{
  long TIC_cntr = TIC_in;
  program_TIC (TIC_cntr);
}

void
channel_init (int ch)
{
  chan[ch].t_count      = 0;
  chan[ch].codes        = 0;
  chan[ch].bit_counter  = 0;
  chan[ch].del_freq     = 1;
  chan[ch].offset       = 0;

  ichan[ch].state     = acquisition;
  ichan[ch].n_frame   = 0;
  ichan[ch].tow_sync  = 0;
  ichan[ch].n_freq    = 0;
  ichan[ch].frame_bit = 0;
  ichan[ch].CNo       = 0;
  ichan[ch].sfid      = 0;
  ichan[ch].TOW = ichan[ch].TLM = 0;
}


static void
set_PRN (int ch, int PRN)
{
  const int prn_code[38] =
    { 0, 0x3f6, 0x3ec, 0x3d8, 0x3b0, 0x04b, 0x096, 0x2cb, 0x196, 0x32c,
      0x3ba, 0x374, 0x1d0, 0x3a0, 0x340, 0x280, 0x100, 0x113, 0x226,
      0x04c, 0x098, 0x130, 0x260, 0x267, 0x338, 0x270, 0x0e0, 0x1c0,
      0x380, 0x22b, 0x056, 0x0ac, 0x158, 0x2b0, 0x058, 0x18b, 0x316, 0x058
    };
  const int waas_code[19] = 
    { 0x2c4, 0x30a, 0x1da, 0x0b2, 0x3e3, 0x0f8, 0x25f, 0x1e7, 0x2b5,
      0x22a, 0x10e, 0x12d, 0x215, 0x337, 0x0c7, 0x0e2, 0x20f, 0x3c0,
      0x29 };

  const int GIC_code[11] = 
    { 0x2c4, 0x10a, 0x3e3, 0x0f8, 0x25f, 0x1e7, 0x2b5, 0x000, 0x10e};

  ichan[ch].prn = PRN;

  if ( PRN < 37 ) {
    /* 0xa000 for late select satellite */
    ch_cntl (ch, prn_code[PRN] | 0xa000);
  }
  else if ( (PRN > 119) && (PRN < 138) ) {
    /* 0xa000 for late select satellite */
    ch_cntl (ch, waas_code[PRN-120] | 0xa000);
  }
  else if ( ( PRN > 200) && (PRN < 212) ) {
    /* 0xa000 for late select satellite */
    ch_cntl (ch, GIC_code[PRN-201] | 0xa000);
  }
  else {
    /* ERROR: unknown PRN */
  }
    
}


/* XXX FIX ME XXX: Change the input to this to some standard units */
void
set_code_freq (int ch, long code_corr)
{
  chan[ch].code_freq = code_ref + code_corr;

  chan[ch].code_corr = code_corr;

  /* 1.023 MHz chipping rate */
  ch_code (ch, chan[ch].code_freq); 
}

/* XXX FIX ME XXX: Change the input to this to some standard units */
void
set_carrier_freq (int ch, long carrier_corr)
{

  chan[ch].carrier_freq = carrier_ref  /*CARRIER_REF_FREQ */
    + carrier_corr + d_freq * ichan[ch].n_freq;

  chan[ch].carrier_corr = carrier_corr;

  /* select carrier */
  ch_carrier (ch, chan[ch].carrier_freq);
}

void
setup_channel (int ch, int prn, long code_corr, long carrier_corr)
{
  if (prn != ichan[ch].prn)
    channel_off (ch);
  channel_init (ch);
  set_PRN (ch, prn);
  set_code_freq (ch, code_corr);
  set_carrier_freq (ch, carrier_corr);
  ch_on (ch);
}

void
channel_off (int ch)
{
  /* Re-init to flush all data */
  channel_init (ch);

  /* Set to PRN 0 */
  set_PRN (ch, 0);

  /* Set state to "off" */
  ichan[ch].state = off;
}

/* Turbo C version does not currently support PCI */
#ifdef __TURBOC__
int pcifind(void) {return 0;}
#endif

int
gp2021_detect (unsigned short bldraddr)
{
  extern int self_test (void);
  extern int pcifind (void);
  
  /* If the user explicitly gives a bldr2 address use it. */
  if (bldraddr) {
    Base_address = bldraddr;
    
    if (self_test () != 0)
      return -1;
    
  }
  else {
    /* Auto-detect */
    unsigned short int io_port;

    /* First check for PCI card */
    if ((io_port = pcifind ()) == 0) {
      /* if not found assume we have an ISA interface @ 0x300 */
      io_port = 0x300;
    }

    Register_address = io_port + 4;
    Data_address     = io_port + 8;


    /* If the self_test fails, try Base_address = io_port, for bldr2 card */
    if (self_test () != 0) {
      Base_address = io_port;

      if (self_test () != 0)
	return -1;
    }
  }

  return 0;
}


void
check_for_new_data (void)
{
  /* Stub */
}
