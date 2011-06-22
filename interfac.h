#ifndef __KERNEL__
#include <time.h>
#endif


#define N_channels 12
#include "types.h"

#if 0
#ifdef SOFT
#define CARRIER_REF_FREQ   0x5c855daL   /* for SiGe use 4.1304 MHz IF */
/* #define CARRIER_REF_CYCLES 422952537L */
#define CARRIER_REF_CYCLES 135727787L
#define DOP_SIGN  1
#else
#define CARRIER_REF_FREQ   0x1f7b1b9L   /* for GP2021 use 1.4054 MHz IF */
#define CARRIER_REF_CYCLES 143912491L
#define DOP_SIGN  1
#endif
#endif

extern int dop_sign;

/*double Carrier_DCO_Delta, Code_DCO_Delta;

float Bnpullcarr,Bnpullcode;
float Bntrkcarr,Bntrkcode;

int trk_carr_C1,trk_carr_C2;
int trk_code_C1,trk_code_C2;
int pull_carr_C1,pull_carr_C2;
int pull_code_C1,pull_code_C2;
*/

enum
{ off, acquisition, confirm, pull_in, track };
/*     0       1         2       3      4   */

/* XXX FIX ME XXX: This structure should go away someday when we have
   proper interfaces for everything. */
struct interface_channel
{
  /* Status Information */
  int prn;
  int state;                    /* Tracking state */
  char tow_sync;                /* A ready for nav. solution flag */
  char frame_ready;             /* semaphore for data message */
  char debug_ready;             /* semaphore for debug data */
  int frame_bit;                /* bit counter */
  int n_frame;                  /* informational only */
  long TOW, TLM;                /* informational only */
  int sfid;                     /* informational only */
  int missed;                   /* informational only */
  int n_freq;                   /* informational only */

  /* Replace me with SNR or CNo */
  char CNo;
};

struct measurement
{
  int32_t transmit_time_offset; /* seconds * 1024 * 2046 * 1000 */
  int32_t doppler_prn;          /* Doppler in 23 MSBs, PRN is 8 LSBs */
};

#define MEASUREMENT_BOW_INVALID 0xffffffff
struct measurement_set
{
  /* Number of bits since the beginning of the week */
  uint32_t bit_of_week;

  /* interval time / 175 nanoseconds */
  int32_t i_TIC_dt;

  struct measurement measurement[N_channels];
};

struct tracking_status
{
  uint8_t prn;
  uint8_t stat;                 /* state & (tow_sync << 3) & (sfid << 4) */
  int8_t n_freq;
  uint8_t CNo;
  uint16_t missed;
  uint16_t n_frame;
};

struct debug_struct {
  /* Add debug tmp stuff here... */
  int struct_can_not_be_empty;
};
extern struct debug_struct debug_data;
extern int debug_ready; 

extern struct interface_channel ichan[N_channels];
extern struct measurement_set measurements;
extern int sec_flag, min_flag, nav_flag;
extern int search_max_f;
extern int nav_tic;

extern time_t thetime;
extern unsigned long clock_tow;

extern int Base_address;
extern int Register_address;
extern int Data_address;

extern uint16_t data_message[1500];

extern int data_frame_ready;
enum
{ DOP_CTL, DOP_ICP };
extern int ICP_CTL;

/* Detect the GP2021 Card */
extern int gp2021_detect (unsigned short bldraddr);

/* General start up set up */
extern void gp2021_init (void);

/* (re-) Initialize channel structures */
extern void channel_init (int ch);

/* Set the code frequency on the DCO */
extern void set_code_freq (int ch, long code_corr);

/* Set the carrier frequency on the DCO */
extern void set_carrier_freq (int ch, long carrier_corr);

/* Set the PRN of a specific channel and call the
   set_code/carrier_freq functions */
extern void setup_channel (int ch, int prn, long code_corr,
                           long carrier_corr);

/* Call gp2021_init() and then idle the channel */
extern void channel_off (int ch);

/* Check if new data is available (needed for Linux version) */
extern void check_for_new_data (void);

/* Set the TIC period (not used) */
extern void set_TIC (long TIC_in);

