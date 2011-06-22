#ifdef __KERNEL__
#include <linux/kernel.h>
#include <linux/string.h>
#else
#include <stdlib.h>             /* for abs() only */
#include <string.h>
#endif /* __KERNEL__ */

#include  "gp2021.h"
#include  "interfac.h"
#include  "gpsisr.h"

int d_freq = 4698;
long carrier_ref, code_ref = 0x016ea4a8L;
long carrier_ref_cycles = 143912491L;

float Bnpullcarr,Bnpullcode;
float Bntrkcarr,Bntrkcode;

int trk_carr_C1 = 241, trk_carr_C2 = 8;
int trk_code_C1 =  61, trk_code_C2 = 2;
int pull_carr_C1 = 241,  pull_carr_C2 = 8;
int pull_code_C1 = 1515, pull_code_C2 = 51;
long TIC_ref = 571427L;
time_t thetime;
/******** Tracking constants for 2nd order tracking loops */

/*  Variable definitions:                                      */
/*   On   = Natural frequency (Hz)                             */
/*   zeta = Damping coefficient (unitless)                     */
/*   BWn  = Noise bandwidth (Hz)                               */
/*   Ts   = Integration time (s)                               */
/*   Ko   = Discriminator gain  chip (code) /cycle (carrier)   */
/*        = 1 for code loop                                    */
/*        = 2*pi for carrier loop                              */
/*   Kn   = NCO resolution (Hz)                                */
/*        = 0.042575 Hz or 40e6/7/2^27                         */

/*  Equations:                                                 */
/*   On = 2*zeta*BWn/(4*zeta^2+1)                              */
/*   C1 = 1/(Ko*Kn)*8*zeta*On*Ts/(4+4*zeta*On*Ts+(On*Ts)^2)    */
/*   C2 = 1/(Ko*Kn)*4*(On*Ts)^2/(4+4*zeta*On*Ts+(On*Ts)^2)     */

/*  For pull-in state Ts = 1 ms             */
/*  carrier BWn = 25 Hz, zeta = 0.7071      */
/*  code BWn = 25 Hz, zeta = 0.7071         */
/*#define pull_carr_C1  241
#define pull_carr_C2    8
#define pull_code_C1 1515
#define pull_code_C2   51 */
#define pull_carr_F0 1680
/*  For track state code Ts is 20 ms        */
/*  code BWn = 1 Hz, zeta = 0.7071          */
/*  For track state carrier Ts = 1 ms       */
/*  carrier BWn = 25 Hz, zeta = 0.7071      */
/*#define trk_carr_C1   241
#define trk_carr_C2     8
#define trk_code_C1    61
#define trk_code_C2     2 */

int acq_thresh = 650;

#define pull_in_time 1500
#define phase_test    500
#define n_of_m_thresh   8
#define confirm_m      10
#define cc_scale     1540
#define rms           312
/********************************************/

#define sign(x) (x > 0 ? 1 : (x == 0) ? 0 : -1)
#define bsign(x) (x > 0 ? 1 : 0)

#ifndef __KERNEL__
#define  test_bit(bit_n, data) ((*((uint16_t *)(data))) &   (0x1 << (bit_n)))
#define   set_bit(bit_n, data) ((*((uint16_t *)(data))) |=  (0x1 << (bit_n)))
#define clear_bit(bit_n, data) ((*((uint16_t *)(data))) &= ~(0x1 << (bit_n)))
#endif


struct tracking_channel chan[N_channels];

/*long TIC_cntr = TIC_ref;*/
static int nav_count, tic_count = 0, hms_count = 0;
static uint16_t astat, mstat, a_missed;
static uint16_t data_overflow[16];
static uint16_t data_mesg[1500];
long ch_0_track_counter=0;

static void pream (int, char);
static void ch_acq (int);
static void ch_confirm (int);
static void ch_pull_in (int);
static void ch_track (int);

/******************************************************************************
FUNCTION rss(long a, long b)
RETURNS  long integer

PARAMETERS
      a  long integer
      b  long integer

PURPOSE
	This function finds the fixed point magnitude of a 2 dimensional vector

WRITTEN BY
	Clifford Kelley

******************************************************************************/

static long
rss (long a, long b)
{
  long result, c, d;
  c = abs (a);
  d = abs (b);
  if (c == 0 && d == 0)
    result = 0;
  else {
    if (c > d)
      result = (d >> 1) + c;
    else
      result = (c >> 1) + d;
  }
  return (result);
}

/******************************************************************************
FUNCTION fix_sqrt(long x)
RETURNS  long integer

PARAMETERS
      x long integer

PURPOSE
	This function finds the fixed point square root of a long integer

WRITTEN BY
	Clifford Kelley

******************************************************************************/

static long
fix_sqrt (long x)
{
  long xt, scr;
  int i;
  i = 0;
  xt = x;
  do {
    xt = xt >> 1;
    i++;
  }
  while (xt > 0);
  i = (i >> 1) + 1;
  xt = x >> i;
  do {
    scr = xt * xt;
    scr = x - scr;
    scr = scr >> 1;
    scr = scr / xt;
    xt = scr + xt;
  }
  while (scr != 0);
  xt = xt << 7;
  return (xt);
}

/******************************************************************************
FUNCTION fix_atan2(long y,long x)
RETURNS  long integer

PARAMETERS 
		x  long   in-phase fixed point value
		y  long   quadrature fixed point value

PURPOSE
      This function computes the fixed point arctangent represented by
      x and y in the parameter list
      1 radian = 16384
      based on the power series  f-f^3*2/9

WRITTEN BY
	Clifford Kelley
	Fixed for y==x added special code for x==0 suggested by Joel
	Barnes, UNSW
******************************************************************************/


static long
fix_atan2 (long y, long x)
{
  static long const SCALED_PI_ON_2 = 25736L;
  static long const SCALED_PI = 51472L;
  long result = 0, n, n3;
  if ((x == 0) && (y == 0))
    return (0);                 /* invalid case */

  if (x > 0 && x >= abs (y)) {
    n = (y << 14) / x;
    n3 = ((((n * n) / 16384) * n) / 8192) / 9;
    result = n - n3;
  }
  else if (x <= 0 && -x >= abs (y)) {
    n = (y << 14) / x;
    n3 = ((((n * n) / 16384) * n) / 8192) / 9;
    if (y > 0)
      result = n - n3 + SCALED_PI;
    else if (y <= 0)
      result = n - n3 - SCALED_PI;
  }
  else if (y > 0 && y > abs (x)) {
    n = (x << 14) / y;
    n3 = ((((n * n) / 16384) * n) / 8192) / 9;
    result = SCALED_PI_ON_2 - n + n3;
  }
  else if (y < 0 && -y > abs (x)) {
    n = (x << 14) / y;
    n3 = ((((n * n) / 16384) * n) / 8192) / 9;
    result = -n + n3 - SCALED_PI_ON_2;
  }
  return (result);
}

static unsigned char
compute_CNo (unsigned long avg)
{
   /**
    * CNo = 10.0*log10( (signal/noise)^2/2*Bandwidth)
    *
    * 1395=sqrt(20)*312 or the noise sigma for an integration interval
    * of 20 ms
    *
    * 25 of course is 50 Hz divided by 2
    *
    * 1.7777 = (1/0.75)^2 which accounts for the fact that the
    * correlator values are not at the peak but nominally 75% of the
    * peak.
    *
    * CNo = 10. * log10 (avg / 1395. *
    *                    avg / 1395. * 25. * 1.7777);
    * or
    *
    * CNo =  6 * log2 ( sqrt(5) * avg / 468 );
    **/

  const unsigned long CNo_table[60] = {
    1, 249, 280, 314, 352, 395, 443, 497,
    557, 625, 702, 787, 883, 991, 1112, 1247,
    1399, 1570, 1762, 1976, 2217, 2488, 2792, 3132,
    3514, 3943, 4424, 4964, 5569, 6249, 7011, 7867,
    8826, 9903, 11112, 12467, 13989, 15695, 17611, 19759,
    22170, 24875, 27911, 31316, 35137, 39425, 44235, 49632,
    55688, 62483, 70107, 78662, 88260, 99029, 111112, 124670,
    139882, 156950, 176101, 197589
  };
  int i;
  unsigned char CNo = 0;

  CNo = 60;
  for (i = 0; i < 61; i++) {
    if (avg < CNo_table[i]) {
      CNo = i;
      break;
    }
  }

  return CNo;
}


static int
compute_transmit_time (struct tracking_channel *c, uint32_t * bow, 
		       struct measurement *meas)
{
  char bit;
  unsigned long int ms, chip, phase;
  int good = 0;
  int meas_bit_time_rem;
  long meas_bit_time_offset;
  long corrected_meas_bit_time;

  meas_bit_time_offset = 0;
  ms = c->epoch & 0x1f;
  chip = c->code_phase;
  phase = c->code_dco_phase;
  bit = c->epoch >> 8;

  meas_bit_time_rem = c->meas_bit_time % 50;
  if (meas_bit_time_rem == (bit + 1) % 50)
    meas_bit_time_offset = -1;
  if ((meas_bit_time_rem + 1) % 50 == bit)
    meas_bit_time_offset = +1;

  corrected_meas_bit_time = c->meas_bit_time + meas_bit_time_offset;

  good = 0;
  if ((corrected_meas_bit_time % 50 == bit)
      && (phase < 1024)
      && (chip < 2046)) {

    if (*bow == MEASUREMENT_BOW_INVALID)
      *bow = corrected_meas_bit_time;

    /**
     * 1024 phases / half chips
     * 2046 half chips / ms 
     * 1000 ms / sec
     * 50   data bits / sec
     * 
     *  1024 * 2046 * 1000 / 50 = 41902080 phases / data bit
     **/
    meas->transmit_time_offset =
      (corrected_meas_bit_time - *bow) * 41902080L
      + ((2046L * ms + chip) << 10) + phase;
    good = 1;
  }

  return good;
}

void
cold_allocate2 (void)
{
  int ch, i, taken;
  static int cold_prn = 1;

  search_max_f = 50;            /* widen the search for a cold start */
  reset_cntl (0x1fff);

  for (ch = 0; ch < N_channels; ch++) {
    /* if C/No is too low turn the channel off */
    /* avg = 6617 is equivalent of C/No of 30 dB */
    if (chan[ch].avg < 6617) {
      channel_off (ch);
    }
    else if ((ichan[ch].tow_sync == 0) && (ichan[ch].state == 4)) {
      ichan[ch].state = 3;
      chan[ch].ms_set = 0;

    }
  }

  for (ch = 0; ch < N_channels; ch++) {

    if (ichan[ch].state == off) {
      do {
        taken = 0;
        /* Make sure this PRN is not already used */
        for (i = 0; i < N_channels; i++) {
          if (ichan[i].prn == cold_prn) {
            cold_prn = cold_prn % 31 + 1;
            taken = 1;
            break;
          }
        }
      } while (taken);

      setup_channel (ch, cold_prn, 0, 0);
      cold_prn = cold_prn % 31 + 1;

    }
  }
}



/******************************************************************************
FUNCTION GPS_Interrupt()

RETURNS  None.

PARAMETERS None.

PURPOSE
	This function replaces the current IRQ0 Interrupt service
	routine with our GPS function which will perform the
	acquisition - tracking functions

WRITTEN BY
	Clifford Kelley

******************************************************************************/
void
gpsisr (void)
{
  /* int astat,mstat; */
  int ch;

  gp2021_latch ();
  a_missed = gp2021_missed ();   /* did we miss any corellation data */
  astat = accum_status ();    /* get info on what channels have data ready */
  for (ch = 0; ch < N_channels; ch++) {
    struct tracking_channel *c = &chan[ch];
    if (test_bit (ch, (void *) &astat)) {
      c->accum.i_dith   = ch_i_track (ch);   /* inphase dither */
      c->accum.q_dith   = ch_q_track (ch);   /* quadrature dither */
      c->accum.i_prompt = ch_i_prompt (ch);  /* inphase prompt */
      c->accum.q_prompt = ch_q_prompt (ch);  /* quadrature prompt */

#ifndef SOFT
      /* Check for bogus (h/w error) ready flag */
      if (memcmp (&c->accum, &c->prev_accum,
		  sizeof(struct accum))==0)
	clear_bit (ch, (void *) &astat);
      else
	c->prev_accum = c->accum;
#endif

      if (test_bit (ch, (void *) &a_missed)) {
        ichan[ch].missed++;
        ch_accum_reset (ch);
      }
    }
  }
  for (ch = 0; ch < N_channels; ch++) {
    if (test_bit (ch, (void *) &astat)) {
      switch (ichan[ch].state) {
      case acquisition:
        ch_acq (ch);
        break;
      case confirm:
        ch_confirm (ch);
        break;
      case pull_in:
        ch_pull_in (ch);
        break;
      case track:
        ch_track (ch);
        break;
      }
    }
  }
  mstat = a_missed & 0x2000;    /* has a tic occured? */
  if (mstat) {
    tic_count = (tic_count + 1) % 10;
    if (tic_count == 0)
      sec_flag = 1;             /* one second has passed */
    hms_count = (hms_count + 1) % 600;
    if (hms_count == 0)
      min_flag = 1;             /* one minute has passed */
    nav_count = (nav_count + 1) % nav_tic;
    if (nav_count == 0)
      nav_flag = 1;
    for (ch = 0; ch < N_channels; ch++) {
      struct tracking_channel *c = &chan[ch];
      /* get carrier data for computing delta-pseudorange */
      c->carr_dco_phase = ch_carrier_DCO_phase (ch);
      c->cycle_sum = ch_carrier_cycle (ch);
    }

    if (nav_count == 0) {       /* time to set up measurement data */
      int n = 0;
      uint32_t *bow = &measurements.bit_of_week;
      memset (&measurements, 0, sizeof (struct measurement_set));
      *bow = MEASUREMENT_BOW_INVALID;
      for (ch = 0; ch < N_channels; ch++) {
        long d_carr_phase;
        struct measurement *meas = &measurements.measurement[n];
	struct interface_channel *ic = &ichan[ch];
	struct tracking_channel *c = &chan[ch];

        /* get code data for computing pseudorange */
        c->code_phase = ch_code_phase (ch);
        c->epoch = ch_epoch (ch);
        c->code_dco_phase = ch_code_DCO_phase (ch);
        c->meas_bit_time = c->tr_bit_time;
        c->carrier_counter = c->cycle_sum;

        d_carr_phase = c->carr_dco_phase - c->old_carr_dco_phase;
        chan[ch].old_carr_dco_phase = chan[ch].carr_dco_phase;
	if ( (ic->state == track) &&
	     (ic->tow_sync == 1) &&
	     (ic->CNo > 33) )
        if (compute_transmit_time (c, bow, meas)) {
	    /**
	     * Let
	     * Zero Doppler setting (ZDS) = 
	     * -(1540 * 1023000 Hz                     [GPS L1]
	     *   - 40 MHz * (7*5 + 7/2 + 7/9)          [Front-End]
	     *   - 40 MHz / 7 )                        [Correlator Frequency]
	     **/
          /* XXX FIX ME XXX: This Doppler math assumes nav_tic = 10 */
          if (ICP_CTL == DOP_CTL)
            /* This integer math will put the carrier setting
               into carrier_counter like counts. */
	    /**
	     * ZDS 
	     *   / (40 MHz / 7 / 2^27)       [Carrier Frequency Scale Factor]
	     *  = 33010105
	     *
	     * 31250 / 7 = (40 MHz / 7) / 128 / 10    [nav_tic=10]
	     **/
            meas->doppler_prn =
              ((c->carrier_freq - carrier_ref) * 31250L / 7)
              & 0xfffffe00L;
          else                  /* ICP_CTL == DOP_ICP */
	    /**
	     *  counter zero = 
	     *  ZDS * 0.9999990 * 1024 / 10           [nav_tic=10]
	     *  = 143912491
	     **/
            meas->doppler_prn = ((c->carrier_counter << 10)
				 + (d_carr_phase) - carrier_ref_cycles) * 1024L;


          /* add the PRN in the LSBs */
          meas->doppler_prn |= ic->prn - 1;
          n++;
        }
      }
      measurements.i_TIC_dt = TIC_ref;  /*  hopefully not a problem  */
    }
    for (ch = 0; ch < N_channels; ch++) {
      struct tracking_channel *c = &chan[ch];
      c->old_carr_dco_phase = c->carr_dco_phase;
    }
  }
}

/******************************************************************************
FUNCTION ch_acq(char ch)
RETURNS  None.

PARAMETERS
			ch  char

PURPOSE  to perform initial acquisition by searching code and frequency space
			looking for a high correllation

WRITTEN BY
	Clifford Kelley

******************************************************************************/

static void
ch_acq (int ch)
{
  long prompt_mag, dith_mag;
  struct interface_channel *ic = &ichan[ch];
  struct tracking_channel *c = &chan[ch];
  if (abs (ic->n_freq) <= search_max_f) { /*  search frequencies */
    prompt_mag = rss (c->accum.i_prompt, c->accum.q_prompt);
    dith_mag = rss (c->accum.i_dith, c->accum.q_dith);
    if ((dith_mag > acq_thresh) && (prompt_mag > acq_thresh)) {
      ch_code_slew (ch, 2044);  /* slew back 1 chip so we can */
      ic->state = confirm;        /* confirm the signal */
      c->i_confirm = 0;
      c->n_thresh = 0;
    }
    else {
      ch_code_slew (ch, 2);
      c->codes += 2;
    }
    if (c->codes == 2044) {
      ic->n_freq += c->del_freq;
      c->del_freq = -(c->del_freq + sign (c->del_freq));
      set_carrier_freq (ch, c->carrier_corr);
      c->codes = 0;
    }
  }
  else {
    ic->n_freq = 0;       /* keep looping */
    c->del_freq = 1;
    set_carrier_freq (ch, c->carrier_corr);
    c->codes = 0;
  }
  ic->CNo = 0;
}

/******************************************************************************
FUNCTION ch_confirm(char ch)
RETURNS  None.

PARAMETERS
			ch  char  channel number

PURPOSE  to confirm the presence of a high correllation peak using an n of m
			algorithm

WRITTEN BY
	Clifford Kelley

******************************************************************************/
static void
ch_confirm (int ch)
{
  long prompt_mag, dith_mag;
  struct interface_channel *ic = &ichan[ch];
  struct tracking_channel *c = &chan[ch];
  prompt_mag = rss (c->accum.i_prompt, c->accum.q_prompt);
  dith_mag = rss (c->accum.i_dith, c->accum.q_dith);
  if ((prompt_mag > acq_thresh) || (dith_mag > acq_thresh))
    c->n_thresh++;
  if (c->i_confirm == confirm_m) {
    if (c->n_thresh >= n_of_m_thresh) {
      ic->state = pull_in;
      ic->CNo = 0;
      c->ch_time = 0;
      c->sum = 0;
      c->th_rms = 0;
      c->ms_set = 0;
      c->code_int = c->code_freq;
      c->phase_int = c->carrier_freq;
    }
    else
      ic->state = acquisition;
  }
  c->i_confirm++;
}

/******************************************************************************
FUNCTION ch_pull_in(char ch)
RETURNS  None.

PARAMETERS
			ch  char  channel number

PURPOSE     
           pull in the frequency by trying to track the signal with a
	   combination FLL and PLL it will attempt to track for xxx
	   ms, the last xxx ms of data will be gathered to determine
	   if we have both code and carrier lock if so we will
	   transition to track

WRITTEN BY
	Clifford Kelley

******************************************************************************/
static void
ch_pull_in (int ch)
{
  long dcode, ddcar, theta_e, wdot_gain;
  long q_sum, i_sum, theta, theta_dot;
  long prompt_mag, dith_mag;
  struct interface_channel *ic = &ichan[ch];
  struct tracking_channel *c = &chan[ch];
  prompt_mag = rss (c->accum.i_prompt, c->accum.q_prompt);
  dith_mag = rss (c->accum.i_dith, c->accum.q_dith);
  c->sum += prompt_mag + dith_mag;
  /* code tracking loop */
  if (prompt_mag != 0 || dith_mag != 0) {
    dcode = ((prompt_mag - dith_mag) * 8192) / (prompt_mag + dith_mag);
    c->code_int += ((dcode*pull_code_C2) / 8192);
    c->code_freq = c->code_int + ((dcode*pull_code_C1) /8192);
    ch_code (ch, c->code_freq);
  }
  c->dfreq1 = c->dfreq;
  q_sum = c->accum.q_dith + c->accum.q_prompt;
  i_sum = c->accum.i_dith + c->accum.i_prompt;
  if (i_sum != 0 || q_sum != 0)
    theta = fix_atan2 (q_sum, -i_sum);
  else
    theta = c->old_theta;
  theta_dot = theta - c->old_theta;
  c->ms_count++;
  /* check if last 20ms have the same sign */
  if ((sign (q_sum) == -1 && (c->ms_sign & 0xfffff) == 0x00000) ||
      (sign (q_sum) == 1 && (c->ms_sign & 0xfffff) == 0xfffff)) {
    if (sign (q_sum) == -sign (c->old_q_sum) 
	&& test_bit (ch, (void *) &a_missed) == 0 
	&& abs (abs (theta) - 25736) < 4096 
	&& abs (abs (c->old_theta) - 25736) < 4096) {
      /* We have detected the *//* start if a data bit */
      c->ms_count = 0;    /* set up counters to keep */
      ch_epoch_load (ch, 0x1);  /* track of them */
      c->ms_set = 1;      /* data bit set */
    }
  }
  c->ms_sign = c->ms_sign << 1;     /* shift sign left */
  if (q_sum < 0)
    c->ms_sign = c->ms_sign | 0x1; /* set bit to 1 if negative */
  c->old_theta = theta;
  if (theta > 0)
    theta_e = theta - 25736;
  else if (theta <= 0)
    theta_e = theta + 25736;
  if (c->ch_time > pull_in_time - phase_test)
    c->th_rms += (theta_e * theta_e) / 16384;
  if (abs (theta_dot) < 32768L) {
    if (q_sum != 0 || i_sum != 0) {
      wdot_gain = c->ch_time / 499;
      wdot_gain *= wdot_gain;
      wdot_gain *= wdot_gain;
      ddcar =  theta_dot * pull_carr_F0 / (1 + wdot_gain);
      c->phase_int += ((-theta_e*pull_carr_C2 - ddcar) / 16384);
      c->carrier_freq = c->phase_int + 
	               + ((-theta_e*pull_carr_C1) / 16384);
  /* don't close the loop for 2 ms to avoid transients  */
      if (c->ch_time > 2) {
        ch_carrier (ch, c->carrier_freq);
      }
    }
  }
  c->dcarr1 = c->dcarr;
  c->old_q_sum = q_sum;
  c->ch_time++;
  c->ms_count = c->ms_count % 20;
  /* done with pull-in wait until end of a data bit */
  if (c->ch_time >= pull_in_time && c->ms_count == 19) {
    c->avg = c->sum / pull_in_time / 2;
    c->th_rms = fix_sqrt (c->th_rms / phase_test);
    if (c->avg > 14 * rms / 10 
	&& c->th_rms < 12000 
	&& c->ms_set) {   /* go to track */
      c->avg = c->avg * 20;
      ic->CNo = compute_CNo (c->avg);
      ic->state = track;
      c->t_count = 0;
      c->sum = 0;
      c->q_dith_20 = 0;
      c->q_prom_20 = 0;
      c->i_dith_20 = 0;
      c->i_prom_20 = 0;
      c->bit_counter = 0;
      c->dfreq = 0;
      c->phase_int = 0;
      c->code_int  = code_ref;
      c->phase_int = c->carrier_freq;
      ic->debug_ready=0;
    }
    else {
      ic->state=acquisition;
      set_code_freq(ch,c->code_corr);
    }
  }
  if (c->code_freq < (code_ref-25000)) { /* go back to acquisition */
      /*  if code freq gets much smaller than it ever should */
      /*  this prevents getting caught in state 3 */
      ic->state=acquisition;
      set_code_freq (ch, c->code_corr);
    }
  
}

/******************************************************************************
FUNCTION ch_track(char ch)
RETURNS  None.

PARAMETERS
			ch  char  channel number

PURPOSE  to track in carrier and code the GPS satellite and partially
			decode the navigation message (to determine
			TOW, subframe etc.)

WRITTEN BY
	Clifford Kelley
          added Carrier Aiding as suggested by Jenna Cheng, UCR
******************************************************************************/
static void
ch_track (int ch)
{
  int i, ext_bit_count;
  long dcode, dcarr, q_sum, i_sum;
  struct interface_channel *ic = &ichan[ch];
  struct tracking_channel *c = &chan[ch];
  /**
   *  1000 Hz carrier tracking loop
   **/
  q_sum = c->accum.q_dith+c->accum.q_prompt;
  i_sum = c->accum.i_dith+c->accum.i_prompt;
  if (q_sum != 0 || i_sum!= 0){
    dcarr=-(i_sum * 1024)*sign(q_sum)/rss(q_sum,i_sum);
    c->phase_int+=(dcarr*trk_carr_C2 / 1024);
    c->carrier_freq=(dcarr*trk_carr_C1 / 1024)+c->phase_int;
    ch_carrier(ch,c->carrier_freq);
  }
  /**
   * 50 Hz code tracking loop
   **/
  c->ms_count = (++c->ms_count) % 20;
  c->q_dith_20 += c->accum.q_dith;
  c->q_prom_20 += c->accum.q_prompt;
  c->i_dith_20 += c->accum.i_dith;
  c->i_prom_20 += c->accum.i_prompt;
  if (c->ms_count == 19) {
    int bitnum;
       /**    q_sum = c->q_prom_20+c->q_dith_20;
    i_sum = c->i_prom_20+c->i_dith_20;
    if (q_sum != 0 || i_sum != 0) {
    dcarr=-(i_sum * 1024)*sign(q_sum)/rss(q_sum,i_sum);
    c->phase_int+=(dcarr*trk_carr_C2 / 1024);
    c->carrier_freq=(dcarr*trk_carr_C1 / 1024)+c->phase_int;
    ch_carrier(ch,c->carrier_freq);
    } **/
    c->tr_bit_time++;
    c->prompt_mag = rss (c->i_prom_20, c->q_prom_20);
    c->dith_mag = rss (c->i_dith_20, c->q_dith_20);
    c->sum += c->prompt_mag + c->dith_mag;
    /* code tracking loop */
    if (c->prompt_mag != 0 || c->dith_mag != 0) {

      /* without carrier aiding */
      /* c->dfreq=(c->prompt_mag-c->dith_mag)*trk_code_k;
	 ddf=(c->dfreq-c->dfreq1)*trk_code_d;
	 c->code_freq+=(c->dfreq+ddf)/trk_div;
      */

      /* with carrier aiding */
      dcode = ((c->prompt_mag - c->dith_mag) * 2048) /
        (c->prompt_mag + c->dith_mag);
      c->code_int += ((dcode*trk_code_C2) / 2048);

      c->code_freq = c->code_int +(( dcode*trk_code_C1) / 2048)
	+ dop_sign * (carrier_ref - c->carrier_freq) / cc_scale;

      ch_code (ch, c->code_freq);
    }
    c->dfreq1 = c->dfreq;

    /* bsign is data bit */
    c->bit = bsign (q_sum);
    pream (ch, c->bit);   /* see if we can find the preamble */
    /* If we haven't gotten sync in 5 minutes drop channel */
    /*    if ((ic->tow_sync) && (thetime - c->tow_sync_time > 300)){
      ic->tow_sync = 0;
      ic->state = acquisition;
      }*/
    bitnum = (c->t_count - c->offset + 1500) % 1500;
    if ((ic->frame_ready) && (bitnum < 16)) {
      /* Overflow */
      if (c->bit)
        set_bit (ch, (void *) &data_overflow[bitnum]);
      else
        clear_bit (ch, (void *) &data_overflow[bitnum]);
    }
    else {
      if (c->bit)
        set_bit (ch, (void *) &data_mesg[bitnum]);
      else
        clear_bit (ch, (void *) &data_mesg[bitnum]);
    }

    ic->frame_bit = c->bit_counter - c->offset;

    ext_bit_count = -1;
    for (i = 0; i < N_channels; i++) {
      if (ichan[i].frame_bit > ext_bit_count)
        ext_bit_count = ichan[i].frame_bit;
    }
    if (ic->frame_bit - ext_bit_count > 1400
        && ic->frame_bit - ext_bit_count < 1600) {
      c->bit_counter -= 1500;
      c->bit_counter = (c->bit_counter + 3000) % 3000;
    }

    if (ext_bit_count - ic->frame_bit > 1400
        && ext_bit_count - ic->frame_bit < 1600) {
      c->bit_counter += 1500;
      c->bit_counter = c->bit_counter % 3000;
    }

    if (ic->frame_bit < 0)
      ic->frame_bit += 3000;
    c->bit_counter++;
    if (c->bit_counter >= 3000)
      c->bit_counter = 0;

    c->t_count++;
    if (c->t_count % 5 == 0) {
      c->avg = c->sum / 10;
      ic->CNo = compute_CNo (c->avg);
      c->sum = 0;
    }
    c->q_dith_20 = 0;
    c->q_prom_20 = 0;
    c->i_dith_20 = 0;
    c->i_prom_20 = 0;
  }
  if (c->t_count == 1500) {
    ic->n_frame++;
    c->t_count = 0;
  }
}

/* Fast Parity for 32-bit integer */
static unsigned int 
Parity (uint32_t w)
{
  /**
   * parity of the number of 1-bit
   * 0 if even; 1 if odd
   * implementation for 32 bit words
   **/
  w ^= w>>1;
  w ^= w>>2;
  w ^= w>>4;
  w ^= w>>8;
  w ^= w>>16;
  return w & 1;
}

/******************************************************************************
FUNCTION pream(char ch, char bit)
RETURNS  None.

PARAMETERS

	ch   channel number (0-11)
	bit  the latest data bit from the satellite

PURPOSE 
        This function finds the preamble in the navigation message and
	synchronizes to the nav message

WRITTEN BY
	Clifford Kelley
made changes suggested by Jenna Cheng, UCR to make routine more readable
******************************************************************************/
static void
pream (int ch, char bit)
{
  static unsigned long pream = 0x22c00000L;
  unsigned long parity0, parity1;
  static unsigned long pb1 = 0xbb1f3480L, pb2 = 0x5d8f9a40L, pb3 =
    0xaec7cd00L;
  static unsigned long pb4 = 0x5763e680L, pb5 = 0x6bb1f340L, pb6 =
    0x8b7a89c0L;
  unsigned long TOWs, HOW, TLM, TLMs;
  int sfid_s;
  struct interface_channel *ic = &ichan[ch];
  struct tracking_channel *c = &chan[ch];

  if (c->fifo1 & 0x20000000L) {
    c->fifo0 = (c->fifo0 << 1) + 1;
  }
  else {
    c->fifo0 = c->fifo0 << 1;
  }

  c->fifo1 = (c->fifo1 << 1) + bit;

  if (c->fifo0 & 0x40000000L) {
    TLM = c->fifo0 ^ 0x3fffffc0L;
  }
  else {
    TLM = c->fifo0;
  }

  if (c->fifo1 & 0x40000000L) {
    HOW = c->fifo1 ^ 0x3fffffc0L;
  }
  else {
    HOW = c->fifo1;
  }
  /*ic->sfid = 1;  for debugging purposes*/
  if (((pream ^ TLM) & 0x3fc00000L) == 0) {     /* preamble pattern found? */
    /*ic->sfid = 2;  preamble found */
    parity0 = (Parity (TLM & pb1) << 5) + (Parity (TLM & pb2) << 4) +
      (Parity (TLM & pb3) << 3) + (Parity (TLM & pb4) << 2) +
      (Parity (TLM & pb5) << 1) + (Parity (TLM & pb6));

    if (parity0 == (TLM & 0x3f)) {      /* is parity of TLM ok? */
      parity1 = (Parity (HOW & pb1) << 5) + (Parity (HOW & pb2) << 4) +
        (Parity (HOW & pb3) << 3) + (Parity (HOW & pb4) << 2) +
        (Parity (HOW & pb5) << 1) + (Parity (HOW & pb6));
      /* ic->sfid =3;  TLM parity is ok */
      if (parity1 == (HOW & 0x3f)) {    /* is parity of HOW ok? */
        long d_tow;
        /* ic->sfid = 4; HOW parity is ok */
	/* compute the subframe id number */
        sfid_s = (int) ((HOW & 0x700) >> 8);
        TLMs = (TLM >> 2) & 0x3fff;
	TOWs = (HOW & 0x3fffffffL) >> 13;
	if ((sfid_s > 0) && (sfid_s < 6) && (TOWs < 100800)) {
	  d_tow = clock_tow - TOWs * 6 + 5;  /* 5 sec because TOWs is next subframe */
	  c->offset = c->t_count - 59 - (sfid_s - 1) * 300;
	  ch_epoch_load (ch, (0x1f & ch_epoch_chk (ch)) | 0xa00); /* cwk */
	  if (c->offset < 0)
	    c->offset += 1500;
	  c->tr_bit_time = TOWs * 300 - 240;
	  ic->TOW = TOWs * 6;
	  ic->tow_sync = 1;
	  c->tow_sync_time =- d_tow;
	  clock_tow = TOWs * 6 - 5;
	  ic->sfid = sfid_s;
	  ic->TLM = TLMs;
	}
      }
    }
  }
  /*  a 1500 bit frame of data is ready to be read */
  if ((((c->t_count - c->offset + 1500) % 1500) == 0)
      && ic->tow_sync) {
    int i, ready = 1, gather_cnt = 0;

    /* another channel is done */
    ic->frame_ready = 1;

    /* check to see if we've done all that are sync'd */
    for (i = 0; i < N_channels; i++) {
      if ((ichan[i].state == track) && ichan[i].tow_sync
          && !ichan[i].frame_ready) {
        ready = 0;
        break;
      }
      gather_cnt++;
    }

    if (ready && (gather_cnt > 0) && (data_frame_ready==0)) {

      /* Ok we've got them all, package and send it out. */
      memcpy (data_message, data_mesg, sizeof (short) * 1500);

      /* Clear the active copy */
      memset (data_mesg, 0, sizeof (short) * 1500);

      /* Move over the overflow */
      memcpy (data_mesg, data_overflow, sizeof (short) * 16);

      /* Clear out the overflow */
      memset (data_overflow, 0, sizeof (short) * 16);

      /* Reset the frame ready counter */
      for (i = 0; i < N_channels; i++)
        ic->frame_ready = 0;

      /* Set the data message ready semaphore */
      data_frame_ready = 1;
    }

  }
}
