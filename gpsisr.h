#ifndef _GPSISR_H
#define _GPSISR_H 1

extern long TIC_cntr;

struct accum {
  short i_prompt;
  short q_prompt;
  short i_dith;
  short q_dith;
};

struct tracking_channel
{
  long code_freq, carrier_freq, carrier_corr, code_corr;
  long avg;
  char bit;
  int bit_counter, offset;
  int codes, del_freq;
  int t_count, ms_count, ms_set, i_confirm;
  int ms_epoch, ch_time, i_count;
  int con_thresh, n_thresh;
  struct accum accum, prev_accum;
  long sum, old_theta, old_q_sum, th_rms;
  long dfreq, dfreq1, dcarr1, dcarr, cycle_sum;
  long old_carr_dco_phase;
  long q_dith_20, q_prom_20, i_dith_20, i_prom_20;
  long prompt_mag, dith_mag;
  long tr_bit_time, meas_bit_time;
  long carrier_counter;
  long phase_int, code_int, carrier_refx, code_refx;
  unsigned long fifo0, fifo1, ms_sign;
  unsigned int carr_dco_phase, carr_cycle_l, carr_cycle_h;
  unsigned int epoch, code_phase, code_dco_phase;
  time_t tow_sync_time;
};

extern struct tracking_channel chan[N_channels];

#endif /* _GPSISR_H */
