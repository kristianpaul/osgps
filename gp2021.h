
extern void reset_cntl (int);
extern void ch_accum_reset (int);
extern void ch_code_slew (int, int);
extern void ch_code (int, long);
extern void io_config (int);
extern void program_TIC (long);
extern void ch_on (int);
extern void ch_carrier (int, long);
extern void ch_epoch_load (int, unsigned int);
extern unsigned int ch_epoch_chk (int);
extern void test_control (int);
extern void system_setup (int);
extern void ch_cntl (int, int);
extern int ch_i_track (int);
extern int ch_q_track (int);
extern int ch_i_prompt (int);
extern int ch_q_prompt (int);
extern void gp2021_latch (void);
extern short int gp2021_missed (void);
extern int accum_status (void);
extern long ch_carrier_cycle (int);
extern int ch_carrier_DCO_phase (int);
extern int ch_code_phase (int);
extern unsigned int ch_epoch (int ch);
extern int ch_code_DCO_phase (int);


#ifdef UNUSED
/* These functions are defined by not currently used. */
extern void data_tst (int data);
extern void ch_code_incr_hi (int, int);
extern void ch_code_incr_lo (int, int);
extern void carr_incr_hi (int ch, int data);
extern void carr_incr_lo (char ch, int data);
extern void all_cntl (int data);
extern void multi_cntl (int data);
extern void all_code_slew (int);
extern void data_retent_w (int);
extern int data_retent_r (void);
extern void data_bus_test_w (int);
extern int data_bus_test_r (void);
extern int meas_status (void);
extern void ch_off (int);
extern void status_latch (void);
#endif
