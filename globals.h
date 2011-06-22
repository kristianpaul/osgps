#ifndef _GLOBALS_H
#define _GLOBALS_H 1
#ifndef __KERNEL__
#include <time.h>
#endif

#include "structs.h"
#ifdef MAIN

int init;
struct solution_channel schan[N_channels];
struct satellite satellite[33];
int n_chan, chmax = 11, display_page = 0, alm_page;
char KFtxt[2];

int out_debug, out_pos, out_vel, out_time, out_kalman, out_data, out_rinex;

hms cur_lat, cur_long;

char Version[40];               /* NMEA */
/*
 *  These are arrays for debugging
 *  they can be written into while running and dumped to a
 *  file at the end of the run
 */
/**
long qdither[6][1500];
long qprompt[6][1500];
long idither[6][1500];
long iprompt[6][1500];
int qdither0[30000];
int qprompt0[30000];
int idither0[30000];
int iprompt0[30000];
**/

long store_code, store_carrier;
/* definitions with default values which can be overridden in file
   rcvr_par.dat */
int cold_prn = 1;
long time_on = 0;
float nav_up = 1.0;
double speed, heading;
int key;
int n_track;
unsigned int interr_int = 512;
float clock_offset = 0.0;
ecef rec_pos_ecef;
double m_time[3], delta_m_time, m_error, TIC_dt;

int last_hi_carr[N_channels], last_hi_code[N_channels];

char last_address;

int ms_count;

unsigned int Com0Baud;          /* NMEA */
unsigned int Com1Baud;
unsigned int GPGGA;
unsigned int GPGSV;
unsigned int GPGSA;
unsigned int GPVTG;
unsigned int GPRMC;
unsigned int GPZDA;

char tzstr[40];                 /* = "TZ=PST8PDT"; */

almanac gps_alm[33];

int SVh[33], ASV[33];
/* broadcast ionospheric delay model */
float b0, b1, b2, b3, al0, al1, al2, al3;       
float a0, a1, tot, WNt, dtls, WNlsf, DN, dtlsf; /* broadcast UTC data */

ephemeris gps_eph[33];

pvt rpvt;

state receiver;

float gdop, pdop, hdop, vdop, tdop, alm_toa;
llh rec_pos_llh;
llh current_loc, rp_llh;
eceft track_sat[N_channels + 1];
ecef rec_pos_xyz;
int alm_gps_week, gps_week, almanac_valid, handle;
unsigned long sf[6][11];
int p_error[6], status;

enum
{ cold_start, warm_start, hot_start, tracking, navigating };
/*        0          1          2        3          4     */

/* single bit set numbers for testing bit positions */
unsigned long test_l[33] = { 0x00000000L,       
  0x00000001L, 0x00000002L, 0x00000004L, 0x00000008L,
  0x00000010L, 0x00000020L, 0x00000040L, 0x00000080L,
  0x00000100L, 0x00000200L, 0x00000400L, 0x00000800L,
  0x00001000L, 0x00002000L, 0x00004000L, 0x00008000L,
  0x00010000L, 0x00020000L, 0x00040000L, 0x00080000L,
  0x00100000L, 0x00200000L, 0x00400000L, 0x00800000L,
  0x01000000L, 0x02000000L, 0x04000000L, 0x08000000L,
  0x10000000L, 0x20000000L, 0x40000000L, 0x80000000L
};

float mask_angle;
char header[45], trailer;
double meas_dop[N_channels + 1];

ecef d_sat[N_channels + 1];

double dt[N_channels + 1], cbias;
double Xv[9], Pv[9][9];

int m_tropo, m_iono, align_t;   /* flags for using tropo and iono models */
unsigned int i4page, i5page;

satvis xyz[33];

#else

extern int init;
extern struct solution_channel schan[N_channels];
extern struct satellite satellite[33];
extern int n_chan, chmax, display_page, alm_page;
extern char KFtxt[2];

extern int out_debug, out_pos, out_vel, out_time, out_kalman, out_data, out_rinex;

extern hms cur_lat, cur_long;

extern char Version[40];        /* NMEA */
/**
 *  These are arrays for debugging
 *  they can be written into while running and dumped to a
 *  file at the end of the run
 **/

/**
//extern long qdither[6][1500];
//extern long qprompt[6][1500];
//extern long idither[6][1500];
//extern long iprompt[6][1500];
//extern int qdither0[30000];
//extern int qprompt0[30000];
//extern int idither0[30000];
//extern int iprompt0[30000];
**/

extern long store_code, store_carrier;
/* definitions with default values which can be overridden in file
   rcvr_par.dat */
extern int cold_prn;
extern float nav_up;
extern double speed, heading;
extern int key;
extern int n_track;
extern unsigned int interr_int;
extern float clock_offset;
extern ecef rec_pos_ecef;
extern double m_time[3], delta_m_time, m_error, TIC_dt;

extern int last_hi_carr[N_channels], last_hi_code[N_channels];

extern char last_address;

extern int ms_count;

extern unsigned int Com0Baud;   /* NMEA */
extern unsigned int Com1Baud;
extern unsigned int GPGGA;
extern unsigned int GPGSV;
extern unsigned int GPGSA;
extern unsigned int GPVTG;
extern unsigned int GPRMC;
extern unsigned int GPZDA;

extern char tzstr[40];          /* = "TZ=PST8PDT"; */

extern almanac gps_alm[33];

extern int SVh[33], ASV[33];
/* broadcast ionospheric delay model */
extern float b0, b1, b2, b3, al0, al1, al2, al3;     
extern float a0, a1, tot, WNt, dtls, WNlsf, DN, dtlsf; /* broadcast UTC data */

extern ephemeris gps_eph[33];

extern pvt rpvt;

extern state receiver;

extern float gdop, pdop, hdop, vdop, tdop, alm_toa;
extern llh rec_pos_llh;
extern llh current_loc, rp_llh;
extern eceft track_sat[N_channels + 1];
extern ecef rec_pos_xyz;
extern int alm_gps_week, gps_week, almanac_valid, handle;
extern unsigned long sf[6][11];
extern int p_error[6], status;

enum
{ cold_start, warm_start, hot_start, tracking, navigating };
/*        0          1          2        3          4    */

extern unsigned long test_l[33];

extern float mask_angle;
extern char header[45], trailer;
extern double meas_dop[N_channels + 1];

extern ecef d_sat[N_channels + 1];

extern double dt[N_channels + 1], cbias;
extern double Xv[9], Pv[9][9];

/* flags for using tropo and iono models */
extern int m_tropo, m_iono, align_t;
extern unsigned int i4page, i5page;

extern satvis xyz[33];

#endif

#endif /* _GLOBALS_H */
