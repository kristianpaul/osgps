#ifndef _RINEX_H
#define _RINEX_H 1
#ifndef __KERNEL__
#include <time.h>
#endif

#ifdef MAIN
int write_rinex_obs_head;
int write_rinex_nav_head;
float version = 2.1;
char system_type[1];
char program_name[20];
char agency_name[40];
char marker_name[20];
char observer_name[20];
char receiver_number[] = "XXXXX";
char receiver_type[] = "OSGPS";
char receiver_version[] = "XXX";
char antenna_number[] = "XXX";
char antenna_type[] = "XXX";
double loc_x = 151008.67;
double loc_y = -4882613.4;
double loc_z = 4087540.5;
double delx = 0;
double dely = 0;
double delz = 0;
int lamda_factor_L1 = 1;
int lamda_factor_L2 = 1;
int n_obs = 2;
char obs1[] = "C1";
char obs2[] = "D1";
char time_system[] = "GPS";
#else
extern int write_rinex_obs_head;
extern int write_rinex_nav_head;
extern float version;
extern char system_type[];
extern char program_name[];
extern char agency_name[];
extern char marker_name[];
extern char observer_name[];
extern char receiver_number[];
extern char receiver_type[];
extern char receiver_version[];
extern char antenna_number[];
extern char antenna_type[];
extern double loc_x;
extern double loc_y;
extern double loc_z;
extern double delx;
extern double dely;
extern double delz;
extern int lamda_factor_L1;
extern int lamda_factor_L2;
extern int n_obs;
extern char obs1[];
extern char obs2[];
extern char time_system[];
#endif
#endif
