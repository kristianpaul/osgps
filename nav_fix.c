#include  <stdio.h>
#include  <stdlib.h>
#include  <math.h>
#include  <string.h>
#include  <time.h>

#include  "structs.h"
#include  "consts.h"
#include  "globals.h"
#include  "gpsfuncs.h"
#include  "file.h"
#include  "interfac.h"
/*#include  "shared.h" */
#include  "rinex.h"

#define SQ(x) ((x) * (x))

#ifdef SOFT
extern long TIC_ref;  /* for SOFT set it to extern */
#else
long TIC_ref= 571427L; /* = 571427L;   for Hardware define it here */
#endif

void nav_fix (void);
void get_velocity (void);
void gps2utc(double, int, int *, int *, int *, int *, int *, double *);

extern void rinex_head_obs(int, int, int, int, int, double);

/*****************************************************************************
FUNCTION nav_fix()
RETURNS  None.

PARAMETERS None.

PURPOSE
	This function determines the pseudorange and doppler to each
	satellite and calls pos_vel_time to determine the position and
	velocity of the receiver

WRITTEN BY
	Clifford Kelley

*****************************************************************************/

void
nav_fix (void)
{
  int n;
  double tr_time[N_channels + 1], ipart, clock_error;
  static double t_cor[N_channels + 1];
  unsigned int i;
  struct measurement *meas = measurements.measurement;
  int tr_prn[N_channels + 1];
  ecef rp_ecef;
  eceft dm_gps_sat[N_channels + 1], dp_gps_sat[N_channels + 1];
  int year, month, day, hour, minute, flag;
  double second;

  n = 1;
  i = 0;

  /* For every measurement we receive (they're null terminated) */
  while ((meas->transmit_time_offset != 0)
         && (meas->transmit_time_offset != 0)
         && i < N_channels) {

    /* Extract the PRN */
    int prn = (meas->doppler_prn & 0x1f) + 1;

    /* Make sure we have a good healthy ephemeris for this PRN */
    if ((gps_eph[prn].valid == 1) && (gps_eph[prn].health == 0)) {

	  /**
	  *  convert doppler messurement to m/s
	  *  correct for Satellite clock frequency error (af1)
	  **/
      meas_dop[n] =
        (double) (meas->doppler_prn) / 1048576.0 / TIC_dt - 1.57542e9 * gps_eph[prn].af1;
 
	  /**  (1024 phases / half chip) * (2046 half chips / ms)
	  *    * (1000 ms / sec) = 2096104000 phases / second
	  **/
      tr_time[n] = (double) measurements.bit_of_week / 50.0
        + (double) meas->transmit_time_offset / 2095104000.0;
      tr_prn[n] = prn;
      n++;

    }

    /* Next measurement */
    meas = &measurements.measurement[++i];

  }

  n_track = n - 1;
#ifdef SOFT
  TIC_dt = 0.1;
#else
  TIC_dt = measurements.i_TIC_dt * 175.0e-9;    /*each clock count is 175 ns */
#endif
  /* use basic TIC interval for ICP */
  if (out_debug)
    fprintf (debug, "n_track= %d\n", n_track);
  for (i = 1; i <= (unsigned) n_track; i++) {
    track_sat[i] = satpos_ephemeris (tr_time[i], gps_eph + tr_prn[i]);
    /* process Carrier Tracking Loop or Integrated Carrier Phase */
    if (ICP_CTL == 0) {         /* satellite velocity */
      dm_gps_sat[i] = satpos_ephemeris (tr_time[i] - TIC_dt / 2.0, 
					gps_eph + tr_prn[i]);  /* for CTL */
      dp_gps_sat[i] = satpos_ephemeris (tr_time[i] + TIC_dt / 2.0, 
					gps_eph + tr_prn[i]);
      d_sat[i].x =
        (dp_gps_sat[i].x - dm_gps_sat[i].x) / TIC_dt -
        track_sat[i].y * omegae;
      d_sat[i].y =
        (dp_gps_sat[i].y - dm_gps_sat[i].y) / TIC_dt +
        track_sat[i].x * omegae;
      d_sat[i].z = (dp_gps_sat[i].z - dm_gps_sat[i].z) / TIC_dt;
    }
    else {
      dm_gps_sat[i] = satpos_ephemeris (tr_time[i] - TIC_dt, gps_eph + tr_prn[i]);    /* for ICP */
      dp_gps_sat[i] = track_sat[i];
      d_sat[i].x =
        (dp_gps_sat[i].x - dm_gps_sat[i].x) / TIC_dt - track_sat[i].y * omegae;
      d_sat[i].y =
        (dp_gps_sat[i].y - dm_gps_sat[i].y) / TIC_dt + track_sat[i].x * omegae;
      d_sat[i].z =
        (dp_gps_sat[i].z - dm_gps_sat[i].z) / TIC_dt;
    }
    t_cor[i] = track_sat[i].tb -
      tropo_iono (tr_prn[i], track_sat[i].az, track_sat[i].el, tr_time[i]);
    dt[i] = m_time[1] - (tr_time[i] - t_cor[i]);
  }
  if (n_track >= 4) {
    rpvt = pos_vel_time (n_track);
    cbias = rpvt.dt;
    clock_error = rpvt.df;
    m_time[1] = m_time[1] - cbias;
    rp_ecef.x = rpvt.x;
    rp_ecef.y = rpvt.y;
    rp_ecef.z = rpvt.z;
    rp_llh = ecef_to_llh (rp_ecef);
    /* a position reasonableness check */
    if (rp_llh.hae > -200000.0 && rp_llh.hae < 1800000) {
          /**
	   *  Translate velocity into North, East, Up coordinates
	   **/
      get_velocity ();
      /* a velocity reasonableness check */
      if (sqrt (SQ (receiver.vel.north) + 
		SQ (receiver.vel.east) +
		SQ (receiver.vel.up)) < 514.0) {
        if (fabs (clock_error) < 500.0)
          clock_offset = clock_error;
        /* if we have a good fix we're navigating */
        status = navigating;
        if (align_t == 1) {
          long TIC_counter;
          delta_m_time = modf (m_time[1], &ipart);
          if (nav_up < 1.0) {
            delta_m_time = modf (delta_m_time / nav_up, &ipart);
            if (delta_m_time > 0.5)
              m_error = (delta_m_time - 1.0) * nav_up;
            else
              m_error = delta_m_time * nav_up;
          }
          else {
            if (delta_m_time > 0.5)
              m_error = (delta_m_time - 1.0) / nav_up;
            else
              m_error = delta_m_time / nav_up;
          }
          TIC_counter =
            (TIC_ref - m_error * TIC_ref / 10) * (1.0 -
                                                  clock_offset * 1.0e-6);
          set_TIC (TIC_counter);
        }
        rec_pos_llh.lon = rp_llh.lon;
        rec_pos_llh.lat = rp_llh.lat;
        rec_pos_llh.hae = rp_llh.hae;
        current_loc.lon = rp_llh.lon;
        current_loc.lat = rp_llh.lat;
        current_loc.hae = rp_llh.hae;
        rec_pos_xyz.x = rp_ecef.x;
        rec_pos_xyz.y = rp_ecef.y;
        rec_pos_xyz.z = rp_ecef.z;
              /**
	       *  Calculate DOPS
	       **/
        dops (n_track);
        if (out_pos == 1)
          fprintf (output,
                   "%20.10f, %f, %f, %f,",
                   m_time[1], rec_pos_llh.lat * r_to_d,
                   rec_pos_llh.lon * r_to_d, rec_pos_llh.hae);
        if (out_vel == 1)
          fprintf (output, " %f, %f, %f,",
                   receiver.vel.north, receiver.vel.east, receiver.vel.up);
        if (out_time == 1)
          fprintf (output, " %f,", clock_offset);
        if (out_pos || out_vel || out_time)
          fprintf (output, " %f, %f, %f\n", hdop, vdop, tdop);
              /**
	       * Since we have a valid position/velocity narrow the
	       * doppler search window to +-5 doppler bins
	       **/
        search_max_f = 5;
        m_time[0] = m_time[1];
      }
    }
  }
  else {
    /* less than 4 sats */
    m_time[1] = m_time[1] + nav_tic * (1.0 + clock_offset / 1.e6);
    rp_ecef.x = 0.0;
    rp_ecef.y = 0.0;
    rp_ecef.z = 0.0;
    rpvt.xv = 0.0;
    rpvt.yv = 0.0;
    rpvt.zv = 0.0;
  }
  if (out_kalman == 1) {        /* Kalman filter output */
    fprintf (kalm,
             "time %20.10f, rpx %15.10f, rpy %15.10f, rpz %15.10f, ",
             m_time[1], rp_ecef.x, rp_ecef.y, rp_ecef.z);
    fprintf (kalm, "rvx %15.10f, rvy %15.10f, rvz %15.10f, Nsats %d\n",
             rpvt.xv, rpvt.yv, rpvt.zv, n_track);
  }
  if (out_rinex == 1) {			/* RINEX OBSERVATION EPOCH/SAT */
	/* Calculate the calendar GPS time corresponding to the current GPS time 
	   adding the leapseconds as these are automatically taken out */
	gps2utc(m_time[1]+dtls, 1024+gps_eph[tr_prn[1]].week, &year, &month, &day, &hour, &minute, &second);
	
	/* Check if this is the first observation, if so, write the
	   RINEX header */
	if(write_rinex_obs_head)
	  {
	    write_rinex_obs_head = 0;
	    rinex_head_obs(year+2000, month, day, hour, minute, second);
	  }

	/* Follows the RINEX-2 Format */
	flag = 0;
    fprintf(rinex_obs," %02d %02d %02d %02d %02d%11.7f  %1d%3d", year, month, day, hour, minute, second, flag, n_track);
	
	for (i = 1; i <= (unsigned) n_track; i++)
	{
		/* Print each satellite in view */
		fprintf(rinex_obs,"G%2d", tr_prn[i]);
	}
	fprintf(rinex_obs,"\n");
  }
  for (i = 1; i <= (unsigned) n_track; i++) {
    satellite[tr_prn[i]].Pr = (m_time[1] - tr_time[i]) * c; /*(m_time[1] - (tr_time[i] - t_cor[i])) * c;*/
    satellite[tr_prn[i]].dPr = meas_dop[i] * lambda;
    if (out_kalman == 1) {      /* Kalman filter output */
      fprintf (kalm,
               "  PRN %2d, px %20.10f,  py %20.10f, pz %20.10f,  ",
               tr_prn[i], track_sat[i].x, track_sat[i].y, track_sat[i].z);
      fprintf (kalm, " vx %16.10f, vy %16.10f, vz %16.10f,  ",
               d_sat[i].x, d_sat[i].y, d_sat[i].z);
      fprintf (kalm, " Pr %20.10f,  dPr %16.10f\n",
               satellite[tr_prn[i]].Pr + t_cor[i] * c, satellite[tr_prn[i]].dPr);
    }
    if(out_rinex == 1) {	  /* RINEX data output */
      /* Currently we only are only saving the pseudorange and Doppler.  The LLI and signal 
	 strength indicators need to be implemented as well and are currently left blank */
      fprintf(rinex_obs, "%14.3f  %14.3f\n", satellite[tr_prn[i]].Pr, meas_dop[i]);
       }
  }

}

/***************************************************************************
FUNCTION velocity(void)
RETURNS  None.

PARAMETERS None.

PURPOSE  To convert velocity from ecef to local level (WGS-84) axes

WRITTEN BY
	Clifford Kelley

****************************************************************************/
void
get_velocity (void)
{
  receiver.north.x = -cos (rec_pos_llh.lon) * sin (rec_pos_llh.lat);
  receiver.north.y = -sin (rec_pos_llh.lon) * sin (rec_pos_llh.lat);
  receiver.north.z = cos (rec_pos_llh.lat);
  receiver.east.x = -sin (rec_pos_llh.lon);
  receiver.east.y = cos (rec_pos_llh.lon);
  /* receiver.east.z=0.0; */
  receiver.up.x = cos (rec_pos_llh.lon) * cos (rec_pos_llh.lat);
  receiver.up.y = sin (rec_pos_llh.lon) * cos (rec_pos_llh.lat);
  receiver.up.z = sin (rec_pos_llh.lat);

  receiver.vel.north =
    rpvt.xv * receiver.north.x + rpvt.yv * receiver.north.y +
    rpvt.zv * receiver.north.z;
  receiver.vel.east = rpvt.xv * receiver.east.x + rpvt.yv * receiver.east.y;
  receiver.vel.up = rpvt.xv * receiver.up.x + rpvt.yv * receiver.up.y +
    rpvt.zv * receiver.up.z;

  speed =
    sqrt (receiver.vel.north * receiver.vel.north +
          receiver.vel.east * receiver.vel.east);
  if (speed == 0.0)
    heading = 0.0;
  else
    heading = atan2 (receiver.vel.east, receiver.vel.north);

}

/******************************************************************************
FUNCTION gps2utc()
RETURNS year, month, day, hour, minute, second.

PARAMETERS gpstime (the gpsTime of the week)
			gpsweek (the gpsWeek number)
			leapseconds (the number of leapseconds)

PURPOSE
	This function converts from a GPS time of week to universal coordinated time
		(corrected for leap seconds).

WRITTEN BY
	Jonathan J. Makela (01-Oct-06)
******************************************************************************/

void gps2utc (double gpsTime, int gpsWeek, int *year, int *month, int *day, int *hour, int *minute, double *second)
{	
	time_t rawtime;	/* The Julian date (seconds since Jan 1, 1970) */
	struct tm *CalendarTime;		/* Holds the calendar time corresponding to the GPS time/week requested */	

	/* Create a valid time structure based on the current time */
	time(&rawtime);
	CalendarTime = gmtime(&rawtime);

	/* Modify the structure to reflect GPS time (since Jan 6, 1980) */
	CalendarTime->tm_year = 80;
	CalendarTime->tm_mon = 0;
	CalendarTime->tm_mday = 6+gpsWeek*7.0;
	CalendarTime->tm_hour = 0;
	CalendarTime->tm_min = 0;
	CalendarTime->tm_sec = gpsTime - dtls;
	mktime(CalendarTime);

	/* Pull the various values out of the tm structure */
	*year = (CalendarTime->tm_year) % 100;
	*month = CalendarTime->tm_mon + 1;
	*day = CalendarTime->tm_mday;
	*hour = CalendarTime->tm_hour;
	*minute = CalendarTime->tm_min;
	*second = CalendarTime->tm_sec + (gpsTime - floor(gpsTime));
}
