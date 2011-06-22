/***********************************************************************
  GPS RECEIVER (GPSRCVR) Ver. 2.00
  See comments in GPSRCVR.CPP
  12 Channel All-in-View GPS Receiver Program based on Mitel GP2021
  chipset
  Clifford Kelley <cwkelley@earthlink.net>
  This program is licensed under GNU GENERAL PUBLIC LICENSE
  This LICENSE file must be included with the GPSRCVR code.
**********************************************************************/

#include  <time.h>
#include  <stdio.h>
#include  <stdlib.h>
#include  <math.h>
#include  <string.h>

#include  "consts.h"
#include  "structs.h"
#include  "globals.h"
#include  "gpsfuncs.h"
#include  "file.h"
#include  "interfac.h"
#include  "rinex.h"

#define bit_test_l(data, bit_n) (data & (0x1L << (bit_n-1)))
#define SQ(x) ((x)*(x))
#define CUBE(x) ((x) * (x) * (x))

extern void rinex_data_nav(ephemeris *, int);
extern void rinex_head_nav(int, int, int, int, int, double);
extern void gps2utc(double, int, int *, int *, int *, int *, int *, double *);
extern time_t utctime;

/**
 * Local (private) functions
 **/

static ecef satpos_almanac (float, int);
static void parity_check (void);
static int exor (char, long);
static void read_almanac (void);

/**
 * binary constants for nav message decoding
 **/

double const c_2p12 = 4096;
double const c_2p4 = 16;
double const c_2m5 = 0.03125;
double const c_2m11 = 4.8828125e-4;
double const c_2m19 = 1.9073486328125e-6;
double const c_2m20 = 9.5367431640625e-7;
double const c_2m21 = 4.76837158203125e-7;
double const c_2m23 = 1.19209289550781e-7;
double const c_2m24 = 5.96046447753906e-8;
double const c_2m27 = 7.45058059692383e-9;
double const c_2m29 = 1.86264514923096e-9;
double const c_2m30 = 9.31322574615479e-10;
double const c_2m31 = 4.65661287307739e-10;
double const c_2m33 = 1.16415321826935E-10;
double const c_2m38 = 3.63797880709171e-12;
double const c_2m43 = 1.13686837721616e-13;
double const c_2m50 = 8.881784197e-16;
double const c_2m55 = 2.77555756156289e-17;

double const a = 6378137.0, b = 6356752.314; /* WGS-84 ellipsoid parameters */

/****************************************************************************
FUNCTION satfind()
RETURNS  None.

PARAMETERS None.

PURPOSE

	THIS FUNCTION DETERMINES THE SATELLITES TO SEARCH FOR
	WHEN ALMANAC DATA IS AVAILABLE

WRITTEN BY
	Clifford Kelley

*****************************************************************************/

satvis
satfind (int i)
{
  float tdot, az;
  float satang, alm_time, almanac_date;
  double range1, range2, xls, yls, zls, xaz, yaz;
  long jd_yr;
  ecef gpspos1, gpspos2, north, east, up;
  satvis result;
  int jd_m;
  struct tm *gmt;
  double time_s;

/*
      INITIALIZE ALL THE CONSTANTS
*/
/*      gotoxy(1,24); */
/*      printf("->satfind"); */
  memset (&result, 0, sizeof(satvis));
  putenv (tzstr);
  tzset ();
  gmt = gmtime (&utctime);
/* set up the correct time */
  if (gmt->tm_mon <= 1)
    {
      jd_yr = 365.25 * (gmt->tm_year - 1. + 1900.);
      jd_m = 30.6001 * (gmt->tm_mon + 14.);
    }
  else
    {
      jd_yr = 365.25 * (gmt->tm_year + 1900.);
      jd_m = 30.6001 * (gmt->tm_mon + 2.);
    }
  time_s =
    gmt->tm_min / 1440. + gmt->tm_sec / 86400. + 1720981.5 +
    gmt->tm_hour / 24. + jd_yr + jd_m + gmt->tm_mday;
  gps_week = (int) ((time_s - 2444244.5) / 7.);
  almanac_date = gps_alm[i].week * 7.0 + 2444244.5;
  if (gps_week - gps_alm[i].week > 512)
    almanac_date += 1024 * 7.0;
  alm_time = (time_s - almanac_date) * 86400.;
  clock_tow = (time_s - gps_week * 7. - 2444244.5) * 86400.;
/*
      CALCULATE THE POSITION OF THE SATELLITES
*/
  if (gps_alm[i].inc > 0.0 && i > 0)
    {
      gpspos1 = satpos_almanac (alm_time, i);
      gpspos2 = satpos_almanac (alm_time + 1.0, i);
/*
      CALCULATE THE POSITION OF THE RECEIVER
*/
      rec_pos_xyz = llh_to_ecef (current_loc);
      north.x = -cos (current_loc.lon) * sin (current_loc.lat);
      north.y = -sin (current_loc.lon) * sin (current_loc.lat);
      north.z = cos (current_loc.lat);
      east.x = -sin (current_loc.lon);
      east.y = cos (current_loc.lon);
/*   east.z=0.0; */
      up.x = cos (current_loc.lon) * cos (current_loc.lat);
      up.y = sin (current_loc.lon) * cos (current_loc.lat);
      up.z = sin (current_loc.lat);
/*
     DETERMINE IF A CLEAR LINE OF SIGHT EXISTS
*/
      xls = gpspos1.x - rec_pos_xyz.x;
      yls = gpspos1.y - rec_pos_xyz.y;
      zls = gpspos1.z - rec_pos_xyz.z;
      range1 = sqrt (xls * xls + yls * yls + zls * zls);
      tdot = (up.x * xls + up.y * yls + up.z * zls) / range1;
      xls = xls / range1;
      yls = yls / range1;
      zls = zls / range1;
      range2 = sqrt (SQ (gpspos2.x - rec_pos_xyz.x - rpvt.xv) +
                     SQ (gpspos2.y - rec_pos_xyz.y - rpvt.yv) +
                     SQ (gpspos2.z - rec_pos_xyz.z - rpvt.zv));

      if (tdot >= 1.00)
        satang = M_PI / 2.0;
      else if (tdot <= -1.00)
        satang = -M_PI / 2.0;
      else
        satang = asin (tdot);

      xaz = east.x * xls + east.y * yls;
      yaz = north.x * xls + north.y * yls + north.z * zls;
      if (xaz != 0.0 || yaz != 0.0)
        az = atan2 (xaz, yaz);
      else
        az = 0.0;
      result.x = gpspos1.x;
      result.y = gpspos1.y;
      result.z = gpspos1.z;
      result.elevation = satang;
      result.azimuth = az;
      result.doppler = (range1 - range2) / lambda;   /* changed to lambda */
    }
/*      gotoxy(1,24); */
/*      printf("satfind->"); */
  return (result);
}


/****************************************************************************
FUNCTION satpos_almanac(float time, char n)
RETURNS  None.

PARAMETERS
			time   float  time of week
			n      char   satellite prn

PURPOSE

	  THIS SUBROUTINE CALCULATES THE SATELLITE POSITION
	  BASED ON ALMANAC DATA
	  
     R    - RADIUS OF SATELLITE AT TIME T
     SLAT - SATELLITE LATITUDE
     SLONG- SATELLITE LONGITUDE
     T    - TIME FROM START OF WEEKLY EPOCH
     ETY  - ORBITAL ECCENTRICITY
     TOA  - TIME OF APPLICABILITY FROM START OF WEEKLY EPOCH
     INC  - ORBITAL INCLINATION
     RRA  - RATE OF RIGHT ASCENSION
	  SQA  - SQUARE ROOT OF SEMIMAJOR AXIS
     LAN  - LONGITUDE OF NODE AT WEEKLY EPOCH
     AOP  - ARGUMENT OF PERIGEE
	  MA   - MEAN ANOMALY AT TOA

WRITTEN BY
	Clifford Kelley

****************************************************************************/

static ecef
satpos_almanac (float time, int n)
{
  double ei, ea, diff, r, ta, la, aol, xp, yp, d_toa;
  ecef result;
/*
      MA IS THE ANGLE FROM PERIGEE AT TOA
*/
/*      gotoxy(1,24); */
/*      printf("->satpos_almanac");  */

  d_toa = time - gps_alm[n].toa;
  if (d_toa > 302400.0)
    d_toa = d_toa - 604800.0;
  else if (d_toa < -302400.0)
    d_toa = d_toa + 604800.0;
  ei = gps_alm[n].ma + d_toa * gps_alm[n].w;
  ea = ei;
  do
    {
      diff =
        (ei - (ea - gps_alm[n].ety * sin (ea))) / (1. -
                                                   gps_alm[n].ety * cos (ea));
      ea = ea + diff;
    }
  while (fabs (diff) > 1.0e-6);
/*
      EA IS THE ECCENTRIC ANOMALY
*/
  if (gps_alm[n].ety != 0.0)
    ta =
      atan2 (sqrt (1. - SQ (gps_alm[n].ety)) * sin (ea),
             cos (ea) - gps_alm[n].ety);
  else
    ta = ea;
/*
      TA IS THE TRUE ANOMALY (ANGLE FROM PERIGEE)
*/
  r = SQ (gps_alm[n].sqa) * (1. - gps_alm[n].ety * cos (ea));
/*
      R IS THE RADIUS OF SATELLITE ORBIT AT TIME T
*/
  aol = ta + gps_alm[n].aop;
/*
      AOL IS THE ARGUMENT OF LATITUDE

		LA IS THE LONGITUDE OF THE ASCENDING NODE
*/
  la =
    gps_alm[n].lan + (gps_alm[n].rra - omegae) * d_toa -
    gps_alm[n].toa * omegae;
  xp = r * cos (aol);
  yp = r * sin (aol);
  result.x = xp * cos (la) - yp * cos (gps_alm[n].inc) * sin (la);
  result.y = xp * sin (la) + yp * cos (gps_alm[n].inc) * cos (la);
  result.z = yp * sin (gps_alm[n].inc);
/*      gotoxy(1,24); */
/*      printf("satpos_almanac->"); */
  return (result);
}

/*****************************************************************************
FUNCTION satpos_ephemeris(double t, struct ephemeris *eph)
RETURNS  None.

PARAMETERS
			t    double              time of week
			eph  struct ephemeris *  satellite ephemeris

PURPOSE

     THIS SUBROUTINE CALCULATES THE SATELLITE POSITION
     BASED ON BROADCAST EPHEMERIS DATA

     R    - RADIUS OF SATELLITE AT TIME T
     Crc  - RADIUS COSINE CORRECTION TERM
     Crs  - RADIUS SINE   CORRECTION TERM
     SLAT - SATELLITE LATITUDE
     SLONG- SATELLITE LONGITUDE
     TOE  - TIME OF EPHEMERIS FROM START OF WEEKLY EPOCH
	  ETY  - ORBITAL INITIAL ECCENTRICITY
	  TOA  - TIME OF APPLICABILITY FROM START OF WEEKLY EPOCH
     INC  - ORBITAL INCLINATION
     IDOT - RATE OF INCLINATION ANGLE
     CUC  - ARGUMENT OF LATITUDE COSINE CORRECTION TERM
     CUS  - ARGUMENT OF LATITUDE SINE   CORRECTION TERM
     CIC  - INCLINATION COSINE CORRECTION TERM
     CIS  - INCLINATION SINE   CORRECTION TERM
     RRA  - RATE OF RIGHT ASCENSION
     SQA  - SQUARE ROOT OF SEMIMAJOR AXIS
     LAN  - LONGITUDE OF NODE AT WEEKLY EPOCH
     AOP  - ARGUMENT OF PERIGEE
     MA   - MEAN ANOMALY AT TOA
     DN   - MEAN MOTION DIFFERENCE

WRITTEN BY
	Clifford Kelley

*****************************************************************************/

eceft
satpos_ephemeris (double t, struct ephemeris *eph)
{
  double ei, ea, diff, ta, aol, delr, delal, delinc, r, inc;
  double la, xp, yp, bclk, tc, d_toc, d_toe;
  double xls, yls, zls, range1, tdot, satang, xaz, yaz;
  double az;
  ecef north, east, up;
  eceft result;
  /**
   *  MA IS THE ANGLE FROM PERIGEE AT TOA
   **/
/*      gotoxy(1,24); */
/*      printf("->satpos_ephemeris"); */
  d_toc = t - eph->toc;
  if (d_toc > 302400.0)
    d_toc = d_toc - 604800.0;
  else if (d_toc < -302400.0)
    d_toc = d_toc + 604800.0;
  bclk =
    eph->af0 + eph->af1 * d_toc + eph->af2 * d_toc * d_toc - eph->tgd;
  tc = t - bclk;
  d_toe = tc - eph->toe;
  if (d_toe > 302400.0)
    d_toe = d_toe - 604800.0;
  else if (d_toe < -302400.0)
    d_toe = d_toe + 604800.0;
  ei = eph->ma + d_toe * (eph->wm + eph->dn);
  ea = ei;
  do
    {
      diff =
        (ei - (ea - eph->ety * sin (ea))) / (1.0E0 -
                                                   eph->ety * cos (ea));
      ea = ea + diff;
    }
  while (fabs (diff) > 1.0e-12);
  bclk = bclk - 4.442807633E-10 * eph->ety * eph->sqra * sin (ea);
  result.tb = bclk;
  /**
   * ea is the eccentric anomaly
   **/
  ta =
    atan2 (sqrt (1.00 - eph->ety * eph->ety) * sin (ea),
           cos (ea) - eph->ety);
  /**
   *  TA IS THE TRUE ANOMALY (ANGLE FROM PERIGEE)
   **/
  aol = ta + eph->w;
  /**
   *  AOL IS THE ARGUMENT OF LATITUDE OF THE SATELLITE
   *
   *  calculate the second harmonic perturbations of the orbit
   **/
  delr = eph->crc * cos (2.0 * aol) + eph->crs * sin (2.0 * aol);
  delal = eph->cuc * cos (2.0 * aol) + eph->cus * sin (2.0 * aol);
  delinc =
    eph->cic * cos (2.0 * aol) + eph->cis * sin (2.0 * aol);
  /**
   *  R IS THE RADIUS OF SATELLITE ORBIT AT TIME T
   **/
  r = SQ (eph->sqra) * (1.00 - eph->ety * cos (ea)) + delr;
  aol = aol + delal;
  inc = eph->inc0 + delinc + eph->idot * d_toe;
  /* WRITE(6,*)T-TOE(N) */
  /**
   * LA IS THE CORRECTED LONGITUDE OF THE ASCENDING NODE
   **/
  la = eph->w0 + (eph->omegadot - omegae) * d_toe -
    omegae * eph->toe;
  xp = r * cos (aol);
  yp = r * sin (aol);
  result.x = xp * cos (la) - yp * cos (inc) * sin (la);
  result.y = xp * sin (la) + yp * cos (inc) * cos (la);
  result.z = yp * sin (inc);
  result.az = 0.0;
  result.el = 0.0;
  if (rec_pos_xyz.x != 0.0 || rec_pos_xyz.y != 0.0 || rec_pos_xyz.z != 0.0)
    {
      /**
       * CALCULATE THE POSITION OF THE RECEIVER
       **/
      north.x = -cos (rec_pos_llh.lon) * sin (rec_pos_llh.lat);
      north.y = -sin (rec_pos_llh.lon) * sin (rec_pos_llh.lat);
      north.z = cos (rec_pos_llh.lat);
      east.x = -sin (rec_pos_llh.lon);
      east.y = cos (rec_pos_llh.lon);
      east.z = 0.0;
      up.x = cos (rec_pos_llh.lon) * cos (rec_pos_llh.lat);
      up.y = sin (rec_pos_llh.lon) * cos (rec_pos_llh.lat);
      up.z = sin (rec_pos_llh.lat);
      /**
       * DETERMINE IF A CLEAR LINE OF SIGHT EXISTS
       **/
      xls = result.x - rec_pos_xyz.x;
      yls = result.y - rec_pos_xyz.y;
      zls = result.z - rec_pos_xyz.z;
      range1 = sqrt (xls * xls + yls * yls + zls * zls);
      tdot = (up.x * xls + up.y * yls + up.z * zls) / range1;

      if (tdot >= 1.00)
        satang = M_PI / 2.0;
      else if (tdot <= -1.00)
        satang = -M_PI / 2.0;
      else
        satang = asin (tdot);

      xaz = east.x * xls + east.y * yls;
      yaz = north.x * xls + north.y * yls + north.z * zls;
      if (xaz != 0.0 || yaz != 0.0)
        az = atan2 (xaz, yaz);
      else
        az = 0.0;
      result.el = satang;
      result.az = az;
      /**
       *   if (satang<(mask_angle-.035))
       *   {
       *       fprintf(kalm,"prn %d LOS problem time=%lf\n",n,t);
       *       fprintf(kalm,"lat=%lf  long=%lf hae=%lf\n",
       *       rec_pos_llh.lat,rec_pos_llh.lon,rec_pos_llh.hae);
       *
       *       fprintf(kalm,"rcvr x=%lf y=%lf
       *       z=%lf\n",rec_pos_xyz.x,rec_pos_xyz.y,rec_pos_xyz.z);
       *
       *       fprintf(kalm,"sat x=%lf y=%lf
       *       z=%lf\n",result.x,result.y,result.z);
       *
       *   	 write_Kalm_ephemeris(n);
       *   }
       **/
    }
  /* gotoxy(1,24); */
  /* printf("satpos_ephemeris->"); */
  return (result);
}

/*****************************************************************************
FUNCTION read_initial_data(void)
RETURNS  None.

PARAMETERS None.

PURPOSE
		  To read in all of the receiver initialization files

WRITTEN BY
	Clifford Kelley

*****************************************************************************/

void
read_initial_data (void)
{
  int id;
  for (id = 1; id <= 32; id++)
    gps_alm[id].inc = 0.0;
  /**
   *  READ THE INPUT DATA FILE(s)
   **/
  /* for initialization, if we have enough data we will switch to warm
     or hot start */
  status = cold_start;    
  read_ion_utc ();              
#ifndef SOFT
  utctime = time (NULL) + dtls;
#endif
  read_almanac ();
  satfind (0);
}

/*****************************************************************************
FUNCTION receiver_loc(void)
RETURNS  None.

PARAMETERS None.

PURPOSE       To read in the last location of the receiver from file
		"curloc.dat" to help in warm and hot starts

WRITTEN BY
	Clifford Kelley

*****************************************************************************/

llh
receiver_loc (void)
{
  float latitude, longitude, height;
  char text[10];
  llh result;
  result.lat = 0.0;
  result.lon = 0.0;
  result.hae = 0.0;
/*
     READ THE CURRENT LOCATION DATA FILE
*/
  if ((in = fopen (location_file, "r")) == NULL)
    {
      printf ("Cannot open %s file.\n",location_file);
      status = cold_start;
    }
  else
    {
      fscanf (in, "%10s", text);
      fscanf (in, "%f", &latitude);
      fscanf (in, "%10s", text);
      fscanf (in, "%f", &longitude);
      fscanf (in, "%10s", text);
      fscanf (in, "%f", &height);
      result.lat = latitude / 57.296;
      result.lon = longitude / 57.296;
      result.hae = height;
      fclose (in);
    }
  return (result);
}

/*****************************************************************************
FUNCTION navmess()
RETURNS  None.

PARAMETERS None.

PURPOSE
	This function assembles and decodes the 1500 bit nav message
	into almanac and ephemeris messages

WRITTEN BY
	Clifford Kelley
3-2-2002  Made corrections suggested by Georg Beyerle GFZ
*****************************************************************************/

void
navmess (int prn, int ch)
{
  int i, k;
  unsigned long isqra, ie;
  long iaf0, iomegadot, iomega0;
  char itgd, iaf2;
  /* int icapl2; */
  short int iweek, iura, ihealth, iodc, iaf1;
  unsigned int itoe, itoc;
  /* int fif; */
  short int iode, icrs, idn, icuc, icus, icic;
  short int iomegad;
  short int icis, icrc, idoe, idot;
  unsigned int iae, iatoa;
  /* static int i4page,i5page; */
  short int i4data, i5data, isv, iaomegad;
  long iaaf0, iaaf1, iadeli, iaomega0, im0, inc0, iw;
  unsigned long iasqr;
  long iaw, iam0, scale, ia0, ia1;
  char ial0, ial1, ial2, ial3, ibt0, ibt1, ibt2, ibt3;
  short int itot, iWNt, idtls, iWNlsf, iDN, idtlsf;   /* WNa */
  short int sfr, word, i4p, i5p;
  double r_sqra, r_inc0, r_ety;
  float d_toe;
  unsigned short *data_msg_ptr, chmask = (1<<ch);
  unsigned long *sfptr;
  int year, month, day, hour, minute;
  double second;
  /**
   *    assemble the 1500 data bits into subframes and words
   **/
  /* gotoxy(1,24); */
  /* printf("->navmess prn %d  ch %d",prn,ch); */
  d_toe = clock_tow - gps_eph[prn].toe;
  if (d_toe > 302400.0)
    d_toe = d_toe - 604800.0;
  else if (d_toe < -302400.0)
    d_toe = d_toe + 604800.0;
  data_msg_ptr = data_message;
  for (sfr = 1; sfr <= 5; sfr++)
    {
      sfptr = sf[sfr] + 1;
      for (word = 1; word <= 10; word++)
        {
          scale = 0x20000000L;
	  *sfptr = 0;
          for (i = 0; i < 30; i++)
            {
	      if (*data_msg_ptr & chmask)
                *sfptr |= scale;
              scale = scale >> 1;
              data_msg_ptr++;
            }
	  sfptr++;
        }
    }
  parity_check ();             /* check the parity of the 1500 bit message */
  schan[ch].word_error[0] = p_error[1];
  schan[ch].word_error[1] = p_error[2];
  schan[ch].word_error[2] = p_error[3];
  schan[ch].word_error[3] = p_error[4];
  schan[ch].word_error[4] = p_error[5];
  /**
   *   EPHEMERIS DECODE subframes 1, 2, 3
   *
   *  subframe 1
   *
   * check parity of first 3 subframes, since it is a circular
   * register we may have over written the first few bits so allow for
   * word 1 of subframe 1 to have a problem
   **/

  if ((p_error[1] == 0 || p_error[1] == 0x200) && p_error[2] == 0
      && p_error[3] == 0)
    {
      iodc = (int) (((sf[1][3] & 0x3) << 8) | ((sf[1][8] & 0xFF0000L) >> 16));
      iode = (int) (sf[2][3] >> 16);
      idoe = (int) (sf[3][10] >> 16);
      /* fprintf(kalm,"prn=%d iodc=%d, iode=%d,idoe=%d\n",prn,iodc,iode,idoe); */
      /* fprintf(kalm," eph.iode=%d, eph.iodc=%d\n",gps_eph[prn].iode,gps_eph[prn].iodc); */
      /* if both copies of iode agree and we have a new iode or new
	 iodc then process the ephemeris */
      if (iode == idoe
          && ((iode != gps_eph[prn].iode) || (iodc != gps_eph[prn].iodc)))
        {

          iweek = (int) (sf[1][3] >> 14);
	  /*                      icapl2=( sf[1][3] & 0x3000 ) >> 12; */
          iura = (int) ((sf[1][3] & 0xF00) >> 8);
          ihealth = (int) ((sf[1][3] & 0xFC) >> 2);
          itgd = (int) (sf[1][7] & 0xFF);
          itoc = (int) (sf[1][8] & 0xFFFF);
          iaf2 = (int) (sf[1][9] >> 16);
          iaf1 = (int) (sf[1][9] & 0xFFFF);
          iaf0 = sf[1][10] >> 2;
          if (bit_test_l (iaf0, 22))
	    iaf0 = ~( (~iaf0) & 0x3fffff);
	  /**
	   * subframe 2 
	   **/
          icrs = (short int) (sf[2][3] & 0xFFFF);
          idn = (int) (sf[2][4] >> 8);
          im0 = ((sf[2][4] & 0xFF) << 24) | sf[2][5];
	  if (bit_test_l (im0, 32))
	    im0 = ~( (~im0) & 0xffffffff);
          icuc = (int) (sf[2][6] >> 8);
          ie = ((sf[2][6] & 0xFF) << 24) | sf[2][7];
	  if (bit_test_l (ie, 32))
	    ie = ~( (~ie) & 0xffffffff);
          icus = (int) (sf[2][8] >> 8);
          isqra = (((sf[2][8] & 0xFF) << 24) | sf[2][9]);
#if 0
	  if (bit_test_l (isqra, 32))
	    isqra = ~( (~isqra) & 0xffffffff);
#endif
          itoe = (int) (sf[2][10] >> 8);
	  /* fif=int((sf[2][10] & 0x80) >> 7); */
	  /**
	   * subframe 3
	   **/
          icic = (short int) (sf[3][3] >> 8);
          icis = (short int) (sf[3][5] >> 8);
          inc0 = ((sf[3][5] & 0xFF) << 24) | sf[3][6];
	  if (bit_test_l (inc0, 32))
	    inc0 = ~( (~inc0) & 0xffffffff);
          iomega0 = ((sf[3][3] & 0xFF) << 24) | sf[3][4];
	  if (bit_test_l (iomega0, 32))
	    iomega0 = ~( (~iomega0) & 0xffffffff);
          icrc = (short int) (sf[3][7] >> 8);
          iw = ((sf[3][7] & 0xFF) << 24) | sf[3][8];
	  if (bit_test_l (iw, 32))
	    iw = ~( (~iw) & 0xffffffff);
          iomegadot = sf[3][9];
          if (bit_test_l (iomegadot, 24))
	    iomegadot = ~( (~iomegadot) & 0xffffff);
          idot = (int) ((sf[3][10] & 0xFFFC) >> 2);
          if (bit_test_l (idot, 14))
	    idot = ~( (~idot) & 0x3fff);

          r_sqra = isqra * c_2m19;
          r_inc0 = inc0 * c_2m31 * M_PI;
          r_ety = ie * c_2m33;
	  /*fprintf(kalm,"sqra=%lf, inc=%lf,ety=%lf\n",r_sqra,r_inc0,r_ety); */
	  /**
	   * Does this ephemeris make sense?
	   **/
          if ((r_inc0 < 1.25 && r_inc0 > 0.873) && (iaf2 ==0)
              && (r_sqra > 5100.0 && r_sqra < 5200.0) && (r_ety < .05
                                                          && r_ety > 0.0))
            {
              gps_eph[prn].valid = 1;

              gps_eph[prn].iode = iode;
              gps_eph[prn].iodc = iodc;
              gps_eph[prn].week = iweek;
              gps_eph[prn].ura = iura;
              gps_eph[prn].health = ihealth;
              gps_eph[prn].tgd = itgd * c_2m31;
              gps_eph[prn].toc = itoc * 16.0;
              gps_eph[prn].af2 = iaf2 * c_2m55;
              gps_eph[prn].af1 = iaf1 * c_2m43;
              gps_eph[prn].af0 = iaf0 * c_2m31;
              gps_eph[prn].crs = icrs * c_2m5;
              gps_eph[prn].dn = idn * c_2m43 * M_PI;
              gps_eph[prn].ma = im0 * c_2m31 * M_PI;
              gps_eph[prn].cuc = icuc * c_2m29;
              gps_eph[prn].ety = r_ety;
              gps_eph[prn].cus = icus * c_2m29;
              gps_eph[prn].sqra = r_sqra;
              gps_eph[prn].wm = 19964981.84 / CUBE (r_sqra);
              gps_eph[prn].toe = itoe * c_2p4;
              gps_eph[prn].cic = icic * c_2m29;
              gps_eph[prn].cis = icis * c_2m29;
              gps_eph[prn].inc0 = r_inc0;
              gps_eph[prn].w0 = iomega0 * c_2m31 * M_PI;
              gps_eph[prn].crc = icrc * c_2m5;
              gps_eph[prn].w = iw * c_2m31 * M_PI;
              gps_eph[prn].omegadot = iomegadot * c_2m43 * M_PI;
              gps_eph[prn].idot = idot * c_2m43 * M_PI;

	      /* Write this ephemeris data to the RINEX nav file */
	      if(out_rinex == 1)
		{
		  if(write_rinex_nav_head)
		    {
		      write_rinex_nav_head = 0;

		      /* Calculate the calendar date */
		      gps2utc(clock_tow, 1024+gps_eph[prn].week, &year, &month, &day, &hour, &minute, &second);
		      /* Write the RINEX header */
		      rinex_head_nav(year, month, day, hour, minute, second);
		    }
		  /* Write the RINEX data */
		  rinex_data_nav(&gps_eph[prn], prn);
		}

	      if (out_debug)
                write_Debug_ephemeris (prn);
            }
          else if (gps_eph[prn].valid == 1 && d_toe > 7500.0)
            gps_eph[prn].valid = 0;

        }
    }
  /**
   *    ALMANAC DECODE  subframes 4 and 5
   *
   *    SUBFRAME 4
   *
   *   check parity of subframes 4 and 5 and don't bother if we already
   *   have the almanac
   **/
/*  fprintf(kalm,"p_error[4]=%d p_error[5]=%d almanac_valid=%d\n", */
/* 	 p_error[4],p_error[5],almanac_valid); */

  /* if no parity errors on subframe 4 & 5 */
  if (p_error[4] == 0 && p_error[5] == 0)  
    {
      i5data = (int) (sf[5][3] >> 22);
      i5p = (int) ((sf[5][3] & 0x3F0000L) >> 16);
      alm_page = i5p;
      if (i5page != (unsigned) i5p && i5data == 1)
        {
          if (i5p != 51 && i5p != 0)
            {
              isv = i5p;
              iatoa = (int) (sf[5][4] >> 16);
              if (gps_alm[isv].week == gps_week % 1024
                  && gps_alm[isv].toa == (float) (iatoa * c_2p12))
                {
                  almanac_valid = 1;    /* if they are the same as what we already have */
                  i5page = i5p; /* then almanac is up to date for that satellite */
                }
              else
                almanac_valid = 0;
	      /*           fprintf(kalm,"isv=%d gps_week=%d gps_alm[isv].week=%d gps_alm[isv].toa=%f iatoa*c_2p12=%f\n valid=%d i5page=%d\n", */
	      /*           isv,gps_week%1024,gps_alm[isv].week,gps_alm[isv].toa,(float)(iatoa*c_2p12),almanac_valid,i5page); */
            }
        }
    }
  if (p_error[4] == 0 && p_error[5] == 0 /* && almanac_valid == 0 */) /* if no parity errors on subframe 4 & 5 */
    {                           /* and data not valid for this page */
      i4data = (int) (sf[4][3] >> 22);
      i4p = (int) ((sf[4][3] & 0x3F0000L) >> 16);
      if ((unsigned) i4p != i4page && i4data == 1)      /*! i4p all we need is a page to be */
        {                       /*! read from 1 satellite */
          i4page = i4p;
          if (i4page > 24 && i4page < 33)       /*  almanacs for PRN 25 -> 32 */
            {
              isv = i4page;
              gps_alm[isv].week = gps_week % 1024;
              iae = (int) (sf[4][3] & 0x00FFFFL);
              gps_alm[isv].ety = iae * c_2m21;
              iatoa = (int) (sf[4][4] >> 16);
              gps_alm[isv].toa = iatoa * c_2p12;
              iadeli = sf[4][4] & 0x00FFFFL;
              if (bit_test_l (iadeli, 16))
		iadeli = ~( (~iadeli) & 0xffff);
              gps_alm[isv].inc = (iadeli * c_2m19 + 0.3) * M_PI;
              iomegad = (short int) (sf[4][5] >> 8);
              gps_alm[isv].rra = iomegad * c_2m38 * M_PI;
              gps_alm[isv].health = (int) (sf[4][5] & 0x0000FF);
              iasqr = sf[4][6];
              gps_alm[isv].sqa = iasqr * c_2m11;
              if (gps_alm[isv].sqa > 0.0)
                gps_alm[isv].w = 19964981.84 / CUBE (gps_alm[isv].sqa);
              iaomega0 = sf[4][7];
              if (bit_test_l (iaomega0, 24))
		iaomega0 = ~( (~iaomega0) & 0xffffff);
              gps_alm[isv].lan = iaomega0 * c_2m23 * M_PI;
              iaw = sf[4][8];
              if (bit_test_l (iaw, 24))
		iaw = ~( (~iaw) & 0xffffff);
              gps_alm[isv].aop = iaw * c_2m23 * M_PI;
              iam0 = sf[4][9];
              if (bit_test_l (iam0, 24))
		iam0 = ~( (~iam0) & 0xffffff);
              gps_alm[isv].ma = iam0 * c_2m23 * M_PI;
              iaaf0 = (sf[4][10] >> 13) | ((sf[4][10] & 0x1C) >> 2);
              if (bit_test_l (iaaf0, 11))
		iaaf0 = ~( (~iaaf0) & 0x7ff);
              gps_alm[isv].af0 = iaaf0 * c_2m20;
              iaaf1 = (sf[4][10] & 0xFFE0) >> 5;
              if (bit_test_l (iaaf1, 11))
		iaaf1 = ~( (~iaaf1) & 0x7ff);
              gps_alm[isv].af1 = iaaf1 * c_2m38;
            }
          else if (i4page == 55)        /* page for text message */
            {
              gps_alm[prn].text_message[0] =
                (char) ((sf[4][3] & 0x00FF00) >> 8);
              gps_alm[prn].text_message[1] = (char) (sf[4][3] & 0x0000FF);
              for (k = 1; k <= 7; k++)
                {
                  gps_alm[prn].text_message[3 * k - 1] =
                    (char) (sf[4][k + 3] >> 16);
                  gps_alm[prn].text_message[3 * k] =
                    (char) ((sf[4][k + 3] & 0x00FF00) >> 8);
                  gps_alm[prn].text_message[3 * k + 1] =
                    (char) (sf[4][k + 3] & 0x0000FF);
                }
            }
          else if (i4page == 56)        /* page for broadcast Ionosphere Model & UTC Parameters */
            {
              ial0 = (int) ((sf[4][3] & 0x00FF00) >> 8);
              al0 = ial0 * c_2m30;
              ial1 = (int) (sf[4][3] & 0x0000FF);
              al1 = ial1 * c_2m27;
              ial2 = (int) (sf[4][4] >> 16);
              al2 = ial2 * c_2m24;
              ial3 = (int) ((sf[4][4] & 0x00FF00) >> 8);
              al3 = ial3 * c_2m24;
              ibt0 = (int) (sf[4][4] & 0x0000FF);
              b0 = ibt0 * 2048.;
              ibt1 = (int) (sf[4][5] >> 16);
              b1 = ibt1 * 16384.;
              ibt2 = (int) ((sf[4][5] & 0x00FF00) >> 8);
              b2 = ibt2 * 65536.;
              ibt3 = (int) (sf[4][5] & 0x00FF);
              b3 = ibt3 * 65536.;
              ia1 = sf[4][6];
              if (bit_test_l (ia1, 24))
		ia1 = ~( (~ia1) & 0xffffff);
              a1 = ia1 * c_2m50;
              ia0 = (sf[4][7] << 8) | (sf[4][8] >> 16);
              a0 = ia0 * c_2m30;
              itot = (int) ((sf[4][8] & 0x00FF00) >> 8);
              tot = (float) itot * 4096.0;
              iWNt = (int) (sf[4][8] & 0xFF);
              WNt = iWNt;
              idtls = (int) (sf[4][10] >> 16);
              if (idtls > 128)
                idtls = idtls | 0xFF00;
              dtls = idtls;
              iWNlsf = (int) ((sf[4][9] & 0x00FF00) >> 8);
              WNlsf = iWNlsf;
              iDN = (int) (sf[4][9] & 0x0000FF);
              DN = iDN;
              idtlsf = (int) (sf[4][9] >> 16);
              if (idtlsf > 128)
                idtlsf = idtlsf | 0xFF00;
              dtlsf = idtlsf;
            }
          else if (i4page == 63)        /* page for constellation health */
            {
              ASV[1] = (int) ((sf[4][3] & 0x00F000) >> 12);
              ASV[2] = (int) ((sf[4][3] & 0x000F00) >> 8);
              ASV[3] = (int) ((sf[4][3] & 0x0000F0) >> 4);
              ASV[4] = (int) (sf[4][3] & 0x00000F);
              ASV[5] = (int) (sf[4][4] >> 20);
              ASV[6] = (int) ((sf[4][4] & 0x0F0000L) >> 16);
              ASV[7] = (int) ((sf[4][4] & 0x00F000) >> 12);
              ASV[8] = (int) ((sf[4][4] & 0x000F00) >> 8);
              ASV[9] = (int) ((sf[4][4] & 0x0000F0) >> 4);
              ASV[10] = (int) (sf[4][4] & 0x00000F);
              ASV[11] = (int) (sf[4][5] >> 20);
              ASV[12] = (int) ((sf[4][5] & 0x0F0000L) >> 16);
              ASV[13] = (int) ((sf[4][5] & 0x00F000) >> 12);
              ASV[14] = (int) ((sf[4][5] & 0x000F00) >> 8);
              ASV[15] = (int) ((sf[4][5] & 0x0000F0) >> 4);
              ASV[16] = (int) (sf[4][5] & 0x00000F);
              ASV[17] = (int) (sf[4][6] >> 20);
              ASV[18] = (int) ((sf[4][6] & 0x0F0000L) >> 16);
              ASV[19] = (int) ((sf[4][6] & 0x00F000) >> 12);
              ASV[20] = (int) ((sf[4][6] & 0x000F00) >> 8);
              ASV[21] = (int) ((sf[4][6] & 0x0000F0) >> 4);
              ASV[22] = (int) (sf[4][6] & 0x00000F);
              ASV[23] = (int) (sf[4][7] >> 20);
              ASV[24] = (int) ((sf[4][7] & 0x0F0000L) >> 16);
              ASV[25] = (int) ((sf[4][7] & 0x00F000) >> 12);
              ASV[26] = (int) ((sf[4][7] & 0x000F00) >> 8);
              ASV[27] = (int) ((sf[4][7] & 0x0000F0) >> 4);
              ASV[28] = (int) (sf[4][7] & 0x00000F);
              ASV[29] = (int) (sf[4][8] >> 20);
              ASV[30] = (int) ((sf[4][8] & 0x0F0000L) >> 16);
              ASV[31] = (int) ((sf[4][8] & 0x00F000) >> 12);
              ASV[32] = (int) ((sf[4][8] & 0x000F00) >> 8);
              SVh[25] = (int) (sf[4][8] & 0x00003F);
              if (SVh[25] == 0x3f)
                gps_alm[25].inc = 0.0;
              SVh[26] = (int) (sf[4][9] >> 18);
              if (SVh[26] == 0x3f)
                gps_alm[26].inc = 0.0;
              SVh[27] = (int) ((sf[4][9] & 0x03F000L) >> 12);
              if (SVh[27] == 0x3f)
                gps_alm[27].inc = 0.0;
              SVh[28] = (int) ((sf[4][9] & 0x000FC0) >> 6);
              if (SVh[28] == 0x3f)
                gps_alm[28].inc = 0.0;
              SVh[29] = (int) (sf[4][9] & 0x00003F);
              if (SVh[29] == 0x3f)
                gps_alm[29].inc = 0.0;
              SVh[30] = (int) (sf[4][10] >> 18);
              if (SVh[30] == 0x3f)
                gps_alm[30].inc = 0.0;
              SVh[31] = (int) ((sf[4][10] & 0x03F000L) >> 12);
              if (SVh[31] == 0x3f)
                gps_alm[31].inc = 0.0;
              SVh[32] = (int) ((sf[4][10] & 0x000FC0) >> 6);
              if (SVh[32] == 0x3f)
                gps_alm[32].inc = 0.0;
            }
        }
      i5data = (int) (sf[5][3] >> 22);
      i5p = (int) ((sf[5][3] & 0x3F0000L) >> 16);
      if (i5page != (unsigned) i5p && i5data == 1)
        {
          schan[ch].page5 = i5p;
          i5page = i5p;
          if (i5page == 51)     /* page for constellation health */
            {
              iatoa = (int) ((sf[5][3] & 0xFF00) >> 8);
              SVh[1] = (int) (sf[5][4] >> 18);
              if (SVh[1] == 0x3f)
                gps_alm[1].inc = 0.0;
              SVh[2] = (int) ((sf[5][4] & 0x03F000L) >> 12);
              if (SVh[2] == 0x3f)
                gps_alm[2].inc = 0.0;
              SVh[3] = (int) ((sf[5][4] & 0x000FC0) >> 6);
              if (SVh[3] == 0x3f)
                gps_alm[3].inc = 0.0;
              SVh[4] = (int) (sf[5][4] & 0x00003F);
              if (SVh[4] == 0x3f)
                gps_alm[4].inc = 0.0;
              SVh[5] = (int) (sf[5][5] >> 18);
              if (SVh[5] == 0x3f)
                gps_alm[5].inc = 0.0;
              SVh[6] = (int) ((sf[5][5] & 0x03F000L) >> 12);
              if (SVh[6] == 0x3f)
                gps_alm[6].inc = 0.0;
              SVh[7] = (int) ((sf[5][5] & 0x000FC0) >> 6);
              if (SVh[7] == 0x3f)
                gps_alm[7].inc = 0.0;
              SVh[8] = (int) (sf[5][5] & 0x00003F);
              if (SVh[8] == 0x3f)
                gps_alm[8].inc = 0.0;
              SVh[9] = (int) (sf[5][6] >> 18);
              if (SVh[9] == 0x3f)
                gps_alm[9].inc = 0.0;
              SVh[10] = (int) ((sf[5][6] & 0x03F000L) >> 12);
              if (SVh[10] == 0x3f)
                gps_alm[10].inc = 0.0;
              SVh[11] = (int) ((sf[5][6] & 0x000FC0) >> 6);
              if (SVh[11] == 0x3f)
                gps_alm[11].inc = 0.0;
              SVh[12] = (int) (sf[5][6] & 0x00003F);
              if (SVh[12] == 0x3f)
                gps_alm[12].inc = 0.0;
              SVh[13] = (int) (sf[5][7] >> 18);
              if (SVh[13] == 0x3f)
                gps_alm[13].inc = 0.0;
              SVh[14] = (int) ((sf[5][7] & 0x03F000L) >> 12);
              if (SVh[14] == 0x3f)
                gps_alm[14].inc = 0.0;
              SVh[15] = (int) ((sf[5][7] & 0x000FC0) >> 6);
              if (SVh[15] == 0x3f)
                gps_alm[15].inc = 0.0;
              SVh[16] = (int) (sf[5][7] & 0x00003F);
              if (SVh[16] == 0x3f)
                gps_alm[16].inc = 0.0;
              SVh[17] = (int) (sf[5][8] >> 18);
              if (SVh[17] == 0x3f)
                gps_alm[17].inc = 0.0;
              SVh[18] = (int) ((sf[5][8] & 0x03F000L) >> 12);
              if (SVh[18] == 0x3f)
                gps_alm[18].inc = 0.0;
              SVh[19] = (int) ((sf[5][8] & 0x000FC0) >> 6);
              if (SVh[19] == 0x3f)
                gps_alm[19].inc = 0.0;
              SVh[20] = (int) (sf[5][8] & 0x00003F);
              if (SVh[20] == 0x3f)
                gps_alm[20].inc = 0.0;
              SVh[21] = (int) (sf[5][9] >> 18);
              if (SVh[21] == 0x3f)
                gps_alm[21].inc = 0.0;
              SVh[22] = (int) ((sf[5][9] & 0x03F000L) >> 12);
              if (SVh[22] == 0x3f)
                gps_alm[22].inc = 0.0;
              SVh[23] = (int) ((sf[5][9] & 0x000FC0) >> 6);
              if (SVh[23] == 0x3f)
                gps_alm[23].inc = 0.0;
              SVh[24] = (int) (sf[5][9] & 0x00003F);
              if (SVh[24] == 0x3f)
                gps_alm[24].inc = 0.0;
            }
          else
            {                   /* pages for almanacs of PRN 1 -> 24 */
              isv = i5page;
              gps_alm[isv].week = gps_week % 1024;
              iae = (int) (sf[5][3] & 0xFFFF);
              gps_alm[isv].ety = iae * c_2m21;
              iatoa = (int) (sf[5][4] >> 16);
              gps_alm[isv].toa = iatoa * c_2p12;
              iadeli = (int) (sf[5][4] & 0xFFFF);
              if (bit_test_l (iadeli, 16))
		iadeli = ~( (~iadeli) & 0xffff);
              gps_alm[isv].inc = (iadeli * c_2m19 + 0.3) * M_PI;
              iaomegad = (short int) (sf[5][5] >> 8);
              gps_alm[isv].rra = iaomegad * c_2m38 * M_PI;
              gps_alm[isv].health = (int) (sf[5][5] & 0xFF);
              iasqr = sf[5][6];
              gps_alm[isv].sqa = iasqr * c_2m11;
              if (gps_alm[isv].sqa > 0.0)
                gps_alm[isv].w = 19964981.84 / CUBE (gps_alm[isv].sqa);
              iaomega0 = sf[5][7];
              if (bit_test_l (iaomega0, 24))
		iaomega0 = ~( (~iaomega0) & 0xffffff);
              gps_alm[isv].lan = iaomega0 * c_2m23 * M_PI;
              iaw = sf[5][8];
              if (bit_test_l (iaw, 24))
		iaw = ~( (~iaw) & 0xffffff);
              gps_alm[isv].aop = iaw * c_2m23 * M_PI;
              iam0 = sf[5][9];
              if (bit_test_l (iam0, 24))
		iam0 = ~( (~iam0) & 0xffffff);
              gps_alm[isv].ma = iam0 * c_2m23 * M_PI;
              iaaf0 = (int) ((sf[5][10] >> 13) | ((sf[5][10] & 0x1C) >> 2));
              if (bit_test_l (iaaf0, 11))
		iaaf0 = ~( (~iaaf0) & 0x7ff);
              gps_alm[isv].af0 = iaaf0 * c_2m20;
              iaaf1 = (int) ((sf[5][10] & 0xFFE0) >> 5);
              if (bit_test_l (iaaf1, 11))
		iaaf1 = ~( (~iaaf1) & 0x7ff);
              gps_alm[isv].af1 = iaaf1 * c_2m38;
            }
        }
    }
  /*   gotoxy(1,24); */
  /*      printf("navmess->"); */

}

/*****************************************************************************
FUNCTION parity_check(void)
RETURNS  None.

PARAMETERS None.

PURPOSE  checks the parity of the 5 subframes of the nav message

WRITTEN BY
	Clifford Kelley

*****************************************************************************/

static void
parity_check (void)
{
  long pb1 = 0x3b1f3480L, pb2 = 0x1d8f9a40L, pb3 = 0x2ec7cd00L;
  long pb4 = 0x1763e680L, pb5 = 0x2bb1f340L, pb6 = 0x0b7a89c0L;
  int parity, m_parity;
  char d29 = 0, d30 = 0;
  int sfm, word;
  int err_bit;
  for (sfm = 1; sfm <= 5; sfm++)
    {
      p_error[sfm] = 0;
      for (word = 1; word <= 10; word++)
        {
          m_parity = (int) (sf[sfm][word] & 0x3f);
	  /**
	   * try new parity generation methodology
	   **/
          parity = exor (d29, sf[sfm][word] & pb1) << 1;
          parity = (parity | exor (d30, sf[sfm][word] & pb2)) << 1;
          parity = (parity | exor (d29, sf[sfm][word] & pb3)) << 1;
          parity = (parity | exor (d30, sf[sfm][word] & pb4)) << 1;
          parity = (parity | exor (0, sf[sfm][word] & pb5)) << 1;
          parity = parity | exor (d29 ^ d30, sf[sfm][word] & pb6);
          err_bit = 0;
          if (parity != m_parity)
            {
              err_bit = 1;
            }
          p_error[sfm] = (p_error[sfm] << 1) + err_bit;
          if (d30 == 1)
            sf[sfm][word] = 0x03fffffc0L & ~sf[sfm][word];
          sf[sfm][word] = sf[sfm][word] >> 6;
          d29 = (m_parity & 0x2) >> 1;
          d30 = m_parity & 0x1;
        }
    }
}

/*****************************************************************************
FUNCTION exor(char bit, long parity)
RETURNS  None.

PARAMETERS
			bit     char
			parity  long

PURPOSE
			count the number of bits set in the parameter parity and
			do an exclusive or with the parameter bit

WRITTEN BY
	Clifford Kelley

*****************************************************************************/
static int
exor (char bit, long parity)
{
  char i;
  int result;
  long v = parity;
  v >>= 6;
  result = 0;
  for (i = 7; i <= 30; i++) {
    if (v & 0x1)
      result++;
    v >>= 1;
  }
  result = (bit ^ result) & 0x1;
  return (result);
}

/*****************************************************************************
FUNCTION ecef_to_llh(ecef pos)
RETURNS  position in llh structure

PARAMETERS
			pos   ecef

PURPOSE    Convert a position in in cartesian ecef coordinates to
			  Geodetic WGS 84 coordinates

Based on equations found in Hoffman-Wellinhoff

WRITTEN BY
	Clifford Kelley

******************************************************************************/

llh
ecef_to_llh (ecef pos)
{
  double p, n, thet, esq, epsq;
  llh result;
  /*      gotoxy(1,24); */
  /*      printf("->ecef_to_llh"); */
  p = sqrt (pos.x * pos.x + pos.y * pos.y);
  thet = atan (pos.z * a / (p * b));
  esq = 1.0 - b * b / (a * a);
  epsq = a * a / (b * b) - 1.0;
  result.lat =
    atan ((pos.z + epsq * b * CUBE (sin (thet))) / (p -
                                                      esq * a *
                                                      CUBE (cos (thet))));
  result.lon = atan2 (pos.y, pos.x);
  n = a * a / sqrt (a * a * cos (result.lat) * cos (result.lat) +
                    b * b * sin (result.lat) * sin (result.lat));
  result.hae = p / cos (result.lat) - n;
  /*      gotoxy(1,24); */
  /*      printf("ecef_to_llh->"); */
  return (result);
}

/******************************************************************************
FUNCTION llh_to_ecef(llh pos)
RETURNS  position in ecef structure

PARAMETERS
			pos   llh

PURPOSE    Convert a position in Geodetic WGS 84 coordinates to cartesian
			  ecef coordinates

Based on equations found in Hoffman-Wellinhoff

WRITTEN BY
	Clifford Kelley

******************************************************************************/

ecef
llh_to_ecef (llh pos)
{
  double n;
  /*      gotoxy(1,24); */
  /*      printf("->llh_to_ecef"); */
  ecef result;
  n =
    a * a / sqrt (a * a * cos (pos.lat) * cos (pos.lat) +
                  b * b * sin (pos.lat) * sin (pos.lat));
  result.x = (n + pos.hae) * cos (pos.lat) * cos (pos.lon);
  result.y = (n + pos.hae) * cos (pos.lat) * sin (pos.lon);
  result.z = (b * b / (a * a) * n + pos.hae) * sin (pos.lat);
  /*      gotoxy(1,24); */
  /*      printf("llh_to_ecef->"); */
  return (result);
}

/******************************************************************************
FUNCTION pos_vel_time(int nsl)
RETURNS  None.

PARAMETERS
			nsl   int

PURPOSE

	This routine processes the all-in-view pseudorange to arrive
	at a receiver position

INPUTS:
    pseudo_range[nsl] Vector of measured range from satellites to the receiver
	 sat_location[nsl][3] Array of satellite locations in ECEF when the signal
								 was sent
    nsl      number of satellites used

OUTPUTS:
    RP[3]    VECTOR OF RECEIVER POSITION IN ECEF (X,Y,Z)
    CBIAS    RECEIVER CLOCK BIAS FROM GPS TIME

VARIABLES USED:
    C        SPEED OF LIGHT IN VACUUM IN M/S
    S[6][5]  MATRIX USED FOR SOLVING FOR RECEIVER POSITION CORRECTION
    B[5]     RESULTING RECEIVER CLOCK BIAS & POSITION CORRECTIONS
    X,Y,Z    TEMPORARY RECEIVER POSITION
    T        TEMPORARY RECEIVER CLOCK BIAS
    R[5]     RANGE FROM RECEIVER TO SATELLITES

IF THE POSITION CANNOT BE DETERMINED THE RESULT OF RP
WILL BE (0,0,0) THE CENTER OF THE EARTH

WRITTEN BY
	Clifford Kelley

******************************************************************************/

pvt
pos_vel_time (int nsl)
{
  double dd[5][5], r, ms[5][13], pm[5][13], bm[13], br[5], correct_mag=0, x, y,
    z, t;
  double a1, b1, c1, d1, e1, f1, g1, h1, i1, j1, k1, l1, m1, n1, o1, p1,
    denom, alpha;
  int i, j, k, nits;
  pvt result;
  /**
   *  USE ITERATIVE APPROACH TO SOLVING FOR THE POSITION OF
   *  THE RECEIVER
   **/
  /*      gotoxy(1,24); */
  /*      printf("->pos_vel_time"); */
  nits = 0;
  t = 0.0;
  x = rec_pos_xyz.x;
  y = rec_pos_xyz.y;
  z = rec_pos_xyz.z;
  /*printf("a  %lf,%lf,%lf\n",x,y,z); */
  do
    {
      for (i = 1; i <= nsl; i++)
        {
	  /**
	   * Compute range in ECI at the time of arrival at the receiver
	   **/
          alpha = (t - dt[i]) * omegae;
          r =
            sqrt (SQ (track_sat[i].x * cos (alpha) -
		      track_sat[i].y * sin (alpha) - x)
		  + SQ (track_sat[i].y * cos (alpha) +
			track_sat[i].x * sin (alpha) - y)
		  + SQ (track_sat[i].z - z));
          bm[i] = r - (dt[i] - t) * c;
          ms[1][i] =
            (track_sat[i].x * cos (alpha) - track_sat[i].y * sin (alpha) -
             x) / r;
          ms[2][i] =
            (track_sat[i].y * cos (alpha) + track_sat[i].x * sin (alpha) -
             y) / r;
          ms[3][i] = (track_sat[i].z - z) / r;
          ms[4][i] = 1.0;
        }
      a1 = 0.;
      b1 = 0.;
      c1 = 0.;
      d1 = 0.;
      e1 = 0.;
      f1 = 0.;
      g1 = 0.;
      h1 = 0.;
      i1 = 0.;
      j1 = 0.;
      k1 = 0.;
      l1 = 0.;
      m1 = 0.;
      n1 = 0.;
      o1 = 0.;
      p1 = 0.;
      for (k = 1; k <= nsl; k++)
        {
          a1 += ms[1][k] * ms[1][k];
          b1 += ms[1][k] * ms[2][k];
          c1 += ms[1][k] * ms[3][k];
          d1 += ms[1][k] * ms[4][k];
	  /* e1+=ms[2][k]*ms[1][k];for completeness, the matrix is symmetric */
          f1 += ms[2][k] * ms[2][k];
          g1 += ms[2][k] * ms[3][k];
          h1 += ms[2][k] * ms[4][k];
	  /*     i1+=ms[3][k]*ms[1][k]; */
	  /*     j1+=ms[3][k]*ms[2][k]; */
          k1 += ms[3][k] * ms[3][k];
          l1 += ms[3][k] * ms[4][k];
	  /*     m1+=ms[1][k]; */
	  /*     n1+=ms[2][k]; */
	  /*     o1+=ms[3][k]; */
          p1 += ms[4][k];
        }
      o1 = l1;
      m1 = d1;
      n1 = h1;
      e1 = b1;
      i1 = c1;
      j1 = g1;
/*
	  SOLVE FOR THE MATRIX INVERSE
*/
      denom =
        (k1 * p1 - l1 * o1) * (a1 * f1 - b1 * e1) + (l1 * n1 -
                                                     j1 * p1) * (a1 * g1 -
                                                                 c1 * e1) +
        (j1 * o1 - k1 * n1) * (a1 * h1 - d1 * e1) + (l1 * m1 -
                                                     i1 * p1) * (c1 * f1 -
                                                                 b1 * g1) +
        (i1 * o1 - k1 * m1) * (d1 * f1 - b1 * h1) + (i1 * n1 -
                                                     j1 * m1) * (c1 * h1 -
                                                                 d1 * g1);
      dd[1][1] =
        f1 * (k1 * p1 - l1 * o1) + g1 * (l1 * n1 - j1 * p1) + h1 * (j1 * o1 -
                                                                    k1 * n1);
      dd[1][2] =
        e1 * (l1 * o1 - k1 * p1) + g1 * (i1 * p1 - l1 * m1) + h1 * (k1 * m1 -
                                                                    i1 * o1);
      dd[1][3] =
        e1 * (j1 * p1 - n1 * l1) - i1 * (f1 * p1 - n1 * h1) + m1 * (f1 * l1 -
                                                                    j1 * h1);
      dd[1][4] =
        e1 * (n1 * k1 - j1 * o1) + i1 * (f1 * o1 - n1 * g1) + m1 * (j1 * g1 -
                                                                    f1 * k1);
      /*              dd[2][1]=b1*(l1*o1-k1*p1)+j1*(c1*p1-d1*o1)+n1*(d1*k1-c1*l1); */
      dd[2][1] = dd[1][2];
      dd[2][2] =
        a1 * (k1 * p1 - l1 * o1) + c1 * (l1 * m1 - i1 * p1) + d1 * (i1 * o1 -
                                                                    k1 * m1);
      dd[2][3] =
        a1 * (l1 * n1 - j1 * p1) + i1 * (b1 * p1 - n1 * d1) + m1 * (j1 * d1 -
                                                                    b1 * l1);
      dd[2][4] =
        a1 * (j1 * o1 - n1 * k1) - i1 * (b1 * o1 - n1 * c1) + m1 * (b1 * k1 -
                                                                    c1 * j1);
      /*              dd[3][1]=b1*(g1*p1-h1*o1)-f1*(c1*p1-o1*d1)+n1*(c1*h1-d1*g1); */
      dd[3][1] = dd[1][3];
      /*              dd[3][2]=a1*(o1*h1-g1*p1)+e1*(c1*p1-o1*d1)+m1*(d1*g1-c1*h1); */
      dd[3][2] = dd[2][3];
      dd[3][3] =
        a1 * (f1 * p1 - h1 * n1) + b1 * (h1 * m1 - e1 * p1) + d1 * (e1 * n1 -
                                                                    f1 * m1);
      dd[3][4] =
        a1 * (n1 * g1 - f1 * o1) + e1 * (b1 * o1 - c1 * n1) + m1 * (c1 * f1 -
                                                                    b1 * g1);
      /*              dd[4][1]=b1*(h1*k1-g1*l1)+f1*(c1*l1-d1*k1)+j1*(d1*g1-c1*h1); */
      dd[4][1] = dd[1][4];
      /*              dd[4][2]=a1*(g1*l1-h1*k1)-e1*(c1*l1-d1*k1)+i1*(c1*h1-d1*g1); */
      dd[4][2] = dd[2][4];
      /*              dd[4][3]=a1*(j1*h1-f1*l1)+e1*(b1*l1-d1*j1)+i1*(d1*f1-b1*h1); */
      dd[4][3] = dd[3][4];
      dd[4][4] =
        a1 * (f1 * k1 - g1 * j1) + b1 * (g1 * i1 - e1 * k1) + c1 * (e1 * j1 -
                                                                    f1 * i1);

      if (denom <= 0.0)
        {
          result.x = 1.0;       /* something went wrong */
          result.y = 1.0;       /* set solution to near center of earth */
          result.z = 1.0;
          result.dt = 0.0;
        }
      else
        {
          for (i = 1; i <= 4; i++)
            {
              for (j = 1; j <= 4; j++)
                dd[i][j] = dd[i][j] / denom;
            }
          for (i = 1; i <= nsl; i++)
            {
              for (j = 1; j <= 4; j++)
                {
                  pm[j][i] = 0.0;
                  for (k = 1; k <= 4; k++)
                    pm[j][i] += dd[j][k] * ms[k][i];
                }
            }
          for (i = 1; i <= 4; i++)
            {
              br[i] = 0.0;
              for (j = 1; j <= nsl; j++)
                br[i] += bm[j] * pm[i][j];
            }
          nits++;
          x = x + br[1];
          y = y + br[2];
          z = z + br[3];
          t = t - br[4] / c;
	  /*      printf("%lf,%lf,%lf,%20.10lf\n",x,y,z,t); */
	  /*      printf("%lf,%lf,%lf,%lf\n",br[1],br[2],br[3],br[4]); */
          correct_mag = sqrt (br[1] * br[1] + br[2] * br[2] + br[3] * br[3]);
	  /*               printf("mag=%lf\n",correct_mag); */
        }
    }
  while (correct_mag > 0.01 && correct_mag < 1.e8 && nits < 10);
  result.x = x;
  result.y = y;
  result.z = z;
  result.dt = t;
  /**
   * Now for Velocity
   **/

  for (i = 1; i <= nsl; i++)
    {
      alpha = (t - dt[i]) * omegae;
      r =
        sqrt (SQ (track_sat[i].x * cos (alpha) 
		  - track_sat[i].y * sin (alpha) - x)
	      + SQ (track_sat[i].y * cos (alpha) +
		    track_sat[i].x * sin (alpha) - y)
	      + SQ (track_sat[i].z - z));
      bm[i] =
        ((track_sat[i].x * cos (alpha) - track_sat[i].y * sin (alpha) -
          x) * d_sat[i].x + (track_sat[i].y * cos (alpha) +
                             track_sat[i].x * sin (alpha) - y) * d_sat[i].y +
         (track_sat[i].z - z) * d_sat[i].z) / r - dop_sign * meas_dop[i] * lambda;
    }
  for (i = 1; i <= 4; i++)
    {
      br[i] = 0.0;
      for (j = 1; j <= nsl; j++)
        br[i] += bm[j] * pm[i][j];
    }
  result.xv = br[1] + y * omegae;       /*  get rid of earth */
  result.yv = br[2] - x * omegae;       /*  rotation rate */
  result.zv = br[3];
  result.df = br[4] / c * 1000000.0;    /* frequency error in parts per million (ppm) */
  /*      gotoxy(1,24); */
  /*      printf("pos_vel_time->"); */
  return (result);
}

/******************************************************************************
FUNCTION dops(int nsl)

RETURNS  None.

PARAMETERS
			nsl  int

PURPOSE

	This routine computes the dops

INPUTS:
	 sat_location[nsl][3] Array of satellite locations in ECEF when the signal
								 was sent
    nsl      number of satellites used
    receiver position

OUTPUTS:
	 hdop = horizontal dilution of precision (rss of ndop & edop)
    vdop = vertical dilution of precision
    tdop = time dilution of precision
	 pdop = position dilution of precision (rss of vdop & hdop)
    gdop = geometric dilution of precision (rss of pdop & tdop)

WRITTEN BY
	Clifford Kelley

******************************************************************************/
#if defined VCPP
void
dops (int nsl)
{
  double r, xls, yls, zls;
  /*  double det; */
  int i;
  Matrix H (nsl, 4), G (4, 4);

  receiver.north.x = -cos (rec_pos_llh.lon) * sin (rec_pos_llh.lat);
  receiver.north.y = -sin (rec_pos_llh.lon) * sin (rec_pos_llh.lat);
  receiver.north.z = cos (rec_pos_llh.lat);
  receiver.east.x = -sin (rec_pos_llh.lon);
  receiver.east.y = cos (rec_pos_llh.lon);
  /*receiver.east.z=0.0; */
  receiver.up.x = cos (rec_pos_llh.lon) * cos (rec_pos_llh.lat);
  receiver.up.y = sin (rec_pos_llh.lon) * cos (rec_pos_llh.lat);
  receiver.up.z = sin (rec_pos_llh.lat);
  for (i = 1; i <= nsl; i++)
    {
      /**
       * Compute line of sight vectors
       **/
      xls = track_sat[i].x - rec_pos_xyz.x;
      yls = track_sat[i].y - rec_pos_xyz.y;
      zls = track_sat[i].z - rec_pos_xyz.z;

      r = sqrt (xls * xls + yls * yls + zls * zls);

      H (i, 1) =
        (xls * receiver.north.x + yls * receiver.north.y +
         zls * receiver.north.z) / r;
      H (i, 2) = (xls * receiver.east.x + yls * receiver.east.y) / r;
      H (i, 3) =
        (xls * receiver.up.x + yls * receiver.up.y + zls * receiver.up.z) / r;
      H (i, 4) = 1.0;
    }

  /*  G=(H.transpose()*H).inverse(); */  /* for Alberto's library */
  G = (H.t () * H).i ();        /* for Newmat library */

  hdop = sqrt (G (1, 1) + G (2, 2));
  vdop = sqrt (G (3, 3));
  tdop = sqrt (G (4, 4));
  pdop = sqrt (G (1, 1) + G (2, 2) + G (3, 3));
  gdop = sqrt (G (1, 1) + G (2, 2) + G (3, 3) + G (4, 4));
  /*  gotoxy(1,24); */
  /*  printf("->dops"); */
}
#elif ((defined BCPP) || (defined __TURBOC__))

void
dops (int nsl)
{
  double ddx, ddy, ddz, ddt, r, rxy, ms[5][13], x, y, z;
  double a1, b1, c1, d1, e1, f1, g1, h1, i1, j1, k1, l1, m1, n1, o1, p1,
    denom;
  double msx, msy, msz;
  int i, k;
  ecef north, east, up;
  /*      gotoxy(1,24); */
  /*      printf("->dops"); */
  x = rec_pos_xyz.x;
  y = rec_pos_xyz.y;
  z = rec_pos_xyz.z;
  r = sqrt (x * x + y * y + z * z);
  rxy = sqrt (x * x + y * y);
  north.x = -x / rxy * z / r;
  north.y = -y / rxy * z / r;
  north.z = rxy / r;
  east.x = -y / rxy;
  east.y = x / rxy;
  /*east.z=0.0; just for completeness */
  up.x = x / r;
  up.y = y / r;
  up.z = z / r;
  for (i = 1; i <= nsl; i++)
    {
      /**
       * Compute line of sight vectors
       **/
      r = sqrt (SQ (track_sat[i].x - x) +
                SQ (track_sat[i].y - y) + 
		SQ (track_sat[i].z - z));
      ms[1][i] = (track_sat[i].x - x) / r;
      ms[2][i] = (track_sat[i].y - y) / r;
      ms[3][i] = (track_sat[i].z - z) / r;
      ms[4][i] = 1.0;
    }
  for (i = 1; i <= nsl; i++)
    {
      msx = ms[1][i] * north.x + ms[2][i] * north.y + ms[3][i] * north.z;
      msy = ms[1][i] * east.x + ms[2][i] * east.y;
      msz = ms[1][i] * up.x + ms[2][i] * up.y + ms[3][i] * up.z;
      ms[1][i] = msx;
      ms[2][i] = msy;
      ms[3][i] = msz;
      ms[4][i] = 1.0;
    }
  a1 = 0.;
  b1 = 0.;
  c1 = 0.;
  d1 = 0.;
  e1 = 0.;
  f1 = 0.;
  g1 = 0.;
  h1 = 0.;
  i1 = 0.;
  j1 = 0.;
  k1 = 0.;
  l1 = 0.;
  m1 = 0.;
  n1 = 0.;
  o1 = 0.;
  p1 = 0.;
  for (k = 1; k <= nsl; k++)
    {
      a1 += ms[1][k] * ms[1][k];
      b1 += ms[1][k] * ms[2][k];
      c1 += ms[1][k] * ms[3][k];
      d1 += ms[1][k] * ms[4][k];
      /*   e1+=ms[2][k]*ms[1][k]; */
      f1 += ms[2][k] * ms[2][k];
      g1 += ms[2][k] * ms[3][k];
      h1 += ms[2][k] * ms[4][k];
      /*   i1+=ms[3][k]*ms[1][k]; */
      /*   j1+=ms[3][k]*ms[2][k]; */
      k1 += ms[3][k] * ms[3][k];
      l1 += ms[3][k] * ms[4][k];
      /*   m1+=ms[1][k]; */
      /*   n1+=ms[2][k]; */
      /*   o1+=ms[3][k]; */
      p1 += ms[4][k];
    }
  o1 = l1;
  m1 = d1;
  n1 = h1;
  e1 = b1;
  i1 = c1;
  j1 = g1;
  /**
   * SOLVE FOR THE DIAGONALS OF THE MATRIX INVERSE
   **/
  denom =
    (k1 * p1 - l1 * o1) * (a1 * f1 - b1 * e1) + (l1 * n1 -
                                                 j1 * p1) * (a1 * g1 -
                                                             c1 * e1) +
    (j1 * o1 - k1 * n1) * (a1 * h1 - d1 * e1) + (l1 * m1 -
                                                 i1 * p1) * (c1 * f1 -
                                                             b1 * g1) +
    (i1 * o1 - k1 * m1) * (d1 * f1 - b1 * h1) + (i1 * n1 -
                                                 j1 * m1) * (c1 * h1 -
                                                             d1 * g1);
  ddx = (f1 * (k1 * p1 - l1 * o1) + g1 * (l1 * n1 - j1 * p1) + h1 * (j1 * o1 - k1 * n1)) / denom;       /* ddx=ndop^2 */
  ddy = (a1 * (k1 * p1 - l1 * o1) + c1 * (l1 * m1 - i1 * p1) + d1 * (i1 * o1 - k1 * m1)) / denom;       /* ddy=edop^2 */
  ddz = (a1 * (f1 * p1 - h1 * n1) + b1 * (h1 * m1 - e1 * p1) + d1 * (e1 * n1 - f1 * m1)) / denom;       /* ddz=vdop^2 */
  ddt = (a1 * (f1 * k1 - g1 * j1) + b1 * (g1 * i1 - e1 * k1) + c1 * (e1 * j1 - f1 * i1)) / denom;       /* ddt=tdop^2 */
  if (denom <= 0.0)
    {
      hdop = 1.e6;              /* something went wrong */
      vdop = 1.e6;              /* set dops to a large number */
      tdop = 1.e6;
      pdop = 1.e6;
      gdop = 1.e6;
    }
  else
    {
      hdop = sqrt (ddx + ddy);  /* rss of ndop and edop */
      vdop = sqrt (ddz);
      tdop = sqrt (ddt);
      pdop = sqrt (ddx + ddy + ddz);    /* rss of ndop, edop, and vdop */
      gdop = sqrt (ddx + ddy + ddz + ddt);      /* rss of ndop, edop, vdop, and tdop */
    }
  /*      gotoxy(1,24); */
  /*      printf("dops->"); */

}
#else /* DJGPP or __linux__ */
void
dops (int nsl)
{
  double ddx, ddy, ddz, ddt, r, rxy, ms[5][13], x, y, z;
  double a1, b1, c1, d1, e1, f1, g1, h1, i1, j1, k1, l1, m1, n1, o1, p1,
    denom;
  double msx, msy, msz;
  int i, k;
  ecef north, east, up;
  /*      gotoxy(1,24); */
  /*      printf("->dops"); */
  x = rec_pos_xyz.x;
  y = rec_pos_xyz.y;
  z = rec_pos_xyz.z;
  r = sqrt (x * x + y * y + z * z);
  rxy = sqrt (x * x + y * y);
  north.x = -x / rxy * z / r;
  north.y = -y / rxy * z / r;
  north.z = rxy / r;
  east.x = -y / rxy;
  east.y = x / rxy;
  /*east.z=0.0; just for completeness */
  up.x = x / r;
  up.y = y / r;
  up.z = z / r;
  for (i = 1; i <= nsl; i++)
    {
      /**
       * Compute line of sight vectors
       **/
      r = sqrt (SQ (track_sat[i].x - x) +
                SQ (track_sat[i].y - y) + 
		SQ (track_sat[i].z - z));
      ms[1][i] = (track_sat[i].x - x) / r;
      ms[2][i] = (track_sat[i].y - y) / r;
      ms[3][i] = (track_sat[i].z - z) / r;
      ms[4][i] = 1.0;
    }
  for (i = 1; i <= nsl; i++)
    {
      msx = ms[1][i] * north.x + ms[2][i] * north.y + ms[3][i] * north.z;
      msy = ms[1][i] * east.x + ms[2][i] * east.y;
      msz = ms[1][i] * up.x + ms[2][i] * up.y + ms[3][i] * up.z;
      ms[1][i] = msx;
      ms[2][i] = msy;
      ms[3][i] = msz;
      ms[4][i] = 1.0;
    }
  a1 = 0.;
  b1 = 0.;
  c1 = 0.;
  d1 = 0.;
  e1 = 0.;
  f1 = 0.;
  g1 = 0.;
  h1 = 0.;
  i1 = 0.;
  j1 = 0.;
  k1 = 0.;
  l1 = 0.;
  m1 = 0.;
  n1 = 0.;
  o1 = 0.;
  p1 = 0.;
  for (k = 1; k <= nsl; k++)
    {
      a1 += ms[1][k] * ms[1][k];
      b1 += ms[1][k] * ms[2][k];
      c1 += ms[1][k] * ms[3][k];
      d1 += ms[1][k] * ms[4][k];
      /*   e1+=ms[2][k]*ms[1][k]; */
      f1 += ms[2][k] * ms[2][k];
      g1 += ms[2][k] * ms[3][k];
      h1 += ms[2][k] * ms[4][k];
      /*   i1+=ms[3][k]*ms[1][k]; */
      /*   j1+=ms[3][k]*ms[2][k]; */
      k1 += ms[3][k] * ms[3][k];
      l1 += ms[3][k] * ms[4][k];
      /*   m1+=ms[1][k]; */
      /*   n1+=ms[2][k]; */
      /*   o1+=ms[3][k]; */
      p1 += ms[4][k];
    }
  o1 = l1;
  m1 = d1;
  n1 = h1;
  e1 = b1;
  i1 = c1;
  j1 = g1;
  /**
   * SOLVE FOR THE DIAGONALS OF THE MATRIX INVERSE
   **/
  denom =
    (k1 * p1 - l1 * o1) * (a1 * f1 - b1 * e1) + (l1 * n1 -
                                                 j1 * p1) * (a1 * g1 -
                                                             c1 * e1) +
    (j1 * o1 - k1 * n1) * (a1 * h1 - d1 * e1) + (l1 * m1 -
                                                 i1 * p1) * (c1 * f1 -
                                                             b1 * g1) +
    (i1 * o1 - k1 * m1) * (d1 * f1 - b1 * h1) + (i1 * n1 -
                                                 j1 * m1) * (c1 * h1 -
                                                             d1 * g1);
  ddx = (f1 * (k1 * p1 - l1 * o1) + g1 * (l1 * n1 - j1 * p1) + h1 * (j1 * o1 - k1 * n1)) / denom;       /* ddx=ndop^2 */
  ddy = (a1 * (k1 * p1 - l1 * o1) + c1 * (l1 * m1 - i1 * p1) + d1 * (i1 * o1 - k1 * m1)) / denom;       /* ddy=edop^2 */
  ddz = (a1 * (f1 * p1 - h1 * n1) + b1 * (h1 * m1 - e1 * p1) + d1 * (e1 * n1 - f1 * m1)) / denom;       /* ddz=vdop^2 */
  ddt = (a1 * (f1 * k1 - g1 * j1) + b1 * (g1 * i1 - e1 * k1) + c1 * (e1 * j1 - f1 * i1)) / denom;       /* ddt=tdop^2 */
  if (denom <= 0.0)
    {
      hdop = 1.e6;              /* something went wrong */
      vdop = 1.e6;              /* set dops to a large number */
      tdop = 1.e6;
      pdop = 1.e6;
      gdop = 1.e6;
    }
  else
    {
      hdop = sqrt (ddx + ddy);  /* rss of ndop and edop */
      vdop = sqrt (ddz);
      tdop = sqrt (ddt);
      pdop = sqrt (ddx + ddy + ddz);    /* rss of ndop, edop, and vdop */
      gdop = sqrt (ddx + ddy + ddz + ddt);      /* rss of ndop, edop, vdop, and tdop */
    }
  /*      gotoxy(1,24); */
  /*      printf("dops->"); */

}
#endif

/******************************************************************************
FUNCTION tropo_iono(float az, float el, double gps_time)
RETURNS  signal time delay due to troposphere and ionosphere (single frequency)

PARAMETERS
			az         float
			el         float
			gps_time   double

PURPOSE
	This function corrects the pseudoranges with a tropospheric model
	and the broadcast ionospheric message corrections.

WRITTEN BY
	Clifford Kelley

******************************************************************************/

double
tropo_iono (int prn, float az, float el, double gps_time)
{
  double d_Trop, alt_factor;
  double d_Ion, psi, phi, lambdai, phim, per, x, F, amp, t;

  /*  Try a simple troposphere model */
  /*      gotoxy(1,24); */
  /*      printf("->tropo_iono"); */
  if (current_loc.hae > 200000.0)
    alt_factor = 0.0;
  else if (current_loc.hae < 0.0)
    alt_factor = 1.0;
  else
    alt_factor = exp (-current_loc.hae * 1.33e-4);
  if (m_tropo == 1)
    d_Trop = 2.47 / (sin (el) + .0121) * alt_factor / c;
  else
    d_Trop = 0.0;
  /*  if (out_kalman==1) fprintf(kalm,",az= %12.6f, el=%12.6f, tropo= %12.10lf", */
  /*                             az,el,d_Trop); */
  satellite[prn].Tropo = d_Trop;
  /*  if (el<(mask_angle-0.035)) */
  /*  { */
  /*              printf("ch=%d, el=%lf",ch,el*57.296); */
  /*              fprintf(kalm,"ch=%d, el=%lf",ch,el); */
  /*  } */
  /*  Use an ionosphere model */

  if (m_iono == 1)
    {
      psi = 0.0137 / (el / M_PI + 0.11) - .022;
      phi = current_loc.lat / M_PI + psi * cos (az);
      if (phi > 0.416)
        phi = 0.416;
      else if (phi < -0.416)
        phi = -0.416;
      lambdai = current_loc.lon / M_PI + psi * sin (az) / cos (phi * M_PI);
      t =
        43200.0 * lambdai + gps_time -
        (int) ((43200.0 * lambdai + gps_time) / 86400.) * 86400.;
      if (t < 0.0)
        t = t + 86400.0;
      phim = phi + 0.064 * cos ((lambdai - 1.617) * M_PI);
      /**
       * If available from the nav message use its Ionosphere model
       **/
      if (b0 != 0.0 && al0 != 0.0)
        {
          per = b0 + b1 * phim + b2 * phim * phim + b3 * phim * phim * phim;
          amp =
            al0 + al1 * phim + al2 * phim * phim + al3 * phim * phim * phim;
        }
      /**
       * else try this set of default iono model parameters
       **/
      else
        {
          per =
            141312.0 - 32768.0 * phim - 131072.0 * phim * phim -
            65536.0 * phim * phim * phim;
          amp =
            3.46e-8 + 7.45e-9 * phim - 1.19e-7 * phim * phim +
            5.96e-8 * phim * phim * phim;
        }
      if (per < 72000.0)
        per = 72000.0;
      x = 2. * M_PI * (t - 50400.) / per;
      F = 1.0 + 16.0 * CUBE (0.53 - el / M_PI);
      if (amp < 0.0)
        amp = 0.0;
      if (fabs (x) < 1.5707)
        d_Ion =
          F * (5.0e-9 + amp * (1.0 - x * x / 2. + x * x * x * x / 24.0));
      else
        d_Ion = F * 5.0e-9;
    }
  else
    d_Ion = 0.0;
  satellite[prn].Iono = d_Ion;
  /*      gotoxy(1,24); */
  /*      printf("tropo_iono->"); */
  return (d_Trop + d_Ion);
}

/******************************************************************************
FUNCTION read_ion_utc(void)
RETURNS  None.

PARAMETERS None.

PURPOSE
	This function reads the broadcast ionospheric correction model and the
	gps time to UTC conversion parameters from "ion_utc.dat" which is
	created by the program when the data has been read from the satellites

WRITTEN BY
	Clifford Kelley

******************************************************************************/

void
read_ion_utc (void)
{
  char text[27];
  if ((in = fopen (ion_utc_file, "r")) == NULL)
    {
      printf ("Cannot open %s file.\n",ion_utc_file);
    }
  else
    {
      handle = fileno (in);
      while (!feof (in))
        {
          fscanf (in, "%27c", text);
          fscanf (in, "%f", &al0);
          fscanf (in, "%27c", text);
          fscanf (in, "%f", &al1);
          fscanf (in, "%27c", text);
          fscanf (in, "%f", &al2);
          fscanf (in, "%27c", text);
          fscanf (in, "%f", &al3);
          fscanf (in, "%27c", text);
          fscanf (in, "%f", &b0);
          fscanf (in, "%27c", text);
          fscanf (in, "%f", &b1);
          fscanf (in, "%27c", text);
          fscanf (in, "%f", &b2);
          fscanf (in, "%27c", text);
          fscanf (in, "%f", &b3);
          fscanf (in, "%27c", text);
          fscanf (in, "%f", &a0);
          fscanf (in, "%27c", text);
          fscanf (in, "%f", &a1);
          fscanf (in, "%27c", text);
          fscanf (in, "%f", &dtls);
          fscanf (in, "%27c", text);
          fscanf (in, "%f", &tot);
          fscanf (in, "%27c", text);
          fscanf (in, "%f", &WNt);
          fscanf (in, "%27c", text);
          fscanf (in, "%f", &WNlsf);
          fscanf (in, "%27c", text);
          fscanf (in, "%f", &DN);
          fscanf (in, "%27c", text);
          fscanf (in, "%f", &dtlsf);
        }
      fclose (in);
    }
}

/******************************************************************************
FUNCTION read_almanac(void)
RETURNS  None.

PARAMETERS None.

PURPOSE
	This function reads the almanac parameters from "current.alm" which is
	created by the program when the data has been read from the satellites

WRITTEN BY
	Clifford Kelley

******************************************************************************/

static void
read_almanac (void)
{
  int id, health, week;
  float eccen, rinc, rras, sqra;
  float ratoa, aopg, rma, af0, af1, toa;
  char text[27];


  for (id = 1; id <= 32; id++)
  {
      gps_alm[id].health = 1; /* reset almanac so that even setting everything
                                 to zeros that could cause problems don't  */
  }
  almanac_valid = 0;
  if ((in = fopen (almanac_file, "r")) == NULL)
    {
      printf ("Cannot open %s\n",almanac_file);
      for (id = 1; id <= 32; id++)
        {
          gps_alm[id].week = gps_week % 1024 - 1;
          gps_alm[id].inc = 1.0;
        }
      /*     almanac_valid=0; */
    }
  else
    {
      status = warm_start;
      handle = fileno (in);
      while (!feof (in))
        {
          fscanf (in, "%45c", header);
          fscanf (in, "%27c", text);
          fscanf (in, "%d", &id);
          fscanf (in, "%27c", text);
          fscanf (in, "%i", &health);
          fscanf (in, "%27c", text);
          fscanf (in, "%f", &eccen);
          fscanf (in, "%27c", text);
          fscanf (in, "%f", &toa);
          fscanf (in, "%27c", text);
          fscanf (in, "%f", &rinc);
          fscanf (in, "%27c", text);
          fscanf (in, "%f", &rras);
          fscanf (in, "%27c", text);
          fscanf (in, "%f", &sqra);
          fscanf (in, "%27c", text);
          fscanf (in, "%f", &ratoa);
          fscanf (in, "%27c", text);
          fscanf (in, "%f", &aopg);
          fscanf (in, "%27c", text);
          fscanf (in, "%f", &rma);
          fscanf (in, "%27c", text);
          fscanf (in, "%f", &af0);
          fscanf (in, "%27c", text);
          fscanf (in, "%f", &af1);
          fscanf (in, "%27c", text);
          fscanf (in, "%i", &week);
          fscanf (in, "%c", &trailer);
          gps_alm[id].health = health;
          gps_alm[id].week = week;
          gps_alm[id].toa = toa;
          gps_alm[id].ety = eccen;
          gps_alm[id].toa = toa;
          gps_alm[id].inc = rinc;
          gps_alm[id].rra = rras;
          gps_alm[id].sqa = sqra;
          gps_alm[id].lan = ratoa;
          gps_alm[id].aop = aopg;
          gps_alm[id].ma = rma;
          gps_alm[id].af0 = af0;
          gps_alm[id].af1 = af1;
          gps_alm[id].sat_file = 0;
          if (gps_alm[id].sqa > 0.0)
            gps_alm[id].w = 19964981.84 / CUBE (gps_alm[id].sqa);
        }
      fclose (in);
      alm_gps_week = week;
      alm_toa = toa;
    }
}

/******************************************************************************
FUNCTION read_ephemeris(void)
RETURNS  None.

PARAMETERS None.

PURPOSE
	This function reads the ephemeris parameters from "current.eph" which is
	created by the program when the data has been read from the satellites

WRITTEN BY
	Clifford Kelley

******************************************************************************/

void
read_ephemeris ()
{
  int id, health, week, ura, iode, iodc;
  double toc, toe;
  double crc, crs, cic, cis, cuc, cus, tgd, ety, inc0, omegadot, w0, w, ma,
    dn, idot;
  double daf0, daf1, daf2, esqra;
  double d_toe;
  char text[27];

  if ((in = fopen (ephemeris_file, "r")) == NULL)
    {
      printf ("Cannot open %s file.\n",ephemeris_file);
    }
  else
    {
      handle = fileno (in);
      while (!feof (in))
        {
          fscanf (in, "%37c", header);
          fscanf (in, "%27c", text);
          fscanf (in, "%i", &id);
          fscanf (in, "%27c", text);
          fscanf (in, "%i", &health);
          fscanf (in, "%27c", text);
          fscanf (in, "%i", &week);
          fscanf (in, "%27c", text);
          fscanf (in, "%i", &ura);
          fscanf (in, "%27c", text);
          fscanf (in, "%le", &toe);
          fscanf (in, "%27c", text);
          fscanf (in, "%i", &iode);
          fscanf (in, "%27c", text);
          fscanf (in, "%le", &toc);
          fscanf (in, "%27c", text);
          fscanf (in, "%i", &iodc);
          fscanf (in, "%27c", text);
          fscanf (in, "%le", &tgd);
          fscanf (in, "%27c", text);
          fscanf (in, "%le", &daf0);
          fscanf (in, "%27c", text);
          fscanf (in, "%le", &daf1);
          fscanf (in, "%27c", text);
          fscanf (in, "%le", &daf2);
          fscanf (in, "%27c", text);
          fscanf (in, "%le", &ety);
          fscanf (in, "%27c", text);
          fscanf (in, "%le", &inc0);
          fscanf (in, "%27c", text);
          fscanf (in, "%le", &idot);
          fscanf (in, "%27c", text);
          fscanf (in, "%le", &omegadot);
          fscanf (in, "%27c", text);
          fscanf (in, "%le", &esqra);
          fscanf (in, "%27c", text);
          fscanf (in, "%le", &dn);
          fscanf (in, "%27c", text);
          fscanf (in, "%le", &w0);
          fscanf (in, "%27c", text);
          fscanf (in, "%le", &w);
          fscanf (in, "%27c", text);
          fscanf (in, "%le", &ma);
          fscanf (in, "%27c", text);
          fscanf (in, "%le", &cuc);
          fscanf (in, "%27c", text);
          fscanf (in, "%le", &cus);
          fscanf (in, "%27c", text);
          fscanf (in, "%le", &crc);
          fscanf (in, "%27c", text);
          fscanf (in, "%le", &crs);
          fscanf (in, "%27c", text);
          fscanf (in, "%le", &cic);
          fscanf (in, "%27c", text);
          fscanf (in, "%le", &cis);
          fscanf (in, "%c", &trailer);
          d_toe = clock_tow - toe;
          if (d_toe > 302400.0)
            d_toe = d_toe - 604800.0;
          else if (d_toe < -302400.0)
            d_toe = d_toe + 604800.0;
	  /**
	   * If week is current and time is less than 2 hours old use
	   * for hot start
	   *
	   *    note: gps_week is computed from the PCs real time clock
	   *          and does not roll over at 1024
	   **/
          if (week == (gps_week % 1024) && fabs (d_toe) <= 7500.0)
            {
              gps_eph[id].valid = 1;
              gps_eph[id].health = health;
              gps_eph[id].week = week;
              gps_eph[id].ura = ura;
              gps_eph[id].toe = toe;
              gps_eph[id].iode = iode;
              gps_eph[id].toc = toc;
              gps_eph[id].iodc = iodc;
              gps_eph[id].tgd = tgd;
              gps_eph[id].af0 = daf0;
              gps_eph[id].af1 = daf1;
              gps_eph[id].af2 = daf2;
              gps_eph[id].ety = ety;
              gps_eph[id].inc0 = inc0;
              gps_eph[id].idot = idot;
              gps_eph[id].omegadot = omegadot;
              gps_eph[id].sqra = esqra;
              gps_eph[id].dn = dn;
              gps_eph[id].w0 = w0;
              gps_eph[id].w = w;
              gps_eph[id].ma = ma;
              gps_eph[id].cuc = cuc;
              gps_eph[id].cus = cus;
              gps_eph[id].crc = crc;
              gps_eph[id].crs = crs;
              gps_eph[id].cic = cic;
              gps_eph[id].cis = cis;
              if (gps_eph[id].sqra > 0.0)
                gps_eph[id].wm = 19964981.84 / CUBE (gps_eph[id].sqra);
              if (out_debug)
                write_Debug_ephemeris (id);
            }
        }
      fclose (in);

    }
}

/******************************************************************************
FUNCTION write_almanac()
RETURNS  None.

PARAMETERS None.

PURPOSE
	This function writes the broadcast almanac data to a file for later
	use.  In particular to support a warm or hot start.

WRITTEN BY
	Clifford Kelley

******************************************************************************/

void
write_almanac ()
{
  int i;
/*   if (almanac_valid == 1) */
    {
      if ( (out = fopen (almanac_file, "w+")) == NULL) {
	fprintf (stderr,"Can not open %s for writing\n",almanac_file);
	return;
      }
      for (i = 1; i <= 32; i++)
        {
          if (gps_alm[i].inc > 0.0)
            {
              fprintf (out, "**** Week %4d almanac for PRN-%2d ***********\n",
                       gps_alm[i].week % 1024, i);
              fprintf (out, "ID:                         %3d\n", i);
              fprintf (out, "Health:                     %3d\n",
                       gps_alm[i].health);
              fprintf (out, "Eccentricity:               %10.9e\n",
                       gps_alm[i].ety);
              fprintf (out, "Time of Applicability(s):   %10.9e\n",
                       gps_alm[i].toa);
              fprintf (out, "Orbital Inclination(rad):   %10.9e\n",
                       gps_alm[i].inc);
              fprintf (out, "Rate of Right Ascen(R/s):   %10.9e\n",
                       gps_alm[i].rra);
              fprintf (out, "SQRT(A) (m^1/2):            %10.9e\n",
                       gps_alm[i].sqa);
              fprintf (out, "Right Ascen at TOA(rad):    %10.9e\n",
                       gps_alm[i].lan);
              fprintf (out, "Argument of Perigee(rad):   %10.9e\n",
                       gps_alm[i].aop);
              fprintf (out, "Mean Anom(rad):             %10.9e\n",
                       gps_alm[i].ma);
              fprintf (out, "Af0(s):                     %10.9e\n",
                       gps_alm[i].af0);
              fprintf (out, "Af1(s/s):                   %10.9e\n",
                       gps_alm[i].af1);
              fprintf (out, "week:                       %4d   \n",
                       gps_alm[i].week % 1024);
              fprintf (out, "\n");
            }
        }
      fclose (out);
    }
}

/******************************************************************************
FUNCTION write_ephemeris()
RETURNS  None.

PARAMETERS None.

PURPOSE
	This function writes the broadcast ephemeris data to a file for later
	use.  In particular to support a hot start.

WRITTEN BY
	Clifford Kelley

******************************************************************************/

void
write_ephemeris ()
{
  int i;
  out = fopen (ephemeris_file, "w+");
  for (i = 1; i <= 32; i++)
    {
      if (gps_eph[i].inc0 > 0.0)
        {
          fprintf (out, "**** Ephemeris for PRN-%2d ***********\n", i);
          fprintf (out, "ID:                         %3d\n", i);
          fprintf (out, "Health:                     %3d\n",
                   gps_eph[i].health);
          fprintf (out, "Week:                       %4d\n", gps_eph[i].week);
          fprintf (out, "URA:                        %3d\n", gps_eph[i].ura);
          fprintf (out, "E Time of Applic(s):        %12.11e\n",
                   gps_eph[i].toe);
          fprintf (out, "IODE:                       %4d\n", gps_eph[i].iode);
          fprintf (out, "C Time of Applic(s):        %12.11e\n",
                   gps_eph[i].toc);
          fprintf (out, "IODC:                       %4d\n", gps_eph[i].iodc);
          fprintf (out, "Tgd(s):                     %12.11e\n",
                   gps_eph[i].tgd);
          fprintf (out, "Af0(s):                     %12.11e\n",
                   gps_eph[i].af0);
          fprintf (out, "Af1(s/s):                   %12.11e\n",
                   gps_eph[i].af1);
          fprintf (out, "Af2(s/s/s):                 %12.11e\n",
                   gps_eph[i].af2);
          fprintf (out, "Eccentricity:               %12.11e\n",
                   gps_eph[i].ety);
          fprintf (out, "Orbital Inclination(rad):   %12.11e\n",
                   gps_eph[i].inc0);
          fprintf (out, "inc rate (r/s)              %12.11e\n",
                   gps_eph[i].idot);
          fprintf (out, "Rate of Right Ascen(R/s):   %12.11e\n",
                   gps_eph[i].omegadot);
          fprintf (out, "SQRT(A) (m^1/2):            %12.11e\n",
                   gps_eph[i].sqra);
          fprintf (out, "dn                          %12.11e\n",
                   gps_eph[i].dn);
          fprintf (out, "Right Ascen at TOE(rad):    %12.11e\n",
                   gps_eph[i].w0);
          fprintf (out, "Argument of Perigee(rad):   %12.11e\n",
                   gps_eph[i].w);
          fprintf (out, "Mean Anom(rad):             %12.11e\n",
                   gps_eph[i].ma);
          fprintf (out, "Cuc(rad):                   %12.11e\n",
                   gps_eph[i].cuc);
          fprintf (out, "Cus(rad):                   %12.11e\n",
                   gps_eph[i].cus);
          fprintf (out, "Crc(m):                     %12.11e\n",
                   gps_eph[i].crc);
          fprintf (out, "Crs(m):                     %12.11e\n",
                   gps_eph[i].crs);
          fprintf (out, "Cic(rad):                   %12.11e\n",
                   gps_eph[i].cic);
          fprintf (out, "Cis(rad):                   %12.11e\n",
                   gps_eph[i].cis);
          fprintf (out, "\n");
        }
    }
  fclose (out);
}

/******************************************************************************
FUNCTION write_ephemeris()
RETURNS  None.

PARAMETERS None.

PURPOSE
	This function writes the broadcast ephemeris data to a file for later
	use.  In particular to support a hot start.

WRITTEN BY
	Clifford Kelley

******************************************************************************/

void
write_Debug_ephemeris (int i)
{
  fprintf (debug, "**** Ephemeris for PRN-%2d ***********\n", i);
  fprintf (debug, "ID:                         %3d\n", i);
  fprintf (debug, "Health:                     %3d\n", gps_eph[i].health);
  fprintf (debug, "Week:                       %4d\n", gps_eph[i].week);
  fprintf (debug, "URA:                        %3d\n", gps_eph[i].ura);
  fprintf (debug, "E Time of Applic(s):        %12.11e\n", gps_eph[i].toe);
  fprintf (debug, "IODE:                       %4d\n", gps_eph[i].iode);
  fprintf (debug, "C Time of Applic(s):        %12.11e\n", gps_eph[i].toc);
  fprintf (debug, "IODC:                       %4d\n", gps_eph[i].iodc);
  fprintf (debug, "Tgd(s):                     %12.11e\n", gps_eph[i].tgd);
  fprintf (debug, "Af0(s):                     %12.11e\n", gps_eph[i].af0);
  fprintf (debug, "Af1(s/s):                   %12.11e\n", gps_eph[i].af1);
  fprintf (debug, "Af2(s/s/s):                 %12.11e\n", gps_eph[i].af2);
  fprintf (debug, "Eccentricity:               %12.11e\n", gps_eph[i].ety);
  fprintf (debug, "Orbital Inclination(rad):   %12.11e\n", gps_eph[i].inc0);
  fprintf (debug, "inc rate (r/s)              %12.11e\n", gps_eph[i].idot);
  fprintf (debug, "Rate of Right Ascen(R/s):   %12.11e\n",
           gps_eph[i].omegadot);
  fprintf (debug, "SQRT(A) (m^1/2):            %12.11e\n", gps_eph[i].sqra);
  fprintf (debug, "dn                          %12.11e\n", gps_eph[i].dn);
  fprintf (debug, "Right Ascen at TOE(rad):    %12.11e\n", gps_eph[i].w0);
  fprintf (debug, "Argument of Perigee(rad):   %12.11e\n", gps_eph[i].w);
  fprintf (debug, "Mean Anom(rad):             %12.11e\n", gps_eph[i].ma);
  fprintf (debug, "Cuc(rad):                   %12.11e\n", gps_eph[i].cuc);
  fprintf (debug, "Cus(rad):                   %12.11e\n", gps_eph[i].cus);
  fprintf (debug, "Crc(m):                     %12.11e\n", gps_eph[i].crc);
  fprintf (debug, "Crs(m):                     %12.11e\n", gps_eph[i].crs);
  fprintf (debug, "Cic(rad):                   %12.11e\n", gps_eph[i].cic);
  fprintf (debug, "Cis(rad):                   %12.11e\n", gps_eph[i].cis);
  fprintf (debug, "\n");

}


/******************************************************************************
FUNCTION write_ion_utc()
RETURNS  None.

PARAMETERS None.

PURPOSE
	This function writes the broadcast ionospheric correction data and the
      parameters to tie GPS time to UTC to a file for later use.

WRITTEN BY
	Clifford Kelley

******************************************************************************/

void
write_ion_utc ()
{
  out = fopen (ion_utc_file, "w+");
  fprintf (out, "al0:                        %e\n", al0);
  fprintf (out, "al1:                        %e\n", al1);
  fprintf (out, "al2:                        %e\n", al2);
  fprintf (out, "al3:                        %e\n", al3);
  fprintf (out, "b0:                         %f\n", b0);
  fprintf (out, "b1:                         %f\n", b1);
  fprintf (out, "b2:                         %f\n", b2);
  fprintf (out, "b3:                         %f\n", b3);
  fprintf (out, "a0                          %f\n", a0);
  fprintf (out, "a1                          %f\n", a1);
  fprintf (out, "dtls                        %f\n", dtls);
  fprintf (out, "tot                         %f\n", tot);
  fprintf (out, "WNt                         %f\n", WNt);
  fprintf (out, "WNlsf                       %f\n", WNlsf);
  fprintf (out, "DN                          %f\n", DN);
  fprintf (out, "dtlsf                       %f\n", dtlsf);
  fclose (out);
}
