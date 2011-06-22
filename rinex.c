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
#include  "rinex.h"

extern void gps2utc (double, int, int *, int *, int *, int *, int *,
		     double *);

/*****************************************************************************
FUNCTION rinex_head_obs()
RETURNS  None.

PARAMETERS None.

PURPOSE
        This function writes the header information to the RINEX observation 
        file.

WRITTEN BY
	Jonathan J. Makela (03-Oct-2006)

*****************************************************************************/
void
rinex_head_obs (int year, int month, int day, int hour, int minute,
		double second)
{
  char file_type[1] = "O";	/* Observation file */
  char date_str[20];

  sprintf (date_str, "%4d%02d%02d %02d:%02d:%02dGPS", year, month, day, hour,
	   minute, (int) second);

  fprintf (rinex_obs,
	   "%9.2f           %s                   %s                   RINEX VERSION / TYPE\n",
	   version, file_type, system_type);
  fprintf (rinex_obs, "%-20s%-20s%-20sPGM / RUN BY / DATE\n", program_name,
	   agency_name, date_str);
  fprintf (rinex_obs, "%-60sMARKER NAME\n", marker_name);
  fprintf (rinex_obs, "%20s%-40sOBSERVER / AGENCY\n", observer_name,
	   agency_name);
  fprintf (rinex_obs, "%-20s%-20s%-20sREC # / TYPE / VERS\n", receiver_number,
	   receiver_type, receiver_version);
  fprintf (rinex_obs, "%-20s%-20s                    ANT # / TYPE\n",
	   antenna_number, antenna_type);
  fprintf (rinex_obs,
	   "%14.4f%14.4f%14.4f                  APPROX POSITION XYZ\n", loc_x,
	   loc_y, loc_z);
  fprintf (rinex_obs,
	   "%14.4f%14.4f%14.4f                  ANTENNA: DELTA H/E/N\n", delx,
	   dely, delz);
  fprintf (rinex_obs,
	   "%6d%6d                                                WAVELENGTH FACT L1/2\n",
	   lamda_factor_L1, lamda_factor_L2);
  fprintf (rinex_obs,
	   "%6d    %2s    %2s                                          # / TYPES OF OBSERV\n",
	   n_obs, obs1, obs2);
  fprintf (rinex_obs,
	   "%6d%6d%6d%6d%6d%13.7f     %-3s         TIME OF FIRST OBS\n", year,
	   month, day, hour, minute, second, time_system);
  fprintf (rinex_obs,
	   "                                                            END OF HEADER\n");
}

/*****************************************************************************
FUNCTION rinex_head_nav()
RETURNS  None.

PARAMETERS None.

PURPOSE
        This function writes the header information to the RINEX navigation 
        file.

WRITTEN BY
	Jonathan J. Makela (03-Oct-2006)

*****************************************************************************/
void
rinex_head_nav (int year, int month, int day, int hour, int minute,
		double second)
{
  char file_type[1] = "N";	/* Navigation file */
  char date_str[20];

  sprintf (date_str, "%4d%02d%02d %02d:%02d:%02dGPS", year, month, day, hour,
	   minute, (int) second);

  fprintf (rinex_nav,
	   "%9.2f           %s                                        RINEX VERSION / TYPE\n",
	   version, file_type);
  fprintf (rinex_nav, "%-20s%-20s%-20sPGM / RUN BY / DATE\n", program_name,
	   agency_name, date_str);
  /*  fprintf(rinex_nav, "%6d                                                      LEAP SECONDS\n", (int) dtls); */
  fprintf (rinex_nav,
	   "                                                            END OF HEADER\n");
}

/*****************************************************************************
FUNCTION rinex_data_nav()
RETURNS  None.

PARAMETERS ephem - a ephemeris structure to be written
           prn - the satellite number

PURPOSE
        This function writes the RINEX navigation data to a file

WRITTEN BY
	Jonathan J. Makela (03-Oct-2006)

*****************************************************************************/
void
rinex_data_nav (ephemeris * ephem, int prn)
{
  int year, month, day, hour, minute;
  double second;

  /* FIX ME */
  float code_on_L2 = 0;
  float L2_P_flag = 0;
  float fit_interval = 0;
  float spare = 0;
  float URA;

  /* Calculate the URA */
  if (ephem->ura <= 6)
    URA = pow (2, 1 + ephem->ura / 2.0);
  else if (ephem->ura <= 15 && ephem->ura > 6)
    URA = pow (2, ephem->ura - 2.0);
  else
    URA = 0.0;

  /* Find the GPS date of the ephemeris (adding leap seconds which
     are automatically removed in the gps2utc routine. */
  gps2utc (ephem->toc + dtls, 1024 + ephem->week, &year, &month, &day, &hour,
	   &minute, &second);

  /* Output the data to the RINEX file */
  fprintf (rinex_nav, "%2d %02d %2d %2d %2d %2d%5.1f%19.12e%19.12e%19.12e\n",
	   prn, year, month, day, hour, minute, second, ephem->af0,
	   ephem->af1, ephem->af2);
  fprintf (rinex_nav, "   %19.12e%19.12e%19.12e%19.12e\n",
	   (float) ephem->iode, ephem->crs, ephem->dn, ephem->ma);
  fprintf (rinex_nav, "   %19.12e%19.12e%19.12e%19.12e\n", ephem->cuc,
	   ephem->ety, ephem->cus, ephem->sqra);
  fprintf (rinex_nav, "   %19.12e%19.12e%19.12e%19.12e\n", ephem->toe,
	   ephem->cic, ephem->w0, ephem->cis);
  fprintf (rinex_nav, "   %19.12e%19.12e%19.12e%19.12e\n", ephem->inc0,
	   ephem->crc, ephem->w, ephem->omegadot);
  fprintf (rinex_nav, "   %19.12e%19.12e%19.12e%19.12e\n", ephem->idot,
	   code_on_L2, (float) ephem->week + 1024, L2_P_flag);
  fprintf (rinex_nav, "   %19.12e%19.12e%19.12e%19.12e\n", (float) ephem->ura,
	   (float) ephem->health, ephem->tgd, (float) ephem->iodc);
  fprintf (rinex_nav, "   %19.12e%19.12e%19.12e%19.12e\n", (float) clock_tow,
	   fit_interval, spare, spare);
}
