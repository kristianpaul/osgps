/***********************************************************************
// Clifford Kelley <cwkelley@earthlink.net>
// This program is licensed under GNU GENERAL PUBLIC LICENSE
    Version 0.7 3/13/04 
	1.) Added Geoidal Seperation to GGA. Hard coded to 0.0
	2.) Set Mode to A in GAS
	Version 0.6 2/26/04 Removed channels structure definition
	Version 0.4 Added buad rate selection and NMEA sentence selection capablity
	Version 0.3 Turned off hardware flow control
	Version 0.2 Fixed SV in view bug
	Version 0.1 Initial release
***********************************************************************/
#include "stdlib.h"
#include "stdio.h"
#include "math.h"
#include "FwInter.h"
#include "nmea.h"
#include "structs.h"
#include "interfac.h"

/**
 * NMEA These function provide an interface API between the GPS
 * firmware and external modules, like the serial interface module.
 **/

extern unsigned int GPGGA;
extern unsigned int GPGSV;
extern unsigned int GPGSA;
extern unsigned int GPVTG;
extern unsigned int GPRMC;
extern unsigned int GPZDA;

void
GetNmeaSentenceEables (pNMEA_ENABLES pNmeaEnables)
{
  pNmeaEnables->GPGGA = GPGGA;
  pNmeaEnables->GPGSV = GPGSV;
  pNmeaEnables->GPGSA = GPGSA;
  pNmeaEnables->GPVTG = GPVTG;
  pNmeaEnables->GPRMC = GPRMC;
  pNmeaEnables->GPZDA = GPZDA;

}


#ifdef HOST_TEST

int
GetEastWest (void)
{
  if (GetLongitude () < 0)
    {
      return NMEA_WEST;
    }
  else
    {
      return NMEA_EAST;
    }
}

int
GetNorthSouth (void)
{
  if (GetLatitude () < 0)
    {
      return NMEA_SOUTH;
    }
  else
    {
      return NMEA_NORTH;
    }
}



int
GetFixQuality (void)
{
  return 1;
}

double
GetAltitude (void)
{
  return -33.8;
}

double
GetLongitude (void)
{
  return -121.570;
}

double
GetLongitudeAbs (void)
{
  return fabs (GetLongitude ());
}

double
GetLatitude (void)
{
  return 37.2994;
}

double
GetLatitudeAbs (void)
{
  return fabs (GetLatitude ());
}

int
GetHDOP (void)
{
  return 20;
}

int
GetNumberOfSVs (void)
{
  return 6;
}

double
GetTimeOfFixUTC (void)
{
  return 123519.89;
}

double
GetHrMinSecUTC (void)
{

  return 201530.8;
}

#else /* Embedded */

char gSimulate = 0;

char TempBuf[100];

#include <time.h>

/*double const pi=3.1415926535898E0,r_to_d=57.29577951308232; */
double const r_to_d = 57.29577951308232;


extern satvis xyz[33];
enum
{ cold_start, warm_start, hot_start, tracking, navigating };
extern struct interface_channel ichan[N_channels];
extern double speed;
extern double heading;
extern llh rec_pos_llh;
extern time_t thetime;
extern double m_time[];
extern float gdop;
extern float hdop;
extern float vdop;
extern float tdop;
extern int n_track;
extern int status;

int
GetEastWest (void)
{
  if (GetLongitude () < 0)
    {
      return NMEA_WEST;
    }
  else
    {
      return NMEA_EAST;
    }
}

int
GetNorthSouth (void)
{
  if (GetLatitude () < 0)
    {
      return NMEA_SOUTH;
    }
  else
    {
      return NMEA_NORTH;
    }
}

int
GetFixQuality (void)
{
  return 1;                     /* Cliff TODO */
}

double
GetAltitude (void)
{
  return rec_pos_llh.hae;       /* Cliff Is this in meters?  yes */
}

double
GetLongitude (void)
{
  return rec_pos_llh.lon * r_to_d;
}

double
GetLongitudeAbs (void)
{
  return fabs (GetLongitude ());
}

double
GetLatitude (void)
{
  return rec_pos_llh.lat * r_to_d;
}

double
GetLatitudeAbs (void)
{
  return fabs (GetLatitude ());
}

double
GetHDOP (void)
{
  return hdop;
}

double
GetVDOP (void)
{
  return vdop;
}

double
GetPDOP (void)
{
  return sqrt (vdop * vdop + hdop * hdop);      /* Cliff TODO */
}


void
GetTrackingPRNs (pTRACKING_PRNs pTrackingPRNs)
{
  int ch;

  for (ch = 0; ch < N_channels; ch++)
    {
      if (ichan[ch].state == track)
        {
          pTrackingPRNs[ch].PRN = ichan[ch].prn;
        }
    }
}

VISIBLE_SV_DATA VisibleSVData[N_channels];


pVISIBLE_SV_DATA
GetVisibleSVData (void)
{
  return VisibleSVData;
}

int
GetNumberOfVisibleSVs (void)
{
  int ch;
  int SVDataIndex = 0;
  int NumberOfVisibleSVs = 0;

  if (gSimulate)
    {
      for (ch = 0; ch < N_channels; ch++)
        {
          if (ichan[ch].state >= acquisition)
            {
              NumberOfVisibleSVs++;
              VisibleSVData[SVDataIndex].elevation =
                xyz[ichan[ch].prn].elevation * 57.3;
              VisibleSVData[SVDataIndex].azimuth =
                xyz[ichan[ch].prn].azimuth * 57.3;
              VisibleSVData[SVDataIndex].CNo = 23.0;    /* chan[ ch ].CNo; */
              VisibleSVData[SVDataIndex].PRN = ichan[ch].prn;
              SVDataIndex++;
            }

        }
    }
  else
    {

      for (ch = 0; ch < N_channels; ch++)
        {
          if (ichan[ch].state >= acquisition)
            {
              NumberOfVisibleSVs++;
              VisibleSVData[SVDataIndex].elevation =
                xyz[ichan[ch].prn].elevation * 57.3;
              VisibleSVData[SVDataIndex].azimuth =
                xyz[ichan[ch].prn].azimuth * 57.3;
              VisibleSVData[SVDataIndex].CNo = ichan[ch].CNo;
              VisibleSVData[SVDataIndex].PRN = ichan[ch].prn;
              SVDataIndex++;
            }

          /*              printf(" %2d %2d  %2d  %6ld   %6ld   %2d     %2d     %2d   %4d\n",
             ch, chan[ ch ].prn, chan[ ch ].state, chan[ch].TLM, chan[ch].TOW,
             gps_eph[ichan[ch].prn].health,gps_eph[ichan[ch].prn].valid,chan[ch].tow_sync,
             chan[ch].offset);

             printf(
             " %2d %2d  %2d  %3d   %4.0f  %3.0f   %6.0f   %4d  %4d  %2d  %3d  %3d%5d   %4.1f\n",
             ch,ichan[ch].prn,ichan[ch].state,chan[ch].n_freq,
             xyz[ichan[ch].prn].azimuth * 57.3, xyz[ichan[ch].prn].elevation * 57.3,
             xyz[ichan[ch].prn].doppler,chan[ch].t_count,chan[ch].n_frame,chan[ch].sfid,
             gps_eph[ichan[ch].prn].ura,chan[ch].page5,chan[ch].missed, chan[ ch ].CNo);






           */
        }
    }


  return NumberOfVisibleSVs;
}

int
GetNumberOfTrackingSVs (void)
{
  return n_track;
}

int
GetNavStatus (void)
{
  if (status == navigating)
    {
      return 1;
    }
  else
    {
      return 0;
    }
}

void
GetDayMonthYear (pDMY pDmy)
{
  struct tm *LocalTime;
  LocalTime = localtime (&thetime);
  pDmy->Day = LocalTime->tm_mday;
  pDmy->Month = LocalTime->tm_mon + 1;
  pDmy->Year = LocalTime->tm_year + 1900;
  return;
}

double
GetUTC (void)
{

  double UTCInt;
  struct tm *LocalTime;
  LocalTime = localtime (&thetime);

  UTCInt = (((double) LocalTime->tm_hour * 10000) +
            ((double) LocalTime->tm_min * 100) + (double) LocalTime->tm_sec);

  return UTCInt;

}

double
GetTimeOfFixUTC (void)
{
  return GetUTC ();             /* Cliff What var should I use here? */
}

double
GetHrMinSecUTC (void)
{
  return GetUTC ();             /* Cliff What var should I use here? */
}

double
GetGroundSpeedKnots (void)
{

  return speed;
}

double
GetTrueHeading (void)
{
  if (heading < 0)
    {
      return heading = 180 + (180 - ((fabs (heading) * r_to_d)));
    }
  else
    {
      return heading * r_to_d;
    }
}

double
GroundSpeedKilos (void)
{
  return 999.0;                 /* TODO */
}

double
GetHeightOfGeoid (void)
{
  return 0.0;                   /* TODO Cliff */
}

#endif
