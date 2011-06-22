#ifndef FW_INTERFACE
#define FW_INTERFACE
/***********************************************************************
// Clifford Kelley <cwkelley@earthlink.net>
// This program is licensed under BSD LICENSE
    Version 0.7 3/13/04 
	1.) Added Geoidal Seperation to GGA. Hard coded to 0.0
	2.) Set Mode to A in GAS
	Version 0.6 2/26/04 Removed channels structure definition
	Version 0.4 Added buad rate selection and NMEA sentence selection capablity
	Version 0.3 Turned off hardware flow control
	Version 0.2 Fixed SV in view bug
	Version 0.1 Initial release
***********************************************************************/
typedef struct _NMEA_ENABLES
{
  unsigned int GPGGA;
  unsigned int GPGSV;
  unsigned int GPGSA;
  unsigned int GPVTG;
  unsigned int GPRMC;
  unsigned int GPZDA;

} NMEA_ENABLES, *pNMEA_ENABLES;

typedef struct _DMY
{
  int Day;
  int Month;
  int Year;
} DMY, *pDMY;

typedef struct _VISIBLE_SV_DATA
{
  int elevation;
  int azimuth;
  double CNo;
  int PRN;

} VISIBLE_SV_DATA, *pVISIBLE_SV_DATA;

typedef struct _TRACKING_PRNs
{
  unsigned int PRN;

} TRACKING_PRNs, *pTRACKING_PRNs;

int GetNavStatus (void);

double GetAltitude (void);

double GetLatitude (void);

double GetLatitudeAbs (void);

double GetLongitude (void);

double GetLongitudeAbs (void);

int GetFixQuality (void);

int GetNorthSouth (void);

int GetEastWest (void);

int GetNumberOfVisibleSVs (void);

int GetNumberOfTrackingSVs (void);

pVISIBLE_SV_DATA GetVisibleSVData (void);

void GetTrackingPRNs (pTRACKING_PRNs pTrackingPRNs);

/**
 * Date time stuff
 **/

void GetDayMonthYear (pDMY pDMY);

double GetTimeOfFixUTC (void);

double GetHrMinSecUTC (void);

double GetGroundSpeedKnots (void);

double GetTrueHeading (void);

double GroundSpeedKilos (void);

int GetNumberOfTrackingSVs (void);

double GetHDOP (void);

double GetVDOP (void);

double GetPDOP (void);

double GetHeightOfGeoid (void);

void GetNmeaSentenceEables (pNMEA_ENABLES pNmeaEnables);


#endif
