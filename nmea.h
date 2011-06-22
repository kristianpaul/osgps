#ifndef NMEA_H
#define NMEA_H
/***********************************************************************
// Clifford Kelley <cwkelley@earthlink.net>
// This program is licensed under BSD LICENSE
    Version 0.7 3/13/04 
	1.) Added Geoidal Seperation to GGA. Hard coded to 0.0
	2.) Set Mode to A in GAS
	Version 0.6 2/26/04 Removed channels structure definition
	Version 0.5 2/22/04  Fixed bug in checksum routine 
	Version 0.4 Added buad rate selection and NMEA sentence selection capablity
	Version 0.3 Turned off hardware flow control
	Version 0.2 Fixed SV in view bug
	Version 0.1 Initial release
***********************************************************************/





/* #define SIM_MODE **/



#define MAX_NMEA_FIELD	 25
#define MAX_NMEA_BUFFER  300
#define MAX_NMEA_SENTENCE 82



/**
 *  Sentence strings.
 **/
#define NMEA_SENTENCE_START "$"
#define NMEA_SENTENCE_COMMA ","
#define NMEA_SENTENCE_CR    '0D'
#define NMEA_SENTENCE_LF    '0A'
#define NMEA_SENTENCE_ASTERISK "*"

#define NMEA_SENTENCE_DGPS_STATION_ID "0000"
#define NMEA_SENTENCE_DUMMY_CHECK_SUM "00"

#define NMEA_SENTENCE_NORTH "N"
#define NMEA_SENTENCE_SOUTH "S"
#define NMEA_SENTENCE_EAST  "E"
#define NMEA_SENTENCE_WEST  "W"

#define NMEA_UNITS_METERS   "M"

typedef enum _NMEA_E_W_N_S
{
  NMEA_EAST,
  NMEA_WEST,
  NMEA_NORTH,
  NMEA_SOUTH
} NMEA_E_W_N_S;



#define NMEA_SENTENCE_GPGGA "GPGGA"
#define NMEA_SENTENCE_GPGSV "GPGSV"
#define NMEA_SENTENCE_GPGSA "GPGSA"
#define NMEA_SENTENCE_GPVTG "GPVTG"
#define NMEA_SENTENCE_GPRMC "GPRMC"
#define NMEA_SENTENCE_GPZDA "GPZDA"


typedef enum _NMEA_STATUS
{
  NMEA_SUCCESS,
  NMEA_FAILED
} NMEA_STATUS;

typedef enum _NMEA_FIX_QUALITY
{
  NMEA_FIX_QUALITY_INVALID,
  NMEA_FIX_QUALITY_SPS
} NMEA_FIX_QUALITY;


typedef struct _NMEA_GGA_DATA
{
  double TimeOfFixUTC;          /* 0 */
  double Lat;                   /* 1 */
  int NorthSouth;               /* 2 */
  double Long;                  /* 3 */
  int EastWest;                 /* 4 */
  int FixQuality;               /* 5 */
  int NumberOfTrackingSV;       /* 6 */
  int HDOP;                     /* 7 */
  double Altitude;              /* 8 */
  int AltitudeUnits;            /* 9 */
  double HeightOfGeoid;         /* 10 Height of geoid above WGS84 ellipsoid */
  int GeoidUnits;               /* 11 */
  int TimeSinceLastDGPSFix;     /* 12 */
  int DGPSID;                   /* 13 */
} NMEA_GGA_DATA, *pNMEA_GGA_DATA;

typedef struct _SV_VIEW_DATA
{
  int Valid;                    /*  Does it contain data */
  int SatellitePRN;             /*  Satellite PRN  */
  int Elevation;                /*  Elevation, degrees */
  int Azimuth;                  /*  Azimuth */
  int SNR;                       
} SV_VIEW_DATA, *pSV_VIEW_DATA;

#define MAX_NMEA_SATELLITES_IN_VIEW 12

typedef struct _NMEA_GSV_DATA
{
  int NumberOfSentences;        /* 0 Number of sentences for full data */
  int SequenceNumber;           /* 1 */
  int NumberOfSatellites;       /* 2 */

  SV_VIEW_DATA SvViewData[MAX_NMEA_SATELLITES_IN_VIEW];

} NMEA_GSV_DATA, *pNMEA_GSV_DATA;

#define NMEA_TRUE_HEADING "T"
#define NMEA_MAG_HEADING  "M"
#define NMEA_KNOTS        "N"
#define NMEA_KILOS        "K"

typedef struct _NMEA_VTG_DATA
{
  double TrueHeading;
  double MagHeading;
  double GroundSpeedKnots;
  double GroundSpeedKilos;
} NMEA_VTG_DATA, *pNMEA_VTG_DATA;

#define NMEA_FIX_MODE_NO_FIX 1
#define NMEA_FIX_MODE_2D_FIX 2
#define NMEA_FIX_MODE_3D_FIX 3

#define NMEA_MODE_A_STR "A"
#define NMEA_MODE_M_STR "M"
#define NMEA_MODE_A      1
#define NMEA_MODE_M      2

typedef struct _NMEA_GSA_DATA
{
  /* Field 0  "A" = Auto selection "M" = Force  2D/3D fix (M = manual) */
  int Mode;               
  int FixMode;         /* Field 1  1 = No Fix 2 = 2D Fix 3 = 3D Fix */
  int PRNs[12];
  float PDOP;
  float HDOP;
  float VDOP;
} NMEA_GSA_DATA, *pNMEA_GSA_DATA;

#define NMEA_ACTIVE_STR "A"
#define NMEA_VOID_STR   "V"
#define NMEA_ACTIVE      1
#define NMEA_VOID        2

typedef struct _NMEA_RMC_DATA
{
  double TimeOfFixUTC;
  int Status;
  double Lat;
  int NorthSouth;
  double Long;
  int EastWest;
  double GroundSpeedKnots;
  double TrueHeading;
  unsigned long DateOfFix;
  int MagVar;
  int MagEastWest;
} NMEA_RMC_DATA, *pNMEA_RMC_DATA;

typedef struct _NMEA_ZDA_DATA
{
  double HrMinSecUTC;
  int Day;
  int Month;
  int Year;
  int LocalTimeZoneHr;
  int LocalTimeZoneMin;
} NMEA_ZDA_DATA, *pNMEA_ZDA_DATA;


void SendNMEA (void);

NMEA_STATUS NMEASendGPGGA (void);

NMEA_STATUS NMEAGetGPGGAData (pNMEA_GGA_DATA pNmeaData);

NMEA_STATUS NMEASendGPGSV (void);

NMEA_STATUS NMEAGetGPGSVData (pNMEA_GSV_DATA pNmeaGSVData);

NMEA_STATUS NMEASendGPGSA (void);

NMEA_STATUS NMEAGetGPGSAData (pNMEA_GSA_DATA pNmeaGSAData);

NMEA_STATUS NMEASendGPVTG (void);

NMEA_STATUS NMEAGetGPVTGData (pNMEA_VTG_DATA pNmeaVTGData);

NMEA_STATUS NMEASendGPRMC (void);

NMEA_STATUS NMEAGetGPRMCData (pNMEA_RMC_DATA pNmeaRMCData);

NMEA_STATUS NMEASendGPZDA (void);

NMEA_STATUS NMEAGetGPRZDAata (pNMEA_ZDA_DATA pNmeaZDAData);



void ftoa (float value, char *strFloat);
void ftod (double value, char *strDouble);
void ftod4 (double value, char *strDouble);
void ftodPrecision1 (double value, char *strDouble);


#endif /* NMEA_H */
