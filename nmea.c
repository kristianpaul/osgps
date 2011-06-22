#include <time.h>
#include <stdio.h>
#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "nmea.h"
#include "consts.h"
#include "structs.h"
#include "globals.h"
#include "FwInter.h"
#include "serport.h"

/*extern char gSimulate;*/

/***********************************************************************
// Clifford Kelley <cwkelley@earthlink.net>
// This program is licensed under GNU GENERAL PUBLIC LICENSE
    Version 0.7 3/13/04 
	1.) Added Geoidal Seperation to GGA. Hard coded to 0.0
	2.) Set Mode to A in GAS
    Version 0.6 2/26/04 Removed channels structure definition
	Version 0.5 2/22/04  Fixed bug in checksum routine 
	Version 0.4 Added baud rate selection and NMEA sentence
  	                selection capablity
	Version 0.3 Turned off hardware flow control
	Version 0.2 Fixed SV in view bug
	Version 0.1 Initial release
***********************************************************************/

void
SendNMEA (void)
{
  if (GPGGA != 0)
    NMEASendGPGGA ();           /* NMEA, use sentance options */
  if (GPGSA != 0)
    NMEASendGPGSA ();
  if (GPGSV != 0)
    NMEASendGPGSV ();
  if (GPRMC != 0)
    NMEASendGPRMC ();
  if (GPZDA != 0)
    NMEASendGPZDA ();
}

/**
 * NMEA Notes  
 *
 * Longitude and latitude can be expressed in several different
 * representations:
 *
 * dms (deg,min,sec) - Degrees, minutes and seconds format
 * (e.g. 5045'23.99")
 *
 * degrees - Decimal degrees (e.g. 50.7567) 
 *
 * radians - Radians (e.g. 0.88587090) 
 *
 * degrees, minutes - Degrees and minutes format (e.g. 5045.3998') 
 *
 * Select the appropriate representation found in the string files
 *
 * Converting Between Decimal Degrees, Degrees, Minutes and Seconds,
 * and Radians (dd + mm/60 +ss/3600) to Decimal degrees (dd.ff)
 *
 * dd = whole degrees, mm = minutes, ss = seconds
 *
 * dd.ff = dd + mm/60 + ss/3600
 *
 * Example: 30 degrees 15 minutes 22 seconds = 30 + 15/60 +
 * 22/3600 = 30.2561
 *
 * Decimal degrees (dd.ff) to (dd + mm/60 +ss/3600)
 *
 * For the reverse conversion, we want to convert dd.ff to dd mm
 * ss. Here ff = the fractional part of a decimal degree.
 *
 * mm = 60*ff
 *
 * ss = 60*(fractional part of mm)
 *
 * Use only the whole number part of mm in the final result.
 *
 * 30.2561 degrees = 30 degrees
 *
 * .2561*60 = 15.366 minutes
 *
 * .366 minutes = 22 seconds, so the final result is 30 degrees 15
 * minutes 22 seconds
 *
 * Decimal degrees (dd.ff) to Radians
 *
 * Radians = (dd.ff)*pi/180
 *
 * Radians to Decimal degrees (dd.ff)
 *
 * (dd.ff) = Radians*180/pi
 *
 * Degrees, Minutes and Seconds to Distance
 *
 * A degree of longitude at the equator is 111.2 kilometers. A minute
 * is 1853 meters.  A second is 30.9 meters. For other latitudes
 * multiply by cos(lat). Distances for degrees, minutes and seconds in
 * latitude are very similar and differ very slightly with latitude.
 * (Before satellites, observing those differences was a principal
 * method for determining the exact shape of the earth.)
 *
 *
 * D = Degrees
 * M = Minutes
 * S = Seconds
 * .m = Decimal Minutes
 * .s = Decimal Seconds
 *
 * DM.m = Degrees, Minutes, Decimal Minutes (eg. 45 22.6333)
 * D.d  = Degrees, Decimal Degrees (eg. 45.3772)
 * DMS  = Degrees, Minutes, Seconds (eg. 45 22'38")
 *
 * Process for Converting Latitude/Longitude Coordinates: 
 * 
 * 1) DMS --> DM.m  (45o22'38" --> 45o22.6333)
 *
 *  - Divide S by 60 to get .m (38/60=.6333)
 *  - Add .m to M to get M.m (22+.6333=22.6333)
 *
 * 2) DM.m --> D.d  (45o 22.6333 --> 45.3772)
 *
 *  - Divide M.m by 60 to get .d (22.6333/60=.3772)
 *  - Add .d to D to get D.d (45+.3772=45.3772)
 *
 * 3) D.d --> DM.m  ( 45.3772 --> 45 22.6333)
 *
 *  - Multiply .d by 60 to get M.m (.3772 * 60 = 22.6333)
 *
 * 4) DM.m --> DMS  (45o22.6333 --> 45 22'38")
 *
 *  - Multiply .m by 60 to get S(.6333*60=38)
 *
 *
 * Converting Degrees, Minutes, Seconds to Decimal Format
 *
 * latitude and longitude in a decimal format: 42.1361
 * 
 * latitude and longitude in degree, minute, second format: 42deg,
 * 08min, 10sec
 *
 * To convert coordinates from degrees, minutes, seconds format to
 * decimal format, use this easy formula:
 *
 * degrees + (minutes/60) +
 * (seconds/3600)
 *
 * The example coordinate above would be
 * calculated as:
 * 42 + (8/60) + (10/3600) = 42.1361
 * or
 * 42 + (.1333) + (.0028) = 42.1361
 *
 *
 **/

void
NMEAAddCRLF (char *TransmitBuffer, int *TXBufferIndex)
{
  char *TempPtr = TransmitBuffer + *TXBufferIndex;

  *TempPtr++ = 0x0D;
  *TempPtr++ = 0x0A;
  /**
   * Update the index.
   **/
  *TXBufferIndex += 2;
  *TempPtr++ = '\0';
}



void
NMEAAddField (char *TransmitBuffer, int *TXBufferIndex, char *StringToAdd)
{
  int NumberOfChars = 0;
  NumberOfChars = strlen (StringToAdd);

  /**
   * Copy the data.
   **/
  memcpy (&TransmitBuffer[*TXBufferIndex], StringToAdd, NumberOfChars);

  /**
   * Update the index.
   **/
  *TXBufferIndex += NumberOfChars;
}

double
ConvertLatLonFromDegreestoDegreesMinutes (double PositionDegrees)
{
  double Degrees = (int) PositionDegrees;
  double Minutes = PositionDegrees - Degrees;

  double DegreesMinutes = Minutes * 60 + (Degrees * 10 * 10);
  return DegreesMinutes;
}

/**
 * The checksum is the 8-bit exclusive OR (no start or stop bits) of
 * all characters in the sentence, including the "," delimiters,
 * between -- but not including -- the "$" and "*" delimiters.
 *
 * The hexadecimal value of the most significant and least significant
 * 4 bits of the result are converted to two ASCII characters (0-9,
 * A-F) for transmission. The most significant character is
 * transmitted first.
 **/
unsigned char
CalculateNmeaCheckSum (char *NmeaSentence)
{
  int Index;
  unsigned char NmeaCheckSum = 0;

  for (Index = 1; NmeaSentence[Index] != '*'; Index++)
    {
      NmeaCheckSum ^= NmeaSentence[Index];
    }

  return NmeaCheckSum;
}

void
NMEAAddCheckSum (char *TransmitBuffer, int *TXBufferIndex)
{
  unsigned char CheckSum;
  unsigned char TXCheckSum;
  char TempBuf[10];



  NMEAAddField (TransmitBuffer, TXBufferIndex, NMEA_SENTENCE_ASTERISK);

  CheckSum = CalculateNmeaCheckSum (TransmitBuffer);

  TXCheckSum = CheckSum & 0xf0;
  TempBuf[0] = (TXCheckSum >> 4) + '0';
  TempBuf[1] = '\0';

  if (TempBuf[0] > '9')
    {

      TempBuf[0] += 'A' - ('9' + 1);
      TempBuf[1] = '\0';
    }


  NMEAAddField (TransmitBuffer, TXBufferIndex, TempBuf);
  TXCheckSum = CheckSum & 0x0f;
  TempBuf[0] = (TXCheckSum & 0xf) + '0';
  TempBuf[1] = '\0';

  if (TempBuf[0] > '9')
    {

      TempBuf[0] += 'A' - ('9' + 1);
      TempBuf[1] = '\0';
    }

  NMEAAddField (TransmitBuffer, TXBufferIndex, TempBuf);
}

NMEA_STATUS
NMEAGetGPGGAData (pNMEA_GGA_DATA pNmeaGGAData)
{

  /**
   * Need to convert from degrees to degrees and minutes format.
   **/
  pNmeaGGAData->TimeOfFixUTC = GetTimeOfFixUTC ();
  pNmeaGGAData->Lat =
    ConvertLatLonFromDegreestoDegreesMinutes (GetLatitudeAbs ());
  pNmeaGGAData->NorthSouth = GetNorthSouth ();
  pNmeaGGAData->Long =
    ConvertLatLonFromDegreestoDegreesMinutes (GetLongitudeAbs ());
  pNmeaGGAData->EastWest = GetEastWest ();
  pNmeaGGAData->FixQuality = GetFixQuality ();
  pNmeaGGAData->NumberOfTrackingSV = GetNumberOfTrackingSVs ();
  pNmeaGGAData->HDOP = GetHDOP ();
  pNmeaGGAData->Altitude = GetAltitude ();
  pNmeaGGAData->HeightOfGeoid = GetHeightOfGeoid ();

  return NMEA_SUCCESS;
}

/**
 * GGA - essential fix data which provide 3D location and accuracy data. 
 *
 *  $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
 *
 * Where:
 *   GGA          Global Positioning System Fix Data
 *   123519       Fix taken at 12:35:19 UTC
 *   4807.038,N   Latitude 48 deg 07.038' N
 *   01131.000,E  Longitude 11 deg 31.000' E
 *   1            Fix quality: 0 = invalid
 *           1 = GPS fix (SPS)
 *           2 = DGPS fix
 *           3 = PPS fix
 *           4 = Real Time Kinematic
 *           5 = Float RTK
 *           6 = estimated (dead reckoning) (2.3 feature)
 *           7 = Manual input mode
 *           8 = Simulation mode
 *   08           Number of satellites being tracked
 *   0.9          Horizontal dilution of position
 *   545.4,M      Altitude, Meters, above mean sea level
 *   46.9,M       Height of geoid (mean sea level) above WGS84 ellipsoid
 *   (empty field) time in seconds since last DGPS update
 *   (empty field) DGPS station ID number
 *   *47          the checksum data, always begins with *
 *
 * If the height of geoid is missing then the altitude should be suspect. 
 *
 * Some non-standard implementations report altitude with respect to
 * the ellipsoid rather than geoid altitude. Some units do not report
 * negative altitudes at all.  This is the only sentence that reports
 * altitude.
 * 
 **/

NMEA_STATUS
NMEASendGPGGA (void)
{

  NMEA_GGA_DATA NmeaGGAData;
  char pFieldBuffer[MAX_NMEA_FIELD];
  int  TXBufferIndex = 0;
  char TransmitBuffer[MAX_NMEA_BUFFER];


  memset (TransmitBuffer, 0, MAX_NMEA_BUFFER);


  /**
   * Get the necessary data from the FW.
   **/
  NMEAGetGPGGAData (&NmeaGGAData);

  /**
   * Add the header
   **/
  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_START);
  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_GPGGA);
  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA);

  /**
   * Field 0 Time   
   *
   * The time field must be 6 digits long. If the hour is less than 10 you 
   * must pad it with an extra zero.
   **/
  sprintf (pFieldBuffer, "%06ld.00", (long) NmeaGGAData.TimeOfFixUTC);
  /*   ftod( NmeaGGAData.TimeOfFixUTC, pFieldBuffer  ); */
  NMEAAddField (TransmitBuffer, &TXBufferIndex, pFieldBuffer);
  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA);



  /**
   * Field 1 Latitude
   **/
  ftod4 (NmeaGGAData.Lat, pFieldBuffer);        /* 4 decimal places */
  NMEAAddField (TransmitBuffer, &TXBufferIndex, pFieldBuffer);
  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA);

  /**
   * Field 2 North/South 
   **/
  if (NmeaGGAData.NorthSouth == NMEA_SOUTH)
    {
      NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_SOUTH);
    }
  else
    {
      NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_NORTH);
    }

  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA);

  /**
   * Field 3 Longitude
   **/
  ftod4 (NmeaGGAData.Long, pFieldBuffer);       /* 4 decimal places */
  NMEAAddField (TransmitBuffer, &TXBufferIndex, pFieldBuffer);
  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA);


  /**
   * Field 4 East/West
   **/
  if (NmeaGGAData.EastWest == NMEA_WEST)
    {
      NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_WEST);
    }
  else
    {
      NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_EAST);
    }

  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA);

  /**
   * Field 5 Fix Quality
   **/
  sprintf (pFieldBuffer, "%d", NmeaGGAData.FixQuality);
  /* ftod( NmeaGGAData.FixQuality, pFieldBuffer ); */
  NMEAAddField (TransmitBuffer, &TXBufferIndex, pFieldBuffer);
  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA);

  /**
   * Field 6 Number of SV's used.
   **/
  sprintf (pFieldBuffer, "%02d", NmeaGGAData.NumberOfTrackingSV);
  /*itoa(  NmeaGGAData.NumberOfTrackingSV, pFieldBuffer, 10 );*/
  NMEAAddField (TransmitBuffer, &TXBufferIndex, pFieldBuffer);
  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA);

  /**
   * Field 7 HDOP
   **/
  ftodPrecision1 (NmeaGGAData.HDOP, pFieldBuffer);
  /* ftod( NmeaGGAData.HDOP, pFieldBuffer ); */
  NMEAAddField (TransmitBuffer, &TXBufferIndex, pFieldBuffer);
  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA);

  /**
   * Field 8 Altitude
   **/
  ftodPrecision1 (NmeaGGAData.Altitude, pFieldBuffer);
  /* ftod( NmeaGGAData.Altitude, pFieldBuffer ); */
  NMEAAddField (TransmitBuffer, &TXBufferIndex, pFieldBuffer);


  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA);
  /**
   * Field 9 Altitude Units
   **/
  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_UNITS_METERS);



  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA);
  /**
   * Field 10 Geoid Seperation ( Height of geoid above WGS84 ellipsoid )
   **/

  ftodPrecision1 (0.0, pFieldBuffer);
  NMEAAddField (TransmitBuffer, &TXBufferIndex, pFieldBuffer);

  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA);
  /**
   * Field 11 Geoid Seperation units
   **/
  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_UNITS_METERS);

  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA);

  /**
   * Field 12  DGPS Age
   **/

  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA);
  /**
   * Field 12  DGPS StationID
   **/


  /**
   * Check Sum 
   **/
  NMEAAddCheckSum (TransmitBuffer, &TXBufferIndex);

  /**
   * CR/LF
   **/
  NMEAAddCRLF (TransmitBuffer, &TXBufferIndex);




  /**
   * Now send the sentence.
   **/
  if (!ComPortWrite ((unsigned char *) &TransmitBuffer[0], TXBufferIndex))
    {
      return NMEA_FAILED;
    }
  else
    {
      return NMEA_SUCCESS;
    }
}

NMEA_STATUS
NMEAGetGPGSAData (pNMEA_GSA_DATA pNmeaGSVData)
{
  int i;
  TRACKING_PRNs TrackingPRNs[12];

  memset (TrackingPRNs, 0, sizeof (TrackingPRNs));

  pNmeaGSVData->Mode = NMEA_MODE_A;     /* Cliff */

  if (GetNavStatus ())
    {
      pNmeaGSVData->FixMode = NMEA_FIX_MODE_3D_FIX;
    }
  else
    {
      pNmeaGSVData->FixMode = NMEA_FIX_MODE_NO_FIX;
    }

/*  if (gSimulate)
    {
      pNmeaGSVData->FixMode = NMEA_FIX_MODE_3D_FIX;
    } */

  GetTrackingPRNs (&TrackingPRNs[0]);

  for (i = 0; i < 11; i++)
    {
      pNmeaGSVData->PRNs[i] = TrackingPRNs[i].PRN;
    }

/*  if (gSimulate)
    {
      pNmeaGSVData->PRNs[0] = 7;
      pNmeaGSVData->PRNs[1] = 9;
      pNmeaGSVData->PRNs[2] = 24;
      pNmeaGSVData->PRNs[3] = 4;
    }   */


  pNmeaGSVData->PDOP = GetPDOP ();      /* ( float )2.5; */
  pNmeaGSVData->HDOP = GetHDOP ();      /* ( float )1.3; */
  pNmeaGSVData->VDOP = GetVDOP ();      /* ( float )2.1; */

  return NMEA_SUCCESS;
}

/**
 * GSA - GPS DOP and active satellites
 **/

/** 
 * GSA - GPS DOP and active satellites. This sentence provides details
 * on the nature of the fix.  It includes the numbers of the
 * satellites being used in the current solution and the DOP. DOP
 * (dilution of precision) is an indication of the effect of satellite
 * geometry on the accuracy of the fix.  It is a unitless number where
 * smaller is better. For 3D fixes using 4 satellites a 1.0 would be
 * considered to be a perfect number, however for overdetermined
 * solutions it is possible to see numbers below 1.0.
 *
 *    There are differences in the way the PRN's are presented which
 * can effect the ability of some programs to display this data. For
 * example, in the example shown below there are 5 satellites in the
 * solution and the null fields are scattered indicating that the
 * almanac would show satellites in the null positions that are not
 * being used as part of this solution. Other receivers might output
 * all of the satellites used at the beginning of the sentence with
 * the null field all stacked up at the end. This difference accounts
 * for some satellite display programs not always being able to
 * display the satellites being tracked. Some units may show all
 * satellites that have ephemeris data without regard to their use as
 * part of the solution but this is non-standard.
 *
 *  $GPGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1*39
 *
 * Where:
 *     GSA      Satellite status
 *     A        Auto selection of 2D or 3D fix (M = manual) 
 *     3        3D fix - values include: 1 = no fix
 *                                       2 = 2D fix
 *                                       3 = 3D fix
 *     04,05... PRNs of satellites used for fix (space for 12) 
 *     2.5      PDOP (dilution of precision) 
 *     1.3      Horizontal dilution of precision (HDOP) 
 *     2.1      Vertical dilution of precision (VDOP)
 *     *39      the checksum data, always begins with *
 *
 **/
NMEA_STATUS
NMEASendGPGSA (void)
{

  NMEA_GSA_DATA NmeaGSAData;
  char pFieldBuffer[MAX_NMEA_FIELD];
  int  i, TXBufferIndex = 0;
  char TransmitBuffer[MAX_NMEA_BUFFER];


  memset (TransmitBuffer, 0, MAX_NMEA_BUFFER);


  /**
   * Get the necessary data from the FW.
   **/
  memset (&NmeaGSAData, 0, sizeof (NMEA_GSA_DATA));
  NMEAGetGPGSAData (&NmeaGSAData);

  /**
   * Add the header
   **/
  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_START);
  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_GPGSA);
  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA);

  /**
   * Field 0 Mode   
   **/
  if (NmeaGSAData.Mode == NMEA_MODE_A)
    {
      strcpy (pFieldBuffer, NMEA_MODE_A_STR);
      NMEAAddField (TransmitBuffer, &TXBufferIndex, pFieldBuffer);
    }
  else if (NmeaGSAData.Mode == NMEA_MODE_M)
    {
      strcpy (pFieldBuffer, NMEA_MODE_M_STR);
      NMEAAddField (TransmitBuffer, &TXBufferIndex, pFieldBuffer);
    }
  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA);

  /**
   * Field 1 FixMode
   **/
  sprintf (pFieldBuffer, "%d", NmeaGSAData.FixMode);
  NMEAAddField (TransmitBuffer, &TXBufferIndex, pFieldBuffer);
  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA);

  /**
   * Now the satellites
   **/
  for (i = 0; i < 12; i++)
    {
      if (NmeaGSAData.PRNs[i])
        {

          sprintf (pFieldBuffer, "%02d", NmeaGSAData.PRNs[i]);
	  /* itoa( NmeaGSAData.PRNs[ i ], pFieldBuffer, 10 ); */
          NMEAAddField (TransmitBuffer, &TXBufferIndex, pFieldBuffer);
          NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA);
        }
      else
        {
          NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA);
        }
    }

  /**
   * PDOP
   **/
  ftoa (NmeaGSAData.PDOP, pFieldBuffer);
  NMEAAddField (TransmitBuffer, &TXBufferIndex, pFieldBuffer);
  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA);

  /**
   * HDOP
   **/
  ftoa (NmeaGSAData.HDOP, pFieldBuffer);
  NMEAAddField (TransmitBuffer, &TXBufferIndex, pFieldBuffer);
  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA);

  /**
   * VDOP
   **/
  ftoa (NmeaGSAData.VDOP, pFieldBuffer);
  NMEAAddField (TransmitBuffer, &TXBufferIndex, pFieldBuffer);
  /*   NMEAAddField( TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA );*/

  /**
   * Check Sum 
   **/
  NMEAAddCheckSum (TransmitBuffer, &TXBufferIndex);

  /**
   * CR/LF
   **/
  NMEAAddCRLF (TransmitBuffer, &TXBufferIndex);


  /**
   * Now send the sentence.
   **/
  if (!ComPortWrite ((unsigned char *) &TransmitBuffer[0], TXBufferIndex))
    {
      return NMEA_FAILED;
    }
  else
    {
      return NMEA_SUCCESS;
    }
}



NMEA_STATUS
NMEAGetGPGSVData (pNMEA_GSV_DATA pNmeaGSVData)
{
  int Satellite;
  pVISIBLE_SV_DATA pVisibleSVData;
  pNmeaGSVData->NumberOfSatellites = GetNumberOfVisibleSVs ();

  /**
   * Calculate the number of sentences.
   **/
  if (pNmeaGSVData->NumberOfSatellites <= 4)
    {
      pNmeaGSVData->NumberOfSentences = 1;
    }
  else
    {
      pNmeaGSVData->NumberOfSentences = pNmeaGSVData->NumberOfSatellites / 4;

      if (pNmeaGSVData->NumberOfSatellites % 4)
        {
          pNmeaGSVData->NumberOfSentences += 1;
        }
    }

  pVisibleSVData = GetVisibleSVData ();


  for (Satellite = 0; Satellite < pNmeaGSVData->NumberOfSatellites;
       Satellite++)
    {
      pNmeaGSVData->SvViewData[Satellite].Valid = 1;

      /* pNmeaGSVData->SvViewData[ Satellite ].Azimuth                = pVisibleSVData[ Satellite ].azimuth; */

      if (pVisibleSVData[Satellite].azimuth < 0)
        {
          pNmeaGSVData->SvViewData[Satellite].Azimuth =
            180 + (180 - ((abs (pVisibleSVData[Satellite].azimuth))));
        }
      else
        {
          pNmeaGSVData->SvViewData[Satellite].Azimuth =
            pVisibleSVData[Satellite].azimuth;
        }

      pNmeaGSVData->SvViewData[Satellite].Elevation =
        pVisibleSVData[Satellite].elevation;
      pNmeaGSVData->SvViewData[Satellite].SatellitePRN =
        pVisibleSVData[Satellite].PRN;
      pNmeaGSVData->SvViewData[Satellite].SNR = pVisibleSVData[Satellite].CNo;
    }

  return NMEA_SUCCESS;
}

/** 
 * GSV - Satellites in View shows data about the satellites that the
 * unit might be able to find based on its viewing mask and almanac
 * data. It also shows current ability to track this data. Note that
 * one GSV sentence only can provide data for up to 4 satellites and
 * thus there may need to be 3 sentences for the full information. It
 * is reasonable for the GSV sentence to contain more satellites than
 * GGA might indicate since GSV may include satellites that are not
 * used as part of the solution. It is not a requirment that the GSV
 * sentences all appear in sequence. To avoid overloading the data
 * bandwidth some receivers may place the various sentences in totally
 * different samples since each sentence identifies which one it is.
 *
 * The field called SNR (Signal to Noise Ratio) in the NMEA standard
 * is often referred to as signal strength.  SNR is an indirect but
 * more useful value that raw signal strength. It can range from 0 to
 * 99 and has units of dB according to the NMEA standard, but the
 * various manufacturers send different ranges of numbers with
 * different starting numbers so the values themselves cannot
 * necessarily be used to evaluate different units. The range of
 * working values in a given gps will usually show a difference of
 * about 25 to 35 between the lowest and highest values, however 0 is
 * a special case and may be shown on satellites that are in view but
 * not being tracked.
 *
 *
 * $GPGSV,2,1,08,01,40,083,46,02,17,308,41,12,07,344,39,14,22,228,45*75
 *
 * Where:
 *      GSV          Satellites in view
 *      2            Number of sentences for full data
 *      1            sentence 1 of 2
 *      08           Number of satellites in view
 *
 *      01           Satellite PRN number
 *      40           Elevation, degrees
 *      083          Azimuth, degrees
 *      46           SNR - higher is better
 *           for up to 4 satellites per sentence
 *      *75          the checksum data, always begins with *
 *
 **/

NMEA_STATUS
NMEASendGPGSV (void)
{

  NMEA_GSV_DATA NmeaGSVData;
  char pFieldBuffer[MAX_NMEA_FIELD];
  int TXBufferIndex = 0;
  char TransmitBuffer[MAX_NMEA_BUFFER];
  TRACKING_PRNs TrackingPRNs[12];
  int Satellite = 0;
  int SequenceNumber;
  int NumSatelliteThisSentence;

  memset (TransmitBuffer, 0, MAX_NMEA_BUFFER);
  memset (&NmeaGSVData, 0, sizeof (NMEA_GSV_DATA));

  /**
   * Get the necessary data from the FW.
   **/
  NMEAGetGPGSVData (&NmeaGSVData);
  GetTrackingPRNs (&TrackingPRNs[0]);


  for (SequenceNumber = 0; SequenceNumber < NmeaGSVData.NumberOfSentences;
       SequenceNumber++)
    {
      TXBufferIndex = 0;

      /**
       * Add the header
       **/
      NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_START);
      NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_GPGSV);
      NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA);

      /**
       * Field 0 Number of sentences for full data 
       **/
      sprintf (pFieldBuffer, "%d", NmeaGSVData.NumberOfSentences);
      NMEAAddField (TransmitBuffer, &TXBufferIndex, pFieldBuffer);
      NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA);



      /**
       * Field 1 SequenceNumber
       **/
      sprintf (pFieldBuffer, "%d", SequenceNumber + 1);
      NMEAAddField (TransmitBuffer, &TXBufferIndex, pFieldBuffer);
      NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA);

      /**
       * Field 2 Number of satellites in view 
       **/
      sprintf (pFieldBuffer, "%02d", NmeaGSVData.NumberOfSatellites);
      /*itoa( NmeaGSVData.NumberOfSatellites, pFieldBuffer, 10 );*/
      NMEAAddField (TransmitBuffer, &TXBufferIndex, pFieldBuffer);
      NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA);

      /**
       * Now add the satellite data.
       **/
      for (NumSatelliteThisSentence = 0; NumSatelliteThisSentence < 4;
           NumSatelliteThisSentence++)
        {
          if (NmeaGSVData.SvViewData[Satellite].Valid)
            {
	      int i;
	      /**
	       * PRN
	       **/
              sprintf (pFieldBuffer, "%02d",
                       NmeaGSVData.SvViewData[Satellite].SatellitePRN);
              /* itoa( NmeaGSVData.SvViewData[ Satellite ].SatellitePRN, pFieldBuffer, 10 ); */
              NMEAAddField (TransmitBuffer, &TXBufferIndex, pFieldBuffer);
              NMEAAddField (TransmitBuffer, &TXBufferIndex,
                            NMEA_SENTENCE_COMMA);

              /**
	       * Elevation
	       **/
              sprintf (pFieldBuffer, "%02d",
                       NmeaGSVData.SvViewData[Satellite].Elevation);
              /*itoa( NmeaGSVData.SvViewData[ Satellite ].Elevation, pFieldBuffer, 10 );*/
              NMEAAddField (TransmitBuffer, &TXBufferIndex, pFieldBuffer);
              NMEAAddField (TransmitBuffer, &TXBufferIndex,
                            NMEA_SENTENCE_COMMA);

              /**
	       * Azimuth Needs to be 0 - 360
	       **/

              sprintf (pFieldBuffer, "%03d",
                       NmeaGSVData.SvViewData[Satellite].Azimuth);
              /* itoa( NmeaGSVData.SvViewData[ Satellite ].Azimuth, pFieldBuffer, 10 ); */
              NMEAAddField (TransmitBuffer, &TXBufferIndex, pFieldBuffer);
              NMEAAddField (TransmitBuffer, &TXBufferIndex,
                            NMEA_SENTENCE_COMMA);

              /**
	       * SNR
	       *
               *
	       * If we are not tracking then do not display
	       * the SNR.
	       **/
              for (i = 0; i < 11; i++)
                {
                  /**
		   * See if we are tracking this guy.
		   **/
                  if ((unsigned) 
        NmeaGSVData.SvViewData[Satellite].SatellitePRN ==
                      TrackingPRNs[i].PRN)
                    {
                      sprintf (pFieldBuffer, "%02d",
                               NmeaGSVData.SvViewData[Satellite].SNR);
                      NMEAAddField (TransmitBuffer, &TXBufferIndex,
                                    pFieldBuffer);
                      break;

                    }

                }

              if (NumSatelliteThisSentence < 3)
                {
                  NMEAAddField (TransmitBuffer, &TXBufferIndex,
                                NMEA_SENTENCE_COMMA);
                }

            }
          else
            {
              NMEAAddField (TransmitBuffer, &TXBufferIndex,
                            NMEA_SENTENCE_COMMA);
              NMEAAddField (TransmitBuffer, &TXBufferIndex,
                            NMEA_SENTENCE_COMMA);
              NMEAAddField (TransmitBuffer, &TXBufferIndex,
                            NMEA_SENTENCE_COMMA);
              if (NumSatelliteThisSentence < 3)
                {
                  NMEAAddField (TransmitBuffer, &TXBufferIndex,
                                NMEA_SENTENCE_COMMA);
                }
            }
          Satellite++;
          /*SequenceNumber++; */
        }

      /**
       * Check Sum 
       **/
      NMEAAddCheckSum (TransmitBuffer, &TXBufferIndex);

      /**
       * CR/LF
       **/
      NMEAAddCRLF (TransmitBuffer, &TXBufferIndex);

      /**
       * Now send the sentence.
       **/
      if (!ComPortWrite ((unsigned char *) &TransmitBuffer[0], TXBufferIndex))
        {
          return NMEA_FAILED;
        }

      memset (TransmitBuffer, 0, MAX_NMEA_BUFFER);

    }
  return NMEA_SUCCESS;


}

/**
 *
 * RMC - NMEA has its own version of essential gps pvt (position,
 * velocity, time) data. It is called RMC, The Recommended Minimum,
 * which will look similar to:
 *
 * $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
 *
 * Where:
 *     RMC          Recommended Minimum sentence C
 *     123519       Fix taken at 12:35:19 UTC
 *     A            Status A=active or V=Void.
 *     4807.038,N   Latitude 48 deg 07.038' N
 *     01131.000,E  Longitude 11 deg 31.000' E
 *     022.4        Speed over the ground in knots
 *     084.4        Track angle in degrees True
 *     230394       Date - 23rd of March 1994
 *     003.1,W      Magnetic Variation
 *     *6A          The checksum data, always begins with *
 *
 *
 **/



NMEA_STATUS
NMEAGetGPVTGData (pNMEA_VTG_DATA pNmeaVTGData)
{

  pNmeaVTGData->TrueHeading = GetTrueHeading ();
  pNmeaVTGData->MagHeading = 180.32;
  pNmeaVTGData->GroundSpeedKnots = GetGroundSpeedKnots ();
  pNmeaVTGData->GroundSpeedKilos = GroundSpeedKilos ();

  return NMEA_SUCCESS;
}

/**
 * VTG - Velocity made good. The gps receiver may use the LC prefix
 * instead of GP if it is emulating Loran output.
 *
 *  $GPVTG,054.7,T,034.4,M,005.5,N,010.2,K
 *
 * where:
 *        VTG          Track made good and ground speed
 *        054.7,T      True track made good
 *        034.4,M      Magnetic track made good
 *        005.5,N      Ground speed, knots
 *        010.2,K      Ground speed, Kilometers per hour
 *
 * Note that, as of the 2.3 release of NMEA, there is a new field in
 * the VTG sentence at the end just prior to the checksum. For more
 * information on this field see here.
 *
 * Receivers that don't have a magnetic deviation (variation) table
 * built in will null out the Magnetic track made good.
 *
 * 
 *
 * $GPVTG
 * Track Made Good and Ground Speed. 
 *
 * eg1. $GPVTG,360.0,T,348.7,M,000.0,N,000.0,K*43
 * eg2. $GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*41
 *
 *           054.7,T      True course made good over ground, degrees
 *           034.4,M      Magnetic course made good over ground, degrees
 *           005.5,N      Ground speed, N=Knots
 *           010.2,K      Ground speed, K=Kilometers per hour
 *
 * eg3. for NMEA 0183 version 3.00 active the Mode indicator field
 *      is added at the end
 *      $GPVTG,054.7,T,034.4,M,005.5,N,010.2,K,A*53
 *           A            Mode indicator (A=Autonomous, D=Differential,
 *                        E=Estimated, N=Data not valid)
 *
 *
 *
 **/

NMEA_STATUS
NMEASendGPVTG (void)
{

  NMEA_VTG_DATA NmeaVTGData;
  char pFieldBuffer[MAX_NMEA_FIELD];
  int TXBufferIndex = 0;
  char TransmitBuffer[MAX_NMEA_BUFFER];


  memset (TransmitBuffer, 0, MAX_NMEA_BUFFER);
  memset (&NmeaVTGData, 0, sizeof (NMEA_VTG_DATA));

  /**
   * Get the necessary data from the FW.
   **/
  NMEAGetGPVTGData (&NmeaVTGData);

  TXBufferIndex = 0;

  /**
   * Add the header
   **/
  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_START);
  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_GPVTG);
  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA);

  /**
   * True heading
   **/
  ftoa (NmeaVTGData.TrueHeading, pFieldBuffer);
  NMEAAddField (TransmitBuffer, &TXBufferIndex, pFieldBuffer);
  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA);
  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_TRUE_HEADING);
  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA);

  /**
   * Mag heading
   **/
  ftoa (NmeaVTGData.MagHeading, pFieldBuffer);
  /*  NMEAAddField( TransmitBuffer, &TXBufferIndex, pFieldBuffer ); */
  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA);
  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_MAG_HEADING);
  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA);


  /**
   * Ground speed knots
   **/
  ftoa (NmeaVTGData.GroundSpeedKnots, pFieldBuffer);
  NMEAAddField (TransmitBuffer, &TXBufferIndex, pFieldBuffer);
  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA);
  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_KNOTS);
  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA);

  /**
   * Ground speed kilos
   **/
  ftoa (NmeaVTGData.GroundSpeedKilos, pFieldBuffer);
  NMEAAddField (TransmitBuffer, &TXBufferIndex, pFieldBuffer);
  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA);
  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_KILOS);
  /* NMEAAddField( TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA ); */

  /**
   * Check Sum 
   **/
  NMEAAddCheckSum (TransmitBuffer, &TXBufferIndex);

  /**
   * CR/LF
   **/
  NMEAAddCRLF (TransmitBuffer, &TXBufferIndex);

  /**
   * Now send the sentence.
   **/
  if (!ComPortWrite ((unsigned char *) &TransmitBuffer[0], TXBufferIndex))
    {
      return NMEA_FAILED;
    }

  return NMEA_SUCCESS;
}

NMEA_STATUS
NMEAGetGPRMCData (pNMEA_RMC_DATA pNmeaRMCData)
{
  DMY Dmy;
  unsigned long DateOfFix;
  /**
   * Need to convert from degrees to degrees and minutes format.
   **/
  pNmeaRMCData->TimeOfFixUTC = GetTimeOfFixUTC ();

  pNmeaRMCData->Status = NMEA_ACTIVE;   /* Cliff TODO */

  pNmeaRMCData->Lat =
    ConvertLatLonFromDegreestoDegreesMinutes (GetLatitudeAbs ());

  pNmeaRMCData->NorthSouth = GetNorthSouth ();

  pNmeaRMCData->Long =
    ConvertLatLonFromDegreestoDegreesMinutes (GetLongitudeAbs ());
  pNmeaRMCData->EastWest = GetEastWest ();


  pNmeaRMCData->GroundSpeedKnots = GetGroundSpeedKnots ();
  pNmeaRMCData->TrueHeading = GetTrueHeading ();
  GetDayMonthYear (&Dmy);

  DateOfFix =
    ((unsigned long) Dmy.Day * 10000) + ((unsigned long) Dmy.Month * 100) +
    ((unsigned long) Dmy.Year - 2000);
  pNmeaRMCData->DateOfFix = DateOfFix;

  return NMEA_SUCCESS;
}

/**
 * RMC - NMEA has its own version of essential gps pvt (position,
 * velocity, time) data. It is called RMC, The Recommended Minimum,
 * which will look similar to:
 *
 * $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
 *
 * Where:
 *     RMC          Recommended Minimum sentence C
 *     123519       Fix taken at 12:35:19 UTC
 *     A            Status A=active or V=Void. Not sure what is means??? TODO
 *     4807.038,N   Latitude 48 deg 07.038' N
 *     01131.000,E  Longitude 11 deg 31.000' E
 *     022.4        Speed over the ground in knots
 *     084.4        Track angle in degrees True
 *     
 *  TODO Why do they have a different DMY format from the one that is in ZDA?? 
 *  
 *  230394       Date - 23rd of March 1994
 *     003.1,W      Magnetic Variation
 *     *6A          The checksum data, always begins with *
 *
 * Note that, as of the 2.3 release of NMEA, there is a new field in
 * the RMC sentence at the end just prior to the checksum. For more
 * information on this field see here.
 *
 **/

NMEA_STATUS
NMEASendGPRMC (void)
{

  NMEA_RMC_DATA NmeaRMCData;
  char pFieldBuffer[MAX_NMEA_FIELD];
  int TXBufferIndex = 0;
  char TransmitBuffer[MAX_NMEA_BUFFER];


  memset (TransmitBuffer, 0, MAX_NMEA_BUFFER);
  memset (&NmeaRMCData, 0, sizeof (NMEA_RMC_DATA));

  /**
   * Get the necessary data from the FW.
   **/
  NMEAGetGPRMCData (&NmeaRMCData);

  TXBufferIndex = 0;

  /**
   * Add the header
   **/
  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_START);
  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_GPRMC);
  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA);


  /**
   * Field 0 Time   
   *
   * The time field must be 6 digits long. If the hour is less than 10 you 
   * must pad it with an extra zero.
   **/
  sprintf (pFieldBuffer, "%06ld.00", (long) NmeaRMCData.TimeOfFixUTC);
  /* ftod( NmeaRMCData.TimeOfFixUTC, pFieldBuffer  ); */
  NMEAAddField (TransmitBuffer, &TXBufferIndex, pFieldBuffer);
  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA);

  /**
   * Active or Void
   **/
  if (NmeaRMCData.Status == NMEA_ACTIVE)
    {
      NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_ACTIVE_STR);
      NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA);
    }
  else
    {
      NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_VOID_STR);
      NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA);
    }

  /**
   * Field 1 Latitude
   **/
  ftod4 (NmeaRMCData.Lat, pFieldBuffer);        /* 4 decimal places */
  NMEAAddField (TransmitBuffer, &TXBufferIndex, pFieldBuffer);
  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA);

  /**
   * Field 2 North/South 
   **/
  if (NmeaRMCData.NorthSouth == NMEA_SOUTH)
    {
      NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_SOUTH);
    }
  else
    {
      NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_NORTH);
    }

  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA);

  /**
   * Field 3 Longitude
   **/
  ftod4 (NmeaRMCData.Long, pFieldBuffer);       /* 4 decimal places */
  NMEAAddField (TransmitBuffer, &TXBufferIndex, pFieldBuffer);
  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA);


  /**
   * Field 4 East/West
   **/
  if (NmeaRMCData.EastWest == NMEA_WEST)
    {
      NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_WEST);
    }
  else
    {
      NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_EAST);
    }

  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA);



  /**
   * Speed over the ground in knots.
   **/
  ftodPrecision1 (NmeaRMCData.GroundSpeedKnots, pFieldBuffer);
  /*ftod( NmeaRMCData.GroundSpeedKnots, pFieldBuffer ); */
  NMEAAddField (TransmitBuffer, &TXBufferIndex, pFieldBuffer);
  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA);

  /**
   * True heading
   **/
  ftodPrecision1 (NmeaRMCData.TrueHeading, pFieldBuffer);
  /* ftod( NmeaRMCData.TrueHeading, pFieldBuffer ); */
  NMEAAddField (TransmitBuffer, &TXBufferIndex, pFieldBuffer);
  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA);

  /**
   * Date
   **/
  /* itoa (NmeaRMCData.DateOfFix, pFieldBuffer, 10); */
  sprintf (pFieldBuffer, "%ld", NmeaRMCData.DateOfFix);
  NMEAAddField (TransmitBuffer, &TXBufferIndex, pFieldBuffer);
  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA);

  /**
   * MagVar
   **/
  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA);



  /**
   * Check Sum 
   **/
  NMEAAddCheckSum (TransmitBuffer, &TXBufferIndex);

  /**
   * CR/LF
   **/
  NMEAAddCRLF (TransmitBuffer, &TXBufferIndex);

  /**
   * Now send the sentence.
   **/
  if (!ComPortWrite ((unsigned char *) &TransmitBuffer[0], TXBufferIndex))
    {
      return NMEA_FAILED;
    }

  return NMEA_SUCCESS;
}

NMEA_STATUS
NMEAGetGPZDAData (pNMEA_ZDA_DATA pNmeaZDAData)
{
  DMY Dmy;

  pNmeaZDAData->HrMinSecUTC = GetHrMinSecUTC ();
  GetDayMonthYear (&Dmy);

  pNmeaZDAData->Day = Dmy.Day;
  pNmeaZDAData->Month = Dmy.Month;
  pNmeaZDAData->Year = Dmy.Year;
  pNmeaZDAData->LocalTimeZoneHr = 8;    /* TODO */
  pNmeaZDAData->LocalTimeZoneMin = 19;

  return NMEA_SUCCESS;
}


/**
 * ZDA - Data and Time 
 *
 * $GPZDA,hhmmss.ss,dd,mm,yyyy,xx,yy*CC
 * $GPZDA,201530.00,04,07,2002,00,00*6E
 *
 * where:
 * hhmmss    HrMinSec(UTC)
 *  dd,mm,yyy Day,Month,Year
 *  xx        local zone hours -13..13
 *  yy        local zone minutes 0..59
 *  *CC       checksum
 *
 **/

NMEA_STATUS
NMEASendGPZDA (void)
{

  NMEA_ZDA_DATA NmeaZDAData;
  char pFieldBuffer[MAX_NMEA_FIELD];
  int TXBufferIndex = 0;
  char TransmitBuffer[MAX_NMEA_BUFFER];


  memset (TransmitBuffer, 0, MAX_NMEA_BUFFER);
  memset (&NmeaZDAData, 0, sizeof (NMEA_ZDA_DATA));

  /**
   * Get the necessary data from the FW.
   **/
  NMEAGetGPZDAData (&NmeaZDAData);

  TXBufferIndex = 0;

  /**
   * Add the header
   **/
  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_START);
  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_GPZDA);
  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA);

  /**
   * Time   
   *
   * The time field must be 6 digits long. If the hour is less than 10 you 
   * must pad it with an extra zero.
   **/
  sprintf (pFieldBuffer, "%06ld.00", (long) NmeaZDAData.HrMinSecUTC);
  /* ftod( NmeaZDAData.HrMinSecUTC, pFieldBuffer  ); */
  NMEAAddField (TransmitBuffer, &TXBufferIndex, pFieldBuffer);
  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA);

  /**
   * Day  
   **/
  sprintf (pFieldBuffer, "%02d", NmeaZDAData.Day);
  /*itoa( NmeaZDAData.Day, pFieldBuffer, 10  ); */
  NMEAAddField (TransmitBuffer, &TXBufferIndex, pFieldBuffer);
  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA);

  /**
   * Month   
   **/
  sprintf (pFieldBuffer, "%02d", NmeaZDAData.Month);
  /* itoa( NmeaZDAData.Month, pFieldBuffer, 10 ); */
  NMEAAddField (TransmitBuffer, &TXBufferIndex, pFieldBuffer);
  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA);

  /**
   * Year 
   **/
  sprintf (pFieldBuffer, "%d", NmeaZDAData.Year);
  NMEAAddField (TransmitBuffer, &TXBufferIndex, pFieldBuffer);
  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA);


  /**
   * Local time offset hour from UTC 
   **/
  sprintf (pFieldBuffer, "%02d", NmeaZDAData.LocalTimeZoneHr);
  /*itoa( NmeaZDAData.LocalTimeZoneHr, pFieldBuffer, 10  ); */
  NMEAAddField (TransmitBuffer, &TXBufferIndex, pFieldBuffer);
  NMEAAddField (TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA);


  /**
   * Local time offset minute from UTC  
   **/
  sprintf (pFieldBuffer, "%02d", NmeaZDAData.LocalTimeZoneMin);
  /* itoa( NmeaZDAData.LocalTimeZoneMin, pFieldBuffer, 10  ); */
  NMEAAddField (TransmitBuffer, &TXBufferIndex, pFieldBuffer);
  /* NMEAAddField( TransmitBuffer, &TXBufferIndex, NMEA_SENTENCE_COMMA ); */

  /**
   * Check Sum 
   **/
  NMEAAddCheckSum (TransmitBuffer, &TXBufferIndex);

  /**
   * CR/LF
   **/
  NMEAAddCRLF (TransmitBuffer, &TXBufferIndex);

  /**
   * Now send the sentence.
   **/
  if (!ComPortWrite ((unsigned char *) &TransmitBuffer[0], TXBufferIndex))
    {
      return NMEA_FAILED;
    }

  return NMEA_SUCCESS;
}

void
ftoaPrecision0 (float value, char *strFloat)
{
  sprintf (strFloat, "%.0f", value);
}


void
ftoa (float value, char *strFloat)
{
  sprintf (strFloat, "%.2f", value);
}

void
ftod (double value, char *strDouble)
{
  sprintf (strDouble, "%.2f", value);
}

void
ftodPrecision1 (double value, char *strDouble)
{
  sprintf (strDouble, "%.1f", value);
}

void
ftod4 (double value, char *strDouble)
{
  sprintf (strDouble, "%.4f", value);   /* added two more decimal places */
}
