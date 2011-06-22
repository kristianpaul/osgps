#include "types.h"

/*
 *  This is the primary data structure that provides access
 *  to the components of each channel
 */
#ifndef STRUCTS_H
#define STRUCTS_H

struct solution_channel
{
  int word_error[5];
  int page5;
};

struct satellite
{
  double Pr, dPr, Tropo, Iono;
};

struct hms
{
  int deg, min;
  float sec;
};
typedef struct hms hms;

struct almanac                  /* Approximate orbital parameters */
{
  float w, ety, inc, rra, sqa, lan, aop, ma, toa, af0, af1;
  char text_message[23];
  int health, week, sat_file;
};
typedef struct almanac almanac;

struct ephemeris                /* Precise orbital parameters */
{
  int iode, iodc, ura, valid, health, week;
  double dn, tgd, toe, toc, omegadot, idot, cuc, cus, crc, crs, cic, cis;
  double ma, e, sqra, w0, inc0, w, wm, ety, af0, af1, af2;
};
typedef struct ephemeris ephemeris;

struct satvis
{
  float azimuth, elevation, doppler;
  float x, y, z;
};
typedef struct satvis satvis;

struct ecef
{
  double x, y, z;
};
typedef struct ecef ecef;

struct eceft
{
  double x, y, z, tb;
  float az, el;
};
typedef struct eceft eceft;

struct llh
{
  double lat, lon, hae;
};
typedef struct llh llh;

struct pvt
{
  double x, y, z, dt;
  double xv, yv, zv, df;
};
typedef struct pvt pvt;

struct velocity
{
  double east, north, up;
  double clock_err;
  ecef x, y, z;
};
typedef struct velocity velocity;

struct state
{
  velocity vel;
  eceft pos;
  llh loc;
  ecef north, east, up;
};
typedef struct state state;

#endif
