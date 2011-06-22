#ifndef _FILE_H
#define _FILE_H

#include <stdio.h>

extern FILE *output, *debug, *in, *out, *kalm, *data_bits, *rinex_obs, *rinex_nav;
extern char  output_file[40],almanac_file[40],ephemeris_file[40];
extern char  receiver_file[40],location_file[40],ion_utc_file[40];
extern char  kalman_file[40],rinex_obs_file[40],rinex_nav_file[40];
extern char  debug_file[40],data_bits_file[40];
#endif /* _FILE_H */
