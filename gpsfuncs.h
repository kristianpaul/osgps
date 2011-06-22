
/**
 * Function definitions
 **/
long i_sqrt (long);
satvis satfind (int);
void read_initial_data (void);
void write_almanac (void);
void read_ephemeris (void);
void write_ephemeris (void);
void write_Debug_ephemeris (int);
void read_ion_utc (void);
void write_ion_utc (void);
eceft satpos_ephemeris (double, struct ephemeris *);
llh receiver_loc (void);
llh ecef_to_llh (ecef);
ecef llh_to_ecef (llh);
double tropo_iono (int, float, float, double);
void navmess (int, int);
pvt pos_vel_time (int);
pvt pos_vel_time_a (int);
void dops (int);
/*pvt Kalman_filter(int);*/
