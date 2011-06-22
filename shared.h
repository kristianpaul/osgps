/* variables shared with SoftOSGPS, set here for hardware, overwritten for software */

int d_freq = 4698;

/*long TIC_ref = 571427L;*/

long carrier_ref, code_ref = 0x016ea4a8L;

float Bnpullcarr,Bnpullcode;
float Bntrkcarr,Bntrkcode;

int trk_carr_C1 = 241, trk_carr_C2 = 8;
int trk_code_C1 =  61, trk_code_C2 = 2;
int pull_carr_C1 = 241,  pull_carr_C2 = 8;
int pull_code_C1 = 1515, pull_code_C2 = 51;
