#ifndef SER_PORT_H
#define SER_PORT_H
/***********************************************************************
// Clifford Kelley <cwkelley@earthlink.net>
// This program is licensed under BSD LICENSE
    Version 0.4 Added buad rate selection and NMEA sentence selection capablity
	Version 0.3 Turned off hardware flow control
	Version 0.2 Fixed SV in view bug
	Version 0.1 Initial release
***********************************************************************/
/*void delay(int d);*/
void open_com (int Cport, unsigned baud, int parity, int stopbits,
               int numbits, int *err_code);
void close_com ();
void reset_buffer ();
void send_com (char c, int *err_code);
int ComPortWrite (unsigned char *str, int err_code);
int readln_com (char *str, int *err_code);

#endif
