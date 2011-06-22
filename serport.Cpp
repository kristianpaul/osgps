/***********************************************************************
// Clifford Kelley <cwkelley@earthlink.net>
// This program is licensed under GNU GENERAL PUBLIC LICENSE
    Version 0.4 Added buad rate selection and NMEA sentence selection capablity
	Version 0.3 Turned off hardware flow control
	Version 0.2 Fixed SV in view bug
	Version 0.1 Initial release
***********************************************************************/
#include <bios.h> 
#include  <stdio.h>
#include  <stdlib.h>
#include  <conio.h>
#include  <io.h>
#include <dos.h>
#include <string.h>

#ifdef DJGPP
#include <pc.h>
#endif


#define timeout     10000 /* Readln_com times out at 10000 milliseconds*/
//#define timeout     10000 /* Readln_com times out at 10000 milliseconds*/
#define max_buffer  1000  /* Circular buffer size */
#define near_full   900   /* When buffer_length exceeds near_full */
                          /* RTS line is disabled */
#define near_empty  100   /* When buffer drops below near_empty */
                          /* RTS line is enabled */
int  com_flag = 0;        /*Com open flag*/

int  overrun_flag;
char com_buffer[ max_buffer ];  /*Circular com buffer*/
int  com_port;                /* Current Com port (0 or 1) */
int  intlev;                  /* Hardware interrut level for com port */
int  buffer_in;               /* Pointer for input to com buffer */
int  buffer_out;              /* Pointer for output from com buffer */
int  buffer_length;           /* Current number of characters in com buffer */
int  bf_hndshk;               /* Handshake flag for control lines */

int  thr, rbr, ier, lcr,
     mcr, lsr, msr;           /*Async com board registers*/

void far *oldfunc;  /* Holds old interrupt vector */

void delay(int d)
/* d = delay in milliseconds on 10 MHz AT */
{ int i;
  int j;
  int k;
  j = 0;
  for (k=0; k < d; k++)
    for (i=1; i < 200; i++ )
    j += 1;
}

#ifdef VCPP
void interrupt far com_isr()
#endif
#ifdef BCPP
void interrupt com_isr(...)
#endif
#ifdef DJGPP
void interrupt com_isr(void)
#endif
{
  if ( com_flag == 1 ) 
  {
    /*Get character - store in circular buffer*/
    com_buffer[ buffer_in ] = inp( rbr );

    /*Increment buffer_in pointer*/
    buffer_in += 1;

    /*Wrap buffer pointer around to start if > max_buffer*/
    if (buffer_in == max_buffer)  buffer_in = 0;

    /*Current number of characters in buffer incremented 1*/
    buffer_length +=  1;
    if (buffer_length > max_buffer)
	{
      buffer_length = max_buffer;
      overrun_flag = 1;
    }
    /*Disable RTS if buffer_length exceeds near_full constant*/
    if (buffer_length > near_full) {
      outp(mcr, 9);          /*Disable rts , leave dtr and out2 set*/
      bf_hndshk = 1;         /*Buffer full handshake = true*/
    }
  }
  outp( 0x20,0x20 );   /* End of Interrupt to 8259 */
}

void reset_buffer()
/* This procedure will reset the com buffer */
{

  bf_hndshk = 0;     /*Buffer full handshake false*/
  buffer_in = 0;     /*Set circular buffer input to 0*/
  buffer_out = 0;    /*Set circular buffer output to 0*/
  buffer_length = 0; /*Set buffer_length to 0*/
  overrun_flag = 0;  /*Set overrun flag to false */
}


void open_com(int Cport,    /*Com port # - 0 or 1          */
              unsigned int baud,     /*baud rate - 110,150,300..9600*/
              int parity,   /*parity 0 = no parity         */
                            /*       1 = even parity       */
                            /*       2 = odd parity        */
              int stopbits, /*stop bits - 1 or 2           */
              int numbits,  /*word length - 7 or 8         */
              int *error_code )
{
   int comdata;
   int ptemp;

        *error_code = 0;
        com_port = Cport;
        comdata = 0;
        if ((numbits == 7) || (numbits == 8))
                comdata = comdata | (numbits-5) ;
        else *error_code = 5;

        if ((stopbits == 2) || (stopbits == 1))
                comdata = comdata | ((stopbits-1) << 2) ;
        else *error_code = 4;

        if ((parity == 1) || (parity == 3) || (parity == 0))
                comdata = comdata | (parity << 3) ;
        else *error_code = 3;

       switch (baud)
	   {
                case 110: comdata = comdata | 0x00;
                        break;
                case 150: comdata = comdata | 0x20;
                        break;
                case 300: comdata = comdata | 0x40;
                        break;
                case 600: comdata = comdata | 0x60;
                        break;
                case 1200: comdata = comdata | 0x80;
                        break;
                case 2400: comdata = comdata | 0xA0;
                        break;
                case 4800: comdata = comdata | 0xC0;
                        break;
                case 9600 : comdata = comdata | 0xE0;
				        break;

				case 38400L:
				{
					goto HighSpeedLinkSetting;
					break;
				}
				
				default : *error_code = 2;
                        break;
     }

	if ((Cport < 0) || (Cport > 1 ) )
	{
		*error_code = 1;
	}

	if ( *error_code == 0 )
	{
		_bios_serialcom( 0 , Cport, comdata );
	}
	//
	// Need this kludge because _bios_serialcom does
	// not support any baud rate over 9600.
	//
HighSpeedLinkSetting:

	if( baud == 38400L )
	{
		//
		// Setup for baud rate programming
		//
		outp( 0x3fb, 0x80 );
		// Baud rate 38,400 
		outp( 0x3f8, 0x03 );
		outp( 0x3f9, 0x00 );

		outp( 0x3fb, 0x00 );
		//
		// Program the Data Format reg
		//
		outp( 0x3fb, 0x07 ); // 8-N-1
	}

     if (Cport == 0)
	 {
       thr = 0x3f8; // Tx Register			  0
       rbr = 0x3f8; // Rx Register			  0
       ier = 0x3f9; // Interrupt enable	  1
				        // 0x3fa Interrupt ID	  2
	    lcr = 0x3fb; // Data format			  3
       mcr = 0x3fc; // Modem control		  4
       lsr = 0x3fd; // Serialization status 5
       msr = 0x3fe; // Modem status			  6
      } 
	 else 
	 {
       thr = 0x2f8;                    /*Set register variables*/
       rbr = 0x2f8;                    /*for port locations of*/
       ier = 0x2f9;                    /*serial com port #2*/
       lcr = 0x2fb;
       mcr = 0x2fc;
       lsr = 0x2fd;
       msr = 0x2fe;
      }

      intlev = 0xC - Cport;
#ifdef VCPP
      oldfunc = _dos_getvect(intlev);
      _dos_setvect(intlev, com_isr);
      _disable();                       /*No interrupts*/
#endif
#ifdef BCPP
      oldfunc = getvect(intlev);
      setvect(intlev,com_isr);
      disable();
#endif
	  ptemp = inp( lcr ) & 0x7f;
      
	  outp(lcr,ptemp);
      
	  ptemp = inp(lsr);            /*Reset any pending errors*/
      
	  ptemp = inp(rbr);            /*Read any pending character*/

      if (Cport == 0) 
	  {                /*Set irq on 8259 controller*/
        ptemp = inp(0x21) & 0xef;
        outp(0x21,ptemp);
      }
      else
	  {
        ptemp = inp(0x21) & 0xf7;
        outp(0x21,ptemp);
      }

      outp(ier,1);       /*Enable data ready interrupt*/
      ptemp = inp(mcr) | 0xb;
      outp(mcr,ptemp);
      _enable();         /*Turn on interrupts*/
      *error_code = 0;     /*Set error code to 0*/
      com_flag = 1;      /*Com inititalization flag true*/
      reset_buffer();
}

void close_com()
/* This procedure disables the com port interrupt. */
{  int ptemp;
  if (com_flag==1) 
  {
    _disable();                   /*No interrupts*/
    ptemp = inp(0x21) | 0x18;
    outp(0x21,ptemp);             /*Set mask register to turn off interrupt*/
    ptemp = inp(lcr) | 0x7f;
    outp(lcr,ptemp);              /*Turn off 8250 data ready interrupt*/
    outp(ier,0);
    outp(mcr,0);                  /*Disable out2 on 8250*/
  // PGB  _dos_setvect( intlev, oldfunc ); /* return to old interrupt vector */
    _enable();                     /*Turn on interrupts*/
    com_flag = 0;
  }
}

/*This procedure returns 1 character from the com_buffer, at the array element*/
/*pointed to by the circular buffer pointer buffer_out.                       */
/*   0 = no error                */
/*   6 = no character available  */
/*   7 = buffer overflow         */
/*  10 = com port not initialized */
void check_com( char* c, int *error_code)  /*error code for check_com       */
{


	if ( com_flag == 0 ) 
	{
		/*Make sure com port has been initialized*/
		*error_code = 10;
	}
	else
	{
		if ( buffer_length == 0 ) 
		{
			/*Check to see if any characters in buffer*/
			*error_code = 6;
		}
		else 
		{
			if ( overrun_flag == 1 ) 
			{
				/*buffer overflow */
				*error_code = 7;
			}
			else 
			{
				*error_code = 0;
			}

			*c = ( com_buffer[ buffer_out ] );   /*Get charater out of buffer*/
			buffer_out += 1;                     /*Increment buffer_out_pointer*/
									             /*Wrap buffer_out pointer around if > */
									             /*max_buffer*/
			if ( buffer_out == max_buffer )
			{
				buffer_out = 0;
			}

			buffer_length -= 1; /*Decrement buffer_length*/

			/*Enable RTS if buffer_length < near_empty*/
			if ( bf_hndshk && ( buffer_length < near_empty ) ) 
			{
				outp( mcr,0xb );
				bf_hndshk = 0;
			}
		}
	}
}


void send_com(char c,        /*Character to send out com port*/
              int *error_code)   /*Error code for send_com       */
                                         /*  0 = no error                */
                                         /*  8 = time out error          */
                                         /* 10 = com port not initialized*/
/*This procedure sends a character out the com port.                     */

{
	int handshake;
	int counter;

	if ( com_flag == 0 )
	{
		/*Make sure com port has been initialized*/
		*error_code = 10;
	}
	else
	{
		counter   = 0;       /* Initialize time out counter          */
		handshake = 0; //    0x30;    /* Use the following handshake values:  */
							 /*          0x0 no handshake            */
							 /*          0x10 CTS handshaking         */
							 /*          0x20 DSR handshaking         */
					         /*          0x30 CTS and DSR handshaking */
		do
		{
			counter += 1;
		//	delay( 1 );          /*delay 1 millisecond - causes timeout at 10 seconds*/
		}while ( ( ( ( inp( msr ) & handshake ) != handshake ) ||  /*Check handshake*/
			  ( ( inp( lsr ) & 0x20) != 0x20 ) ) &&               /*Check that transmit reg empty*/
			  ( counter < timeout ) ); 

		
		   


		/* Give up after 10 seconds */
		if ( counter == timeout )
		{
			*error_code = 8;
		}
		else
		{
			_disable();         /*No interrpts*/
			outp( thr, c );     /*Transmit character*/
			_enable();          /*Interrupts on*/
			*error_code = 0;
		}
	}
}


int ComPortWrite( unsigned char* str,       /*string to send out com port*/
                   int           NumberOfBytes ) /*error code for writeln_com*/
{  
	
	int error_code;
	int i;


	for ( i = 0; i < NumberOfBytes; i++)
	{
	  send_com( str[ i ], &error_code );
	}
	
	return 1; 

}


int readln_com( char* str,               /*string to received from com port*/
                int*  error_code)       /*error code for writeln_com*/
{  int i=0;
   char c;
   int counter = 0;
   do 
   {
     check_com( &c, error_code );

     if ( *error_code == 0 ) 
	 {
       str[ i ] = c;
       i += 1;
     } 
	 else 
	 {
       delay(1);
       counter += 1;
     }
   } while ( ( i < 255 ) && ( c != 13) && ( counter < timeout ) );

   if (counter == timeout)
   {
	   *error_code = 8;
   }

   str[ i ] = 0;

   return i;
}
