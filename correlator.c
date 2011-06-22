#include <string.h> /* For memset() */
#include <math.h>   /* For sqrt() */
#include "consts.h"
#include "globals.h"
#include "correlator.h"
#include "SoftOSGPS.h"
#include "interfac.h"

static char prn_prompt[33][2046],prn_late[33][2046];
static long tic_ref, tic=0;
static int   ms_counter[N_channels],bit_counter[N_channels];

static void 
generate_prn_codes(void)
{
  int i,j,G1,G2,prn,chip, half_chip;
  int G2_i[33] = {
    0x000, 0x3f6, 0x3ec, 0x3d8, 0x3b0, 0x04b, 0x096, 0x2cb, 0x196,
    0x32c, 0x3ba, 0x374, 0x1d0, 0x3a0, 0x340, 0x280, 0x100,
    0x113, 0x226, 0x04c, 0x098, 0x130, 0x260, 0x267, 0x338,
    0x270, 0x0e0, 0x1c0, 0x380, 0x22b, 0x056, 0x0ac, 0x158};
  
  
  for (prn=1;prn<33;prn++)
    {
      char prn_code[1023];
      prn_code[0]=1;
      G1 = 0x1FF;
      G2 = G2_i[prn];
      for (chip=1;chip<1023;chip++)
	{
	  prn_code[chip]=(G1^G2) & 0x1;  /* exor the right hand most bit  */
	  i=((G1<<2)^(G1<<9)) & 0x200;
	  G1=(G1>>1) | i;
	  j=((G2<<1)^(G2<<2)^(G2<<5)^(G2<<7)^(G2<<8)^(G2<<9)) & 0x200;
	  G2=(G2>>1) | j;
	}
      for (half_chip=0;half_chip<2046;half_chip++)
	{
	  prn_prompt[prn][half_chip]=2*prn_code[((half_chip+1)%2046)>>1]-1;
	  prn_late[prn][half_chip]=2*prn_code[half_chip>>1]-1;
	}
    }
}


#define SATCNTL               0
#define CARRIER_DCO_INCR_HIGH 3
#define CARRIER_DCO_INCR_LOW  4
#define CODE_DCO_INCR_HIGH    5
#define CODE_DCO_INCR_LOW     6

/* Code select is stored in bits 13 and 14 of the SATCNTL register */
enum {CODE_SELECT_EARLY, CODE_SELECT_LATE, CODE_SELECT_DITHER, 
      CODE_SELECT_EARLY_MINUS_LATE};

struct gp2021_channel {
  uint32_t int_carrier_phase;
  uint32_t int_carrier_cycle;
  uint32_t int_code_phase;
  uint16_t int_code_half_chip;
  int32_t i_prompt_accum;
  int32_t q_prompt_accum;
  int32_t i_tracking_accum;
  int32_t q_tracking_accum;
} gpchan[N_channels];



void
correlator_init (double samp_rate, double tic_period)
{
  tic_ref = samp_rate * tic_period;
  tic = tic_ref;
  memset (gpchan, 0, sizeof(struct gp2021_channel) * N_channels);
  generate_prn_codes();
}

void
Sim_GP2021_int (char *IF, long nsamp)
{
  int ch;
  uint32_t carrier_rollover, code_rollover;
#if 1
#endif
  int Accum_status_A,tic_count;


  if (tic < nsamp)           /* will a tic occur during this sample set? */
    {
      /* yes, tic count is sample number where tic occurs */
      tic_count = tic;       

      /* reset tic for next time    */
      tic += tic_ref - nsamp;
    }
  else  
    {
      tic-=nsamp;            /* no, reduce tic by nsamples  */
      tic_count=-1;           /* -1 means no tic occurs in this sample set */
    }
	  

  Accum_status_A = 0;

  for (ch=0; ch < N_channels; ch++) 
    {

      int reg = ch<<3;
      int slew_dump=REG_write[(ch<<2)+0x84]+2046;

      /* implement epoch set */
      if (REG_write[reg+7] != -1) 
	{
	  REG_read[reg+7] = REG_write[reg+7];
	  ms_counter[ch]=REG_write[reg+7] & 0xff;    /* set ms counter   */
	  bit_counter[ch]=REG_write[reg+7]>>8;       /* set bit counter  */
	  REG_write[reg+7]=-1;                       /* reset epoch set  */
	}

      /* Don't bother continuing if this channel is idle i.e. no PRN
	 assigned */
      if (ichan[ch].prn>0)
	{

	  uint32_t carrier_dco_incr = 
	    (REG_write[reg + CARRIER_DCO_INCR_HIGH]<<16)
	    + REG_write[reg + CARRIER_DCO_INCR_LOW];
	  uint32_t code_dco_incr    = (REG_write[reg + CODE_DCO_INCR_HIGH]<<16)
	    + REG_write[reg + CODE_DCO_INCR_LOW];
	  char *this_prn_prompt = prn_prompt[ichan[ch].prn];
	  char *this_prn_late   = prn_late[ichan[ch].prn];
	  struct gp2021_channel *this_gpchan = &gpchan[ch];
	  char *ifptr = IF;
	  int prompt_ca_bit = 
	    this_prn_prompt[this_gpchan->int_code_half_chip];
	  int track_ca_bit = 
	    this_prn_late[this_gpchan->int_code_half_chip];
	  int i;

	  for (i=0;i<nsamp;i++)   /* correlate each data sample  */
	    {
	      /* We use "int" type here instead of "char" ONLY because
		 it's faster */
	      static const int i_lo_seq[8] = {-1, 1, 2, 2, 1,-1,-2,-2};
	      static const int q_lo_seq[8] = { 2, 2, 1,-1,-2,-2,-1, 1};

	      int tif = *ifptr++;
	      int idx = this_gpchan->int_carrier_phase >> 29;

	      /* First (carrier) mixer */
	      int ival = tif * i_lo_seq[idx];
	      int qval = tif * q_lo_seq[idx];

	      /* Second (code) mixer */
	      this_gpchan->q_tracking_accum += track_ca_bit  * qval;
	      this_gpchan->q_prompt_accum   += prompt_ca_bit * qval;
	      this_gpchan->i_tracking_accum += track_ca_bit  * ival;
	      this_gpchan->i_prompt_accum   += prompt_ca_bit * ival;

	      /* Carrier is designed to wrap carrier around at 32-bits */
	      
	       carrier_rollover = this_gpchan->int_carrier_phase;
	       this_gpchan->int_carrier_phase += carrier_dco_incr;
	       if (this_gpchan->int_carrier_phase < carrier_rollover) 
		  this_gpchan->int_carrier_cycle++;
	      

	      /* Code is designed to wrap code around at 32-bits */
	      
		code_rollover =  this_gpchan->int_code_phase;
		this_gpchan->int_code_phase  += (code_dco_incr << 1);
		if (this_gpchan->int_code_phase < code_rollover)
		  this_gpchan->int_code_half_chip++;

		prompt_ca_bit = 
		    this_prn_prompt[this_gpchan->int_code_half_chip];
		track_ca_bit = 
		    this_prn_late[this_gpchan->int_code_half_chip];

		if  (this_gpchan->int_code_half_chip>=slew_dump) /*  dump  */
		{
		      /**
		       * dump correlators into the appropriate registers
		       **/
		      reg=(ch<<2)+0x84;
		      REG_read[reg  ] = this_gpchan->i_tracking_accum;
		      REG_read[reg+1] = this_gpchan->q_tracking_accum;
		      REG_read[reg+2] = this_gpchan->i_prompt_accum;
		      REG_read[reg+3] = this_gpchan->q_prompt_accum;

		      /* reset slew to 0  */
		      REG_write[reg] = 0;

		      /* reset the correlators */
		      this_gpchan->i_tracking_accum  = 
			this_gpchan->q_tracking_accum =
			this_gpchan->i_prompt_accum   = 
			this_gpchan->q_prompt_accum   = 0;
		      this_gpchan->int_code_half_chip = 0;

		      /* set the bit if a dump occurs */
		      Accum_status_A = Accum_status_A | (1<<ch);
           
		      /*  increment ms and bit counters  */
		      ms_counter[ch] ++;      /* ms counter  */          
		      if (ms_counter[ch]==20) 
			/* bit counter */
			bit_counter[ch] = (++bit_counter[ch])%50;
		      ms_counter[ch] = ms_counter[ch] % 20;
		      REG_read[ch*8+7] = ms_counter[ch] + (bit_counter[ch]<<8);
		}
	      
	      /* at TIC save the carrier and code info for measurements */
	        if (i==tic_count)
		{ 
		  reg=ch<<3;

		  /* ms and bit counters          */
		  REG_read[reg+4] = REG_read[reg+7];

		  /* carrier phase (top 10 bits)  */
		  REG_read[reg+3] = this_gpchan->int_carrier_phase >> 22;

		  /* number of half chips         */
		  REG_read[reg+1] = this_gpchan->int_code_half_chip;

		  /* half chip phase (top 10 bits)*/
		  REG_read[reg+5] = this_gpchan->int_code_phase >> 22;

		  /* carrier cycle low            */
		  REG_read[reg+2] = this_gpchan->int_carrier_cycle & 0xffff;

		  /* carrier cycle high           */
		  REG_read[reg+6] = this_gpchan->int_carrier_cycle >> 16;

		  this_gpchan->int_carrier_cycle=0;
		}
	    }
	}
    }
  /* load the register for accum status A bits   */
  REG_read[0x82]=Accum_status_A;

 /* if a tic occurs set the accum status B bit  */
  if (tic_count > -1) 
    REG_read[0x83]=0x2000;
  else 
    REG_read[0x83]=0x0;
}

