#include <dos.h>   /* For Interrupt stuff */

/**
 * Interrupt specific stuff which is heavily dependant on the
 * toolchain used
 **/

#define IRQLEVEL        0       /* IRQ Line */

/**
 * PC/AT I/O Address constants
 **/
#define Interrupt_Controller_Master_8259 0x20
#define Interrupt_Controller_Slave_8259  0xA0
#define Programmable_Timer_8253          0x40
#define MC146818RTC                      0x70


#define END_OF_INTERRUPT 0x20

/**
 * IRQ 0 -  7  --> 0x08 - 0x0F
 * IRQ 8 - 15  --> 0x70 - 0x77
 **/
#define IRQ2VecVal(irq)    (irq < 8 ? irq + 8 : irq + 104)

#define get_8259(irq)  (irq < 8 ? Interrupt_Controller_Master_8259 \
                                : Interrupt_Controller_Slave_8259)


#if ((defined BCPP) || (defined __TURBOC__))

#define portIO_byte_out(addr, val)  outportb(addr, val)
#define portIO_byte_in(addr)        inportb(addr)
#define Interrupt_Enable()          enable()
#define Interrupt_Disable()         disable()
#define Get_Interrupt_Handler(irq)  getvect(IRQ2VecVal(irq))
#define Set_Interrupt_Handler(irq, handler) setvect(IRQ2VecVal(irq), handler)
#define clear_interrupt(irq)        portIO_byte_out (get_8259(irq), \
                                    END_OF_INTERRUPT)

#elif (defined VCPP)

#define portIO_byte_out(addr, val)  _outp(addr, val)
#define portIO_byte_in(addr)        _inp(addr)
#define Interrupt_Enable()          _enable()
#define Interrupt_Disable()         _disable()
#define Get_Interrupt_Handler(irq)  _dos_getvect(IRQ2VecVal(irq))
#define Set_Interrupt_Handler(irq, handler) \
                                    _dos_setvect(IRQ2VecVal(irq), handler)
#define clear_interrupt(irq)        portIO_byte_out (get_8259(irq), \
                                    END_OF_INTERRUPT)

#elif (defined __linux__)

#define portIO_byte_out(addr, val)  outb_p(val, addr)
#define portIO_byte_in(addr)        inb_p(addr)
#define Interrupt_Enable()          sti()
#define Interrupt_Disable()         cli()
#define Get_Interrupt_Handler(irq)              /* N/A */
#define Set_Interrupt_Handler(irq, handler)     /* N/A */
#define clear_interrupt(irq)                    /* N/A */

#endif

extern void gpsisr (void);

#if (defined VCPP)

void __interrupt _far
GPS_Interrupt (void)            /* MS */
{
  gpsisr ();
  
  /* reset the interrupt */
  clear_interrupt (IRQLEVEL);
}

static void (_interrupt _far * Old_Interrupt) ();      /* PGB MS */


#elif (defined __TURBOC__)

void interrupt
GPS_Interrupt (void)
{
  gpsisr ();
  
  /* reset the interrupt */
  clear_interrupt (IRQLEVEL);
}

static void interrupt (*Old_Interrupt) (void);

#elif (defined BCPP)

void interrupt
GPS_Interrupt (...)
{
  gpsisr ();
  
  /* reset the interrupt */
  clear_interrupt (IRQLEVEL);
}

static void interrupt (*Old_Interrupt) (...);     /* Old IRQ0 interrupt handler */

#elif (defined DJGPP)

#include <go32.h>
#include <dpmi.h>

void
GPS_Interrupt (int i __attribute__ ((unused)))
{
  asm ("cli; pusha");
  gpsisr ();
  
  /* reset the interrupt */
  clear_interrupt (IRQLEVEL);
}

static _go32_dpmi_seginfo Old_Interrupt;

#else /* (defined __linux__) */

void
GPS_Interrupt (int i __attribute__ ((unused)))
{
  gpsisr ();
}

#endif



#if ((defined BCPP) || (defined __TURBOC__) || (defined VCPP))

#if (IRQLEVEL == 0)

/**
 * Interface for 8253 Programmable Timer
 *
 * Inputs:
 * counter - which counter you wish to use 0 - 3
 * mode    - which mode to set up (see below)
 * divisor - value that divides the input frequency.
 *
 * The chip is feed with an input frequency of 1.19318 MHZ
 **/
static void
set_8253_Programmable_Timer (int counter, int mode, int divisor)
{
#define Control_Word_8253 4
  /**
   * The 8253 has four registers in the following order:
   * Counter #0, Counter #1, Counter #2, Control Word 
   *
   * Read Order: 
   *
   *   0 - Counter value is latched. This means that the selected
   *   counter has its contents transferred into a temporary latch,
   *   which can then be read by the CPU.
   *
   *   1 - Read / load least-significant byte only.
   *
   *   2 - Read / load most-significant byte only.
   *
   *   3 - Read / load least-significant byte first then
   *   most-significant byte second.
   *
   * Modes:
   *   0 - Interrupt on Terminal Count
   *   1 - Programmable One-Shot
   *   2 - Rate Generator
   *   3 - Square Wave Generator
   *   4 - Software Triggered Strobe
   *   5 - Hardware Triggered Strobe
   **/
  const int read_order = 3;
  const int bcd = 0;  /* We're not using binary coded decimal */

  portIO_byte_out (Programmable_Timer_8253 + Control_Word_8253, 
	      (counter << 6) | (read_order << 4) | (mode << 1) | bcd);
  portIO_byte_out (Programmable_Timer_8253 + counter, divisor & 0xff);
  portIO_byte_out (Programmable_Timer_8253 + counter, divisor >> 8);
}


#elif (IRQLEVEL == 8)

static unsigned char rtc_reg_orig, rtc_a_orig, rtc_b_orig;

/**
 * Set up Motorola 146818 Real Time Clock to generate interrupts on IRQ #8
 *
 * input: log2freq - Log base 2 of desired frequency in Hz
 **/
static void
set_rtc (int log2freq)
{
  int addr = MC146818RTC;
  int val;

  rtc_reg_orig = portIO_byte_in (addr);
  portIO_byte_out (addr, 10);            /* Set RTC for register A */
  rtc_a_orig = portIO_byte_in (addr + 1);
  val = rtc_a_orig;
  val |= (16 - log2freq);    /* e.g. 2^11 => 2048 interrupt / second */
  portIO_byte_out (addr + 1, val);
  
  portIO_byte_out (addr, 11);          /* Set RTC for register B (control) */
  rtc_b_orig = portIO_byte_in (addr + 1);
  val = rtc_b_orig;
  val |= 0x40;               /* Turn on RTC Periodic update */
  portIO_byte_out (addr + 1, val); 
}

static void
restore_rtc (void)
{
  int addr = MC146818RTC;

  portIO_byte_out (addr, 10);     /* Set RTC for register A */
  portIO_byte_out (addr + 1, rtc_a_orig);
  portIO_byte_out (addr, 11);     /* Set RTC for register B */
  portIO_byte_out (addr + 1, rtc_b_orig);
  portIO_byte_out (addr, rtc_reg_orig); /* Restore original register setting */

}

#endif /* IRQLEVEL */

/******************************************************************************
FUNCTION Interrupt_Install()
RETURNS  None.

PARAMETERS None.

PURPOSE
	This function replaces the current IRQ0 Interrupt service routine with
	our own custom function. The old vector is stored in a global variable
	and will be reinstalled at the end of program execution. IRQ0 is
	enabled by altering the interrupt mask stored by the 8259 interrupt
	handler.

******************************************************************************/
void
Interrupt_Install ()
{
  unsigned char int_mask;
  extern unsigned int interr_int;

  /* Address of appropriate 8259 Interrupt Controller */
  int addr_8259 = get_8259 (IRQLEVEL);
  int irq_bit = (IRQLEVEL < 8 ? IRQLEVEL : IRQLEVEL - 8);
 
  /* Save old interrupt handler so we can restore upon exit */
  Old_Interrupt = Get_Interrupt_Handler (IRQLEVEL);

  /* Must disable interrupts while we modify the handler */
  Interrupt_Disable ();
  Set_Interrupt_Handler (IRQLEVEL, GPS_Interrupt);

  /* Get current hardware interrupt mask (second register) */
  int_mask = portIO_byte_in (addr_8259 + 1);
  
  /* Clear the bit for our IRQ */
  int_mask &= ~(1 << irq_bit);

  /* send new mask to 8259 */
  portIO_byte_out (addr_8259 + 1, int_mask);

  /* Now safe to re-enable interrupts */
  Interrupt_Enable ();

  /* Set up different devices depending on which Interrupt is being used */
#if (IRQLEVEL == 0)
  /* Counter #0, Mode 2: Rate Generator */
  set_8253_Programmable_Timer (0, 2, interr_int);
#elif (IRQLEVEL == 8)
  set_rtc (11);  /* 2^11 --> 2048 Interrupts / second */
#else
#error No way defined to use IRQ IRQLEVEL
#endif

  /* Clear any pending interrupts */
  clear_interrupt (IRQLEVEL);

}

/******************************************************************************
FUNCTION Interrupt_Remove()
RETURNS  None.

PARAMETERS None.

PURPOSE
	This function removes the custom interrupt vector from the vector
	table and restores the previous vector.

******************************************************************************/
void
Interrupt_Remove ()
{
  unsigned char int_mask;

  /* Address of appropriate 8259 Interrupt Controller */
  int addr_8259 = get_8259 (IRQLEVEL);
  int irq_bit = (IRQLEVEL < 8 ? IRQLEVEL : IRQLEVEL - 8);

  /* Clear any pending interrupts */
  portIO_byte_out (addr_8259, END_OF_INTERRUPT);

  /* get hardware interrupt mask */
  int_mask = portIO_byte_in (addr_8259 + 1);

  /* Set the bit for our IRQ */
  int_mask |= (1 << irq_bit);

  /* Stop interrupts while we change out the interrupt handler */
  Interrupt_Disable ();

  /* send new mask to 8259 */
  /* portIO_byte_out (addr_8259 + 1, int_mask); */

  /* Restore original interrupt handler */
  Set_Interrupt_Handler (IRQLEVEL, Old_Interrupt);

  /* allow hardware interrupts */
  Interrupt_Enable ();

  /* clear interrupt and allow next one */
  portIO_byte_out (addr_8259, END_OF_INTERRUPT);

#if (IRQLEVEL == 0)
  /* reset clock */
  set_8253_Programmable_Timer (0, 2, 0xffff);
#else
  /* Restore original RTC settings */
  restore_rtc ();
#endif
}


#elif (defined DJGPP)           /* DJGPP compiler */

/* This code for DJGPP has never been shown to work properly */

void
Interrupt_Install ()
{
  int int_handler;

  GPS_Interrupt.pm_offset = int_handler;
  GPS_Interrupt.pm_selector = _go32_my_cs ();
  _go32_dpmi_get_protected_mode_interrupt_vector (0x08, &Old_Interrupt);
  _go32_dpmi_allocate_iret_wrapper (&GPS_Interrupt);
  _go32_dpmi_set_protected_mode_interrupt_vector (0x08, &GPS_Interrupt);

  /* Counter #0, Mode 2: Rate Generator */
  set_8253_Programmable_Timer (0, 2, interr_int);

}

void
Interrupt_Remove ()
{
  _go32_dpmi_set_protected_mode_interrupt_vector (0x08, &Old_Interrupt);
  _go32_dpmi_free_iret_wrapper (&GPS_Interrupt);

  /* reset clock */
  set_8253_Programmable_Timer (0, 2, 0xffff);
}

#endif /* Compiler */
