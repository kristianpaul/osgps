#ifndef _PCI_H
#define _PCI_H



#include "types.h"

#define TRUE  1
#define FALSE 0

#ifdef __cplusplus
extern "C"
{
#endif

#if ((defined __DPMI32__) || (defined __linux__))
#   define far
#   define _cdecl
#endif

  int far _cdecl pci_is_bios_present (void);

/* 33 MHz for 166 MHz CPU, 30 MHz for 120 MHz CPU */
#define PCI_CLOCK_RATE               33E6
#define _82437FX                FALSE
#define _82439HX                TRUE





#define RETCODE_NO_MORE_PCI_DEVICES  0xFFFF

  /**
   * structure of device ID (a.k.a handle)
   *      bits 15:8       bus number
   *            7:3       device number
   *            2:0       function number (subdevice)
   **/
#define PCI_BUSSES_PER_MACHINE       16
#define PCI_DEVICES_PER_BUS          32
#define PCI_FUNCTIONS_PER_DEVICE     8

  uint16_t far _cdecl pci_get_device_handle (uint16_t device_id,
					     uint16_t vendor_id, 
					     uint16_t index);


  /**
   * functions used by macros below
   **/

  uint32_t far _cdecl pci_read_config_32 (uint16_t device_id, uint16_t offset);
  uint16_t far _cdecl pci_read_config_16 (uint16_t device_id, uint16_t offset);
  uint8_t far _cdecl pci_read_config_8 (uint16_t device_id, uint16_t offset);
  void far _cdecl pci_write_config_32 (uint16_t device_id, uint16_t offset,
                                       uint32_t value);
  void far _cdecl pci_write_config_16 (uint16_t device_id, uint16_t offset,
                                       uint16_t value);
  void far _cdecl pci_write_config_8 (uint16_t device_id, uint16_t offset,
                                      uint8_t value);



  /**
   * specific uses of PCI configuration space that work on all PCI devices
   *
   *
   * mask off the control bits
   *
   * bit 0 = 0       use memory space
   * bit 0 = 1       use I/O space (bits 3:1 reserved 0)
   *
   * bits 2:1 = 00   locate anywhere in 32-bit address space
   * bits 2:1 = 01   locate below 1MB 
   * bits 2:1 = 10   locate anywhere in 64-bit address space
   * bits 2:1 = 11   reserved
   *
   * bit 3 = 0       not prefetchable
   * bit 3 = 1       prefetchable
   **/
#define pci_get_base_address(h) (0xFFFFFFF0 & pci_read_config_32(h,0x10))
#define pci_get_base_address_n(h,i) (0xFFFFFFF0 & pci_read_config_32(h,0x10 + 4*(i)))

#define pci_get_irq_level(h) pci_read_config_8(h,0x3C)

#define pci_get_status(h) pci_read_config_16(h,0x06)

#define pci_get_command(h) pci_read_config_16(h,0x04)
#define pci_set_command(h,v) pci_write_config_16(h,0x04,v)

#define pci_get_revision(h) pci_read_config_8(h,0x08)

  /**
   * wants to be an even multiple of 8; rounds down
   **/
#define pci_get_master_latency_timer(h) pci_read_config_8(h,0x0D)
#define pci_set_master_latency_timer(h,v) pci_write_config_8(h,0x0D,v)

  /**
   * is read-only, so set does not work
   *
   * measured in .25 uS
   **/
#define pci_get_max_latency(h) pci_read_config_8(h,0x3F)
#define pci_set_max_latency(h,v) pci_write_config_8(h,0x3F,v)

  /**
   * is read-only, so set doesn't work
   *
   * measured in .25 uS
   **/
#define pci_get_min_grant(h) pci_read_config_8(h,0x3E)
#define pci_set_min_grant(h,v) pci_write_config_8(h,0x3E,v)

#define pci_get_cache_line_size(h) pci_read_config_8(h,0x0C)
#define pci_set_cache_line_size(h,v) pci_write_config_8(h,0x0C,v)

  /**
   * assumes little-endian bit ordering,
   * and structure packing
   **/
  struct PciCommand
  {
    uint16_t input_output_space_enable:1;
    uint16_t memory_space_enable:1;
    uint16_t bus_master_enable:1;
    uint16_t special_cycle_mon_enable:1;
    uint16_t mem_write_inval_enable:1;
    uint16_t vga_palette_snoop_enable:1;
    uint16_t parity_error_response:1;
    uint16_t stepping_enable:1;
    uint16_t system_error_response:1;
    uint16_t fast_back_to_back_enable:1;
      uint16_t:6;
  };

  struct PciStatus
  {
    uint16_t:5;
    uint16_t is_66_mhz_capable:1;
    uint16_t user_def_feature_support:1;
    uint16_t fast_back_to_back_capable:1;
    uint16_t signaled_parity_error:1;
    uint16_t devsel_timing:2;      /* 00=fast 01=medium 10=slow 11=reserved */
    uint16_t signaled_target_abort:1;
    uint16_t received_target_abort:1;
    uint16_t received_master_abort:1;
    uint16_t signaled_system_error:1;
    uint16_t detected_parity_error:1;
  };


  /**
   * stuff that is specific to motherboard chipsets
   *
   *
   * how many PCI wait-states will add during the burst portion
   * of a PCI master read or write cycle targeted (I think
   * that means host memory)
   *
   * only defined for 82437VX
   *
   * 000 = 2 cycles
   * 001 = 4 cycles
   * 010 = 6 cycles
   * 011 = 8 cycles (default, matches PCI 2.0)
   * 1xx = reserved
   **/
#define pci_get_trdy_timer(h) pci_read_config_8(h,0x69)
#define pci_set_trdy_timer(h,v) pci_write_config_8(h,0x69,v)

  /**
   * how long will a PCI master be allowed to retain ownership
   * of the bus (from initial assertion of grant) (minimum)
   *
   * allows a PCI device to do multiple transactions by keeping
   * its request asserted
   *
   * only defined for 82437VX and 82439HX
   *
   * measured in multiples of 4 PCI clock cycles
   * (bottom 2 bits are forced to zeroes)
   **/
#define pci_get_multi_transaction_timer(h) pci_read_config_8(h,0x70)
#define pci_set_multi_transaction_timer(h,v) pci_write_config_8(h,0x70,v)

  /**
   * only applies to 82437FX and 82439HX PCI/host bridge chips
   *
   * note that bit mapping is different on the 2 chips (see below)
   **/
#define pci_get_control(h) pci_read_config_8(h,0x50)
#define pci_set_control(h,v) pci_write_config_8(h,0x50,v)

  /**
   * only applies to 82437FX and 82439HX PCI/host bridge chips
   **/
#define pci_get_cache_control(h) pci_read_config_8(h,0x52)
#define pci_set_cache_control(h,v) pci_write_config_8(h,0x52,v)

  /**
   * only applies to 82371FB and 82371SB PCI/ISA accelerator (PIIX and
   * PIIX3) chips
   **/
#define pci_get_smi_control(h) pci_read_config_8(h,0xA0)
#define pci_set_smi_control(h,v) pci_write_config_8(h,0xA0,v)

  /**
   * only applies to 82371FB and 82371SB PCI/ISA accelerator (PIIX and
   * PIIX3) chips
   **/
#define pci_get_smi_enable(h) pci_read_config_16(h,0xA2)
#define pci_set_smi_enable(h,v) pci_write_config_16(h,0xA2,v)

  /**
   * only applies to 82371FB and 82371SB PCI/ISA accelerator (PIIX and
   * PIIX3) chips
   **/
#define pci_get_smi_event_enable(h) pci_read_config_32(h,0xA4)
#define pci_set_smi_event_enable(h,v) pci_write_config_32(h,0xA4,v)

  /**
   * only applies to 82371FB and 82371SB PCI/ISA accelerator (PIIX and
   * PIIX3) chips
   *
   * has actual value minus 1
   **/
#define pci_get_smi_fastoff_timer(h) pci_read_config_8(h,0xA8)
#define pci_set_smi_fastoff_timer(h,v) pci_write_config_8(h,0xA8,v)

  /**
   * only applies to 82371FB and 82371SB PCI/ISA accelerator (PIIX and
   * PIIX3) chips
   **/
#define pci_get_smi_request(h) pci_read_config_16(h,0xAA)
#define pci_set_smi_request(h,v) pci_write_config_16(h,0xAA,v)

  /**
   * only applies to 82437FX and 82439HX PCI/host bridge chips
   **/
#define pci_get_smi_ram_control(h) pci_read_config_8(h,0x72)
#define pci_set_smi_ram_control(h,v) pci_write_config_8(h,0x72,v)

  /**
   * only applies to 82371FB and 82371SB PCI/ISA accelerator (PIIX and
   * PIIX3) chips
   **/
#define pci_get_ide_timing_n(h,secondary_ch) pci_read_config_16(h,0x40+2*secondary_ch)
#define pci_set_ide_timing_n(h,secondary_ch,v) pci_write_config_16(h,0x40+2*secondary_ch,v)

  /**
   * only applies to 82371FB and 82371SB PCI/ISA accelerator (PIIX and
   * PIIX3) chips
   **/
#define pci_get_slave_ide_timing_n(h) pci_read_config_16(h,0x44)
#define pci_set_slave_ide_timing_n(h,v) pci_write_config_16(h,0x44,v)


  /**
   * measured in cycles
   **/
#define pci_get_max_slave_latency(h) pci_read_config_8(h,0x40)
#define pci_set_max_slave_latency(h,v) pci_write_config_8(h,0x40,v)

  /**
   * nonstandard
   **/
#define pci_get_master_control(h) pci_read_config_8(h,0x41)

  /**
   * nonstandard
   **/
#define pci_set_master_control(h,v) pci_write_config_8(h,0x41,v)

  /**
   * nonstandard
   **/
  /* measured in 32-bit words */
#define pci_get_threshold(h) pci_read_config_8(h,0x42)

  /**
   * nonstandard
   **/
#define pci_set_threshold(h,v) pci_write_config_8(h,0x42,v)

#ifdef __cplusplus
}
#endif



#if _82437FX
/**
 * as far as I know, only defined on 82437FX
 * System Controller
 *
 * it is recommended to use 3 or 5 PCI clocks for the
 * CPU inactivity timer
 **/
struct PciControl
{
  uint8_t bus_concurrency_disable:1;
  uint8_t pci_streaming_disable:1;
  uint8_t cpu_to_pci_write_bursting_disable:1;
  uint8_t peer_concurrency_enable:1;
  uint8_t spare:1;
  /* actual value minus one, measured in PCI clocks */
  uint8_t cpu_inactivity_timer:3;
};
#else
#if _82439HX
/* as far as I know, only defined on 82439HX */
/* System Controller */
/* */
struct PciControl
{
  uint8_t global_txc_enable:1;
  uint8_t:1;                     /* reserved */
  /* 0=open drain compatible with PCI, 1=normal output driven high
     when negated */
  uint8_t serr_pound_output_type:1;
  uint8_t peer_concurrency_enable:1;
  /* 0=use 82437FX policies, 1=use policies for dual-procesor PCI 2.1 */
  uint8_t dual_processor_na_pound_enable:1;
  /* 0=forward host shutdown cycle to PCI bus, 1=write 01h to port 92 */
  uint8_t shutdown_to_port_92:1;
  uint8_t ecc_test_enable:1;       /* 0=normal mode, 1=test mode */
  /* 0=parity, 1=ECC (also adjusts DRAM timings) */
  uint8_t dram_ecc_or_parity_select:1;
};
#endif
#endif



#if _82437FX || _82439HX
/* as far as I know, only defined on 82437FX */
/* and 82439HX System Controller */
/* */
struct PciCacheControl
{
  uint8_t first_level_cache_enable:1;
  uint8_t secondary_cache_force_miss_or_invalidate:1;
#if _82439HX
   /* 0=64MB 1=512MB (enables TIO[10:8] lines) */
  uint8_t extended_cachability_enable:1;
#else
  uint8_t:1;                     /* reserved (for guess who?) */
#endif
  /* must be setup before either L1 or L2 cache is enabled... but what
     does NA# signal do? */
  uint8_t na_pound_signal_disable:1;
  /* 00=pipe burst, 01=burst, 10=async, 11=512K dual bank pipe burst */
  uint8_t static_ram_type:2;
  /* 00=not populated, 01=256K, 10=512K, 11=reserved */
  uint8_t secondary_cache_size:2;
};

/* Phil, for some reason this wouldn't work under Borland */
#ifdef PHIL
typedef enum PciCacheControlSramType
{
    pipelined_burst = 0,
    burst,                                      /* reserved on 82439HX */
      asynchronous,                             /* reserved on 82439HX */
    dual_512k_pipe_burst
};

typedef enum PciCacheControlSecondaryCacheSize
{
    not_populated = 0,
    has_256k,
    has_512k,
    reserved
};
#endif /* PHIL */
#endif



/* as far as I know, only defined on 82371FB */
/* PCI ISA accelerator (PIIX) */
/* */
struct PciSmiControl
{
  uint8_t smi_gate:1;
  uint8_t stpclk_signal_enable:1;
  uint8_t stpclk_scaling_enable:1;
  /* 00=minutes, 01=disabled, 10=PCICLK, 11=mS */
  uint8_t fastoff_timer_granularity:2;
  uint8_t:3;                     /* reserved */
};

/* Phil, for some reason this wouldn't work under Borland */
#ifdef PHIL
typedef enum PciSmiControlFastoffGranularity
  {
    /* actually 1.0 min @33 MHz PCICLK, 1.1 min @30 MHz, 1.32 min @25 MHz */
    one_minute = 0,
    disabled,
    one_pci_clock,
    /* actually 1.0 mS  @33 MHz PCICLK, 1.1 mS  @30 MHz, 1.32 mS  @25 MHz */
    one_millisecond
};
#endif /* PHIL */


 /* as far as I know, only defined on 82371FB */
 /* PCI ISA accelerator (PIIX) */
 /* */
struct PciSmiEnable
{
  uint16_t irq_1_smi_enable:1;     /* keyboard */
  uint16_t irq_3_smi_enable:1;     /* COM1/COM3 or mouse */
  uint16_t irq_4_smi_enable:1;     /* COM2/COM4 or mouse */
  uint16_t irq_8_smi_enable:1;     /* realtime clock alarm */
  uint16_t irq_12_smi_enable:1;    /* PS/2 mouse */
  uint16_t fastoff_timer_smi_enable:1;     /* when it decrements to zero */
  uint16_t extsmi_smi_enable:1;
  uint16_t apmc_write_enable:1;   /* program's write to APM command register */
  uint16_t:8;                    /* reserved */
};


/**
 * as far as I know, only defined on 82371FB
 * PCI ISA accelerator (PIIX)
 *
 * system events keep the system from powering down
 * reloading the fast-off timer with its initial value
 *
 * break events can wake up a powered-down system by
 * negating STPCLK#
 **/
/* Phil, for some reason this wouldn't work under Borland */
#ifdef PHIL
struct PciSmiEventEnable
{
  uint32_t fastoff_irq_0_enable      : 1;        /* system and break events */
  uint32_t fastoff_irq_1_enable      : 1;        /* system and break events */
  uint32_t spare1                    : 1;
  uint32_t fastoff_irq_3_enable      : 1;        /* system and break */
  uint32_t fastoff_irq_4_enable      : 1;        /* system and break */
  uint32_t fastoff_irq_5_enable      : 1;        /* system and break */
  uint32_t fastoff_irq_6_enable      : 1;        /* system and break */
  uint32_t fastoff_irq_7_enable      : 1;        /* system and break */
  uint32_t fastoff_irq_8_enable      : 1;        /* system and break */
  uint32_t fastoff_irq_9_enable      : 1;        /* system and break */
  uint32_t fastoff_irq_10_enable     : 1;        /* system and break */
  uint32_t fastoff_irq_11_enable     : 1;        /* system and break */
  uint32_t fastoff_irq_12_enable     : 1;        /* system and break */
  uint32_t fastoff_irq_13_enable     : 1;        /* system and break */
  uint32_t fastoff_irq_14_enable     : 1;        /* system and break */
  uint32_t fastoff_irq_15_enable     : 1;        /* system and break */
  uint32_t spare2                    : 13;
  uint32_t fastoff_nmi_enable        : 1;        /* system and break */
  uint32_t intr_enable               : 1;        /* break only */
  uint32_t fastoff_smi_enable        : 1;        /* system and break */
};
#endif /* PHIL */


 /**
  * as far as I know, only defined on 82371FB
  * PCI ISA accelerator (PIIX)
  *
  * note that these are all edge sensitive, so if an
  * SMI cause is still occurring when CPU writes zero
  * to the corresponding bit in this register, the bit
  * will not be set again by PIIX unless it goes away
  * and comes back
  **/
struct PciSmiRequest
{
  uint16_t irq_1_caused_smi:1;
  uint16_t irq_3_caused_smi:1;
  uint16_t irq_4_caused_smi:1;
  uint16_t irq_8_caused_smi:1;
  uint16_t irq_12_caused_smi:1;
  /* but it reloaded and kept counting afterwards */
  uint16_t fastoff_timer_expired:1;     
  uint16_t extsmi_caused_smi:1;
  uint16_t apmc_write_caused_smi:1;
  uint16_t spare:8;
};



/**
 * as far as I know, only defined on 82437FX 
 * System Controller (TSC)
 *
 * note: software must ensure that smm_space_open
 * and smm_space_closed are not both TRUE at the
 * same time
 **/
struct PciSmiRamControl
{
  uint8_t smm_space_base_segment:3;       /* must be 010 = A0000h to BFFFFh */
  /* 128K bytes accessible when ADS# and SMIACT# asserted */
  uint8_t sm_ram_enable:1;

  /* smm_space_open and smm_space_closed become R/O; must reset to clear
     this bit */
  uint8_t smm_space_locked:1;

  /* data passes through, code refs see SM RAM if SMIACT# asserted */      
  uint8_t smm_space_closed:1;
  uint8_t smm_space_open:1;   /* code & data see SM RAM regardles of SMIACT# */
  uint8_t:1;                     /* reserved */
};



/* as far as I know, only defined on FORE PCA200E OC3c ATM NIC */
/* */
struct PciMasterControl
{
  uint8_t disable_cache_line_reads:1;
  uint8_t disable_write_and_invals:1;
  uint8_t write_inval_needs_2_lines:1;
  uint8_t ignore_latency_timer:1;
  /* don't deassert REQ# if in and out BIFO's have threshold fullness */
  uint8_t enable_cont_request_mode:1;
  /* (?input only) wait to assert REQ# until threshold input FIFO
     words available */
  uint8_t force_large_pci_bus_bursts:1;
  uint8_t byteswap_slave_ram_access:1;
  uint8_t spare:1;
};



/**
 * someday expand this interface so that the full
 * 64 bytes of configuration registers defined in
 * chapter 17 of PCI System Architecture, 3rd ed.
 * including bit fields where necessary
 **/


#endif /* _PCI_H */
