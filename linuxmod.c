/*
 * A Linux Device Driver (aka a kernel module) for the GP2021 GPS
 * Correlator.  However, the goal of the design was to make it as
 * general as possible for other GPS Correlators.
 *
 * License: GNU General Public License (GPL)
 * Author: Frederick (Rick) A Niles <niles@rickniles.com>
 *
 * $Id: linuxmod.c,v 1.13 2008/09/17 21:02:43 fniles Exp $
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/interrupt.h>    /* tasklets */
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/device.h>
#include <linux/pci.h>
#include <linux/moduleparam.h>
#include <asm/rtc.h>

#include "interfac.h"

#define MIN(x,y) (x < y ? x : y)

MODULE_LICENSE ("GPL");

/* Prototype of generic GP2021 ISR */
extern void gpsisr (void);
extern void cold_allocate2 (void);

/* Three Devices (may want to a add debug device) */
#define GP2021_DEVS 4
enum { DEV_STATUS, DEV_MEASUREMENT, DEV_DATA, DEV_DEBUG };
static char *dev_name[] = { "status", "measurement", "data", "debug" };

/* Status is created on the fly so output struct must be global */
static struct tracking_status status[N_channels];

/**
 * The raw class needs to be allocated when the module is loaded and
 * destroyed when the module is removed, so both functions need access
 * to this.  Ideally, this would be stored in a module specific
 * structure.
 **/
static struct class *raw_class;

/* Internal boolean state of if allocation should happen
   automatically */
static int cold_allocate = 1;


/* Setting this to zero means we're going to use a dynamic major
   device number. */
static int gp2021_major = 0;

#define INTERRUPT_RTC 1
#ifdef INTERRUPT_RTC
/* Needed to use the real-time clock interrupt */
static rtc_task_t rtc_task;
#endif

static struct pci_dev *osgps_pci_dev = NULL;

/* Device specfic structure */
struct gp2021_dev
{
  void *data;
  int *data_ready_flag;
  int max_data_size;
  struct semaphore sem; /* Currently unused */
  struct cdev cdev;     /* Character Device information */
  int type;    /* Identifier to determine which type of device this is */
  wait_queue_head_t wait;
};

/* Again, perhaps this should be rolled into a single module
   structure */
static struct gp2021_dev gp2021_dev[GP2021_DEVS];

static unsigned short int bldraddr = 0;
module_param (bldraddr, ushort, S_IRUGO);

/**
 * Get the time-of-week (TOW) from the real-time clock.
 *
 * Caveat: Using this function assumes the clock time is correct.
 **/
long
get_tow (void)
{
  int day_of_week;
  long days_from_1970;
  long today_seconds;
  const long SECS_PER_DAY = (3600 * 24);
  const long leap_seconds = 13;
  struct timeval tv;

  do_gettimeofday (&tv);

  days_from_1970 = (tv.tv_sec / SECS_PER_DAY);

  today_seconds = tv.tv_sec - days_from_1970 * SECS_PER_DAY;

  /* January 1, 1970 was a Thursday (day 4) */
  day_of_week = (days_from_1970 + 4) % 7;

  return day_of_week * SECS_PER_DAY + today_seconds + leap_seconds;
}


static void
minute_tasklet (unsigned long int i __attribute__ ((unused)))
{
  if (cold_allocate)
    cold_allocate2 ();
}

static DECLARE_TASKLET (one_minute_tasklet, minute_tasklet, 0);

/**
 * Currently only the real-time clock interrupt works.  However, I
 * would like to support the built-in interrupt from the GP2021 as
 * well, since it should work better.
 **/

#ifdef INTERRUPT_PCI
irqreturn_t
gp2021_interrupt (int a, void *b, struct pt_regs *regs)
#else
void
gp2021_interrupt (void *private_data)
#endif
{
  /* Update our estimate of the time-of-week based on the real-time
     clock time. You must set the real-time clock externally using
     something like `rdate' */
  clock_tow = get_tow ();

  /* Do the real work here */
  gpsisr ();

  /* Do the tasklet if the min_flag come up. */
  if (min_flag) {
    /* Temporary!! */
    min_flag = 0;
    tasklet_schedule (&one_minute_tasklet);
  }

  /* Tell the data device file that new data is available. */
  if (data_frame_ready)
    wake_up_interruptible (&gp2021_dev[DEV_DATA].wait);

  /* Notify the measurement device that a new measurement is ready. */
  if (nav_flag)
    wake_up_interruptible (&gp2021_dev[DEV_MEASUREMENT].wait);

}
/**
 * Using the /proc filesystem is considered bad style by kernel
 * developers these days.  However, it's really easy to use and this
 * function is not essential to proper function of the receiver.  Some
 * day in the future, access to the /proc filesystem maybe removed.
 * We should move something like this to the new approved version the
 * /sys filesystem.
 **/
int
osgps_read_procmem (char *buf, char **start, off_t offset, int count,
                    int *eof, void *data)
{
  int ch;
  int len = 0;

  len +=
    sprintf (buf,
             "PRN State N_Freq t_count n_frame sfid Missed TOW_Sync C/No\n");
  for (ch = 0; ch < N_channels; ch++) {
    len +=
      sprintf (buf + len,
               "%3d   %d   %3d   %5d  %4d    %4d   %4d    %4d    %2d\n",
               ichan[ch].prn, ichan[ch].state, ichan[ch].n_freq,
               ichan[ch].frame_bit % 1500, ichan[ch].n_frame, ichan[ch].sfid,
               ichan[ch].missed, ichan[ch].tow_sync, ichan[ch].CNo);
  }

  *eof = 1;
  return len;
}

/**
 * File operations
 **/

/* Open: Nothing special here */
int
gps_open (struct inode *inode, struct file *filp)
{
  int type = inode->i_rdev & 0xff;

  struct gp2021_dev *dev = &gp2021_dev[type];

  filp->private_data = dev;
  return 0;
}

/**
 * This function handles writes to device files.  The goal of this
 * interface is to be generic for other receivers. This means keep
 * things to a minimum.
 *
 *  *** Status ***
 * Writing to the status device with ASCII will be interrupted as:
 * channel PRN Doppler(in mHz).
 *
 * E.g. "echo 2 29 5321 > /dev/gps/status"
 * Will set channel 2 to PRN 29 with a center frequency of 5.321 Hz
 *
 *  *** Data ***
 * Writing to the data device doesn't do anything currently.
 *
 *  *** Measurement ***
 * Writing to the measurement device can set the TIC period.  This is
 * not used currently and maybe removed in the future.
 **/
ssize_t
gps_write (struct file * filp, const char __user * buf, size_t count,
           loff_t * f_pos)
{
  struct gp2021_dev *dev = (struct gp2021_dev *) filp->private_data;
  char kern_buf[80];
  int ch = 0, prn = 0, n;
  long mdop = 0, code_corr, carrier_corr;
  uint16_t tic_val;


  switch (dev->type) {
  case DEV_STATUS:
    /* Set up a channel */

    n = MIN (count, 80);
    if (copy_from_user (kern_buf, buf, n))
      return -EFAULT;

    kern_buf[n] = '\0';

    sscanf (kern_buf, "%d %d %ld", &ch, &prn, &mdop);

    if (prn > -1) {
      code_corr = mdop >> 16;
      /* I hope this integer division is NOT using the FPU */
      carrier_corr = -mdop / 42; 
      channel_off (ch);
      if (prn != 0)
        setup_channel (ch, prn, code_corr, carrier_corr);
      cold_allocate = 0;
      search_max_f = 5;
    }
    else {
      cold_allocate = 1;
      search_max_f = 50;
      cold_allocate2 ();
    }
    break;
  case DEV_DATA:
    /* Nothing here... */
    break;
  case DEV_MEASUREMENT:
    /* Set the TIC interval */
    if (copy_from_user (&tic_val, buf, 2))
      return -EFAULT;
    set_TIC (tic_val);
    break;
  }
  return count;
}

/**
 * Reading from the device files. 
 *
 *  *** Status ***
 *
 * See the definition of the tracking_status structure for binary
 * output of this device.  This data is created at the time this
 * function is called so it is treated separately below.
 *
 *  *** Data ***
 *
 * The data device output is 1500 16-bit words.  Each bit location
 * represents a channel.  Since there are only 12-channels the top
 * four bits won't be set.  The userspace program must mask out the
 * particular bit slot of interest for all 1500 words to reconstruct
 * the data message.  Right now this output once a frame (30 seconds),
 * but in the future it will come out once per sub-frame (6 seconds).
 * This faster rate will not only allow for faster aquistion, but
 * enable WAAS messages at 250 bits/second instead of the GPS 50
 * bits/second.
 *
 *  *** Measurement **
 *
 * This output is a function of the number of satellites tracked so
 * the size must be adjusted for each call.  See the definition of the
 * measurement_set structure for a description of the binary output.
 **/
ssize_t
gps_read (struct file * filp, char __user * buf, size_t count, loff_t * f_pos)
{
  struct gp2021_dev *dev = (struct gp2021_dev *) filp->private_data;
  int i, data_size;

  if (count < dev->max_data_size)
    return -ENOMEM;

  /* Unless overridden */
  data_size = dev->max_data_size;

  switch (dev->type) {
  case DEV_STATUS:
    /* Status message get assembled on the fly */
    for (i = 0; i < N_channels; i++) {
      status[i].prn = ichan[i].prn;
      status[i].stat = (ichan[i].state & 0x7)
        | ((ichan[i].tow_sync & 0x1) << 3) | (ichan[i].sfid & 0x7) << 4;
      status[i].n_freq = ichan[i].n_freq;
      status[i].CNo = ichan[i].CNo;
      status[i].missed = ichan[i].missed;
      status[i].n_frame = ichan[i].n_frame;
    }

    break;

  case DEV_DATA:
  case DEV_MEASUREMENT:
  case DEV_DEBUG:
    /* Block waiting for next data */
    while (*(dev->data_ready_flag) == 0) {
      if (filp->f_flags & O_NONBLOCK)
        return -EAGAIN;
      if (wait_event_interruptible (dev->wait, *(dev->data_ready_flag)))
        return -ERESTARTSYS;
    }

    *(dev->data_ready_flag) = 0;

    /* Reduce the size to the measurement */
    if (dev->type == DEV_MEASUREMENT) {
      for (i = 0; i < N_channels; i++)
        if (measurements.measurement[i].transmit_time_offset == 0)
          break;

      data_size = sizeof (struct measurement_set)
        - (N_channels - i) * sizeof (struct measurement);
    }

    break;
  }

  if (copy_to_user (buf, dev->data, data_size))
    return -EFAULT;

  return data_size;
}

/* release (aka close) nothing special */
int
gps_release (struct inode *inode, struct file *filp)
{
  return 0;
}

/* The poll method lets the user know if the device is ready for
   reading or writing. */
unsigned int
gps_poll (struct file *filp, poll_table * wait)
{
  struct gp2021_dev *dev = (struct gp2021_dev *) filp->private_data;
  int mask = POLLOUT | POLLWRNORM;

  if (dev->data_ready_flag != NULL)
    poll_wait (filp, &dev->wait, wait);
  if ((dev->data_ready_flag == NULL) || *(dev->data_ready_flag))
    mask |= POLLIN | POLLRDNORM;

  return mask;
}

/** 
 * The _probe and _remove methods are currently just stubs, but should
 * be figuring out if the device is detected.  We currently do all
 * this at module insertion time, but these methods are designed for
 * hot-swappable devices.  This should probably be fixed in the future
 * to do all the device detection here.
**/
static int __devinit osgpspci_probe(struct pci_dev *pdev, 
				    const struct pci_device_id *id)
{
  return 0;
}

static void __devexit osgpspci_remove(struct pci_dev *pdev)
{
}

struct file_operations gp2021_fops = {
  .owner = THIS_MODULE,
  .read = gps_read,
  .write = gps_write,
  .open = gps_open,
  .release = gps_release,
  .poll = gps_poll,
};

/* End of File Operations section */


/* Submit to linux/pci_ids.h someday */
#define PCI_SUBDEVICE_ID_GPS1005 0x3077

static struct pci_device_id osgpspci_id_table[] = {
  {  PCI_VENDOR_ID_PLX, PCI_DEVICE_ID_PLX_9050,
     PCI_VENDOR_ID_PLX, PCI_SUBDEVICE_ID_GPS1005, 0, 0, 0},
  {  0, 0, 0, 0, 0, 0, 0}
};

MODULE_DEVICE_TABLE(pci, osgpspci_id_table);

static struct pci_driver osgpspci_driver __attribute__ ((unused)) = {
	.name		= "osgps",
	.id_table	= osgpspci_id_table,
	.probe		= osgpspci_probe,
	.remove		= __devexit_p(osgpspci_remove),
};


/* End of file operations section */

/**
 * Check PCI sub-system for GPSCreations 1005
 **/
int
pcifind (void)
{
  int io_port;

  /* return pci_module_init(&osgpspci_driver); */
  osgps_pci_dev = pci_get_subsys( PCI_VENDOR_ID_PLX, PCI_DEVICE_ID_PLX_9050,
				  PCI_VENDOR_ID_PLX, PCI_SUBDEVICE_ID_GPS1005, 
				  osgps_pci_dev);

  if ((osgps_pci_dev == NULL) || (pci_enable_device(osgps_pci_dev) < 0)) {
    printk(KERN_ALERT "Unable to enable OSGPS PCI device, "
	   "assuming ISA at 0x300\n");
    return 0;
  }
  else {
    /* The I/O port address is stored in the fourth configuration
       spot.  Since you start counting at zero, this is 3. */
    io_port = pci_resource_start(osgps_pci_dev, 3);
  }

  return io_port;
}

static int
gp2021km_init (void)
{
  int i;
  dev_t dev = MKDEV (gp2021_major, 0);

  printk (KERN_ALERT "Open Source GPS Zarlink GP2021 module."
	  " Clifford Kelley and Rick Niles.\n");
  
  /* Find the card and bail out if we can not find it. */
  if (gp2021_detect (bldraddr))
    return -1;

  /* Initialize the correlator */
  gp2021_init ();

  /* Register the interrupt handler */
#ifdef INTERRUPT_RTC
  {
    rtc_task.func = gp2021_interrupt;
    rtc_task.private_data = NULL;

    if (rtc_register (&rtc_task)) {
      printk (KERN_ALERT "unable to register osgps with the RTC.\n");
      return -EIO;
    }
    if (rtc_control (&rtc_task, RTC_IRQP_SET, 4096)) {
      printk (KERN_ALERT "unable to set RTC rate.\n");
      rtc_unregister (&rtc_task);
      return -EIO;
    }

    if (rtc_control (&rtc_task, RTC_PIE_ON, 1)) {
      printk (KERN_ALERT "unable to turn on RTC periodic interrupt.\n");
      rtc_unregister (&rtc_task);
      return -EIO;
    }

  }

#else /* INTERRUPT_PCI */
  request_irq (osgps_pci_dev->irq, gp2021_interrupt, SA_SHIRQ, "gp2021", 
	       (void *) osgps_pci_dev);
#endif
  min_flag = 1;

  /* /proc filesystem creation */
  create_proc_read_entry ("osgps", 0, NULL, osgps_read_procmem, NULL);

  /* Setup device data structure */
  gp2021_dev[DEV_MEASUREMENT].data = &measurements;
  gp2021_dev[DEV_MEASUREMENT].data_ready_flag = &nav_flag;
  gp2021_dev[DEV_MEASUREMENT].max_data_size = sizeof (struct measurement_set);
  gp2021_dev[DEV_MEASUREMENT].type = DEV_MEASUREMENT;
  init_waitqueue_head (&gp2021_dev[DEV_MEASUREMENT].wait);

  gp2021_dev[DEV_DATA].data = data_message;
  gp2021_dev[DEV_DATA].data_ready_flag = &data_frame_ready;
  gp2021_dev[DEV_DATA].max_data_size = 1500 * sizeof (short);
  gp2021_dev[DEV_DATA].type = DEV_DATA;
  init_waitqueue_head (&gp2021_dev[DEV_DATA].wait);

  gp2021_dev[DEV_STATUS].data = &status;
  gp2021_dev[DEV_STATUS].data_ready_flag = NULL;
  gp2021_dev[DEV_STATUS].max_data_size =
    sizeof (struct tracking_status) * N_channels;
  gp2021_dev[DEV_STATUS].type = DEV_STATUS;

  gp2021_dev[DEV_DEBUG].data = &debug_data;
  gp2021_dev[DEV_DEBUG].data_ready_flag = &debug_ready;
  gp2021_dev[DEV_DEBUG].max_data_size = sizeof (struct debug_struct);
  gp2021_dev[DEV_DEBUG].type = DEV_DEBUG;
  init_waitqueue_head (&gp2021_dev[DEV_DEBUG].wait);

  /* Allocate the device files. Only setup the device files after the
     gp2021 is all setup */
  alloc_chrdev_region (&dev, 0, GP2021_DEVS, "gps");
  gp2021_major = MAJOR (dev);
  raw_class = class_create (THIS_MODULE, "gps");

  for (i = 0; i < GP2021_DEVS; i++) {
    dev_t devnum = MKDEV (gp2021_major, i);
    cdev_init (&gp2021_dev[i].cdev, &gp2021_fops);
    gp2021_dev[i].cdev.owner = THIS_MODULE;
    gp2021_dev[i].cdev.ops = &gp2021_fops;
    if (cdev_add (&gp2021_dev[i].cdev, devnum, 1)) {
      printk (KERN_ALERT "Error adding gp2021 kernel device %d\n", i);
    }
    class_device_create (raw_class, NULL, devnum, NULL, "gps.%s",
			 dev_name[i]);
  }

  return 0;
}

static void
gp2021km_exit (void)
{
  int i;
  printk (KERN_ALERT "Unloading OSGPS GP2021\n");

#ifdef INTERRUPT_RTC
  rtc_control (&rtc_task, RTC_PIE_OFF, 1);
  rtc_control (&rtc_task, RTC_IRQP_SET, 1024);
  rtc_unregister (&rtc_task);
#elif (defined INTERRUPT_PCI)
  free_irq (osgps_pci_dev->irq, (void *) osgps_pci_dev);
#endif
  for (i = 0; i < GP2021_DEVS; i++)
    cdev_del (&gp2021_dev[i].cdev);
  unregister_chrdev_region (MKDEV (gp2021_major, 0), GP2021_DEVS);
  class_destroy (raw_class);
  remove_proc_entry ("osgps", NULL);
}

module_init (gp2021km_init);
module_exit (gp2021km_exit);
