#include <stdio.h>              /* printf() */
#include <stdlib.h>             /* exit() */
#include "pci.h"


/* Cliff change these to the OpenSource ones */
#define VENDOR_ID 0x10b5
#define DEVICE_ID 0x5201

#ifdef __linux__
int
pcifind (void)
{
  /* XXX FIX ME XXX */
  return 0;
}
#else
int
pcifind (void)
{
  uint16_t handle;
  uint16_t VendorID;
  uint16_t DeviceID;
  uint32_t address;
  unsigned irq_level;
  uint8_t revision;
  uint16_t bus, device, function;
  uint32_t device_and_vendor;
  uint32_t BaseAddress0;
  uint32_t BaseAddress1;
  uint32_t BaseAddress2;
  uint32_t BaseAddress3;
  uint32_t BaseAddress4;
  uint32_t BaseAddress5;

  BaseAddress3 = 0;
  /* check whether PCI BIOS is present */
  if (!pci_is_bios_present ())
    {
      printf ("Sorry, PCI BIOS is not present.\n");
      BaseAddress3 = 0;
    }
  else
    {
      printf ("searching for all valid PCI devices\n");
      /**
       * for all PCI busses...
       **/
      for (bus = 0; bus < PCI_BUSSES_PER_MACHINE; bus++)
        {
	  /**
	   * for all PCI devices on this bus...
	   **/
          for (device = 0; device < PCI_DEVICES_PER_BUS; device++)
            {
	      /**
	       * for all functions on this device...
	       **/
              for (function = 0; function < PCI_FUNCTIONS_PER_DEVICE;
                   function++)
                {
		  /**
		   * calculate the handle from bus, device, and function
		   **/
                  handle = (bus << 8) | (device << 3) | function;

		  /**
                   * get the device ID and vendor ID together,
		   * and also check if this is a valid device
		   **/

                  device_and_vendor = pci_read_config_32 (handle, 0);

                  DeviceID = (unsigned short) (device_and_vendor >> 16);
                  VendorID = (unsigned short) device_and_vendor;

                  /* Only dump our device */
                  if ((DeviceID != DEVICE_ID) && (VendorID != VENDOR_ID))
                    {
                      continue;
                    }

                  if (device_and_vendor == (uint32_t) - 1)
                    continue;

                  revision = pci_get_revision (handle);
                  address = pci_get_base_address (handle);


                  BaseAddress0 = pci_get_base_address_n (handle, 0);
                  BaseAddress1 = pci_get_base_address_n (handle, 1);
                  BaseAddress2 = pci_get_base_address_n (handle, 2);
                  BaseAddress3 = pci_get_base_address_n (handle, 3);
                  BaseAddress4 = pci_get_base_address_n (handle, 4);
                  BaseAddress5 = pci_get_base_address_n (handle, 5);


                  irq_level = pci_get_irq_level (handle);

                  printf
                    ("bus=%d device = %d func = %d (handle=0x%4.4X) saw device ID 0x%4.4X vendor ID 0x%4.4X revision 0x%.2X phys address 0x%8.8lX IRQ level %d\n",
                     bus, device, function, handle, DeviceID, VendorID,
                     revision, address, irq_level);

                  printf
                    ("Base addresses 0x%8.8lX 0x%8.8lX 0x%8.8lX 0x%8.8lX 0x%8.8lX 0x%8.8lX\n",
                     BaseAddress0, BaseAddress1, BaseAddress2, BaseAddress3,
                     BaseAddress4, BaseAddress5);


                }
            }
        }
    }
  printf ("PCI configuration discovery done\n\n");
  return (BaseAddress3);
}

#endif
