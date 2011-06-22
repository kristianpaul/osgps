# To build modules outside of the kernel tree, we run "make"
# in the kernel source tree; the Makefile these then includes this
# Makefile once again.
# This conditional selects whether we are being included from the
# kernel Makefile or not.
ifeq ($(PATCHLEVEL),)

    # Assume the source tree is where the running kernel was built
    # You should set KERNELDIR in the environment if it's elsewhere
    KERNELDIR ?= /lib/modules/$(shell uname -r)/build
    # The current directory is passed to sub-makes as argument
    PWD := $(shell pwd)


PROJ=gpsrcvr
SRC=  gpsfuncs.c gpsrcvr.c nav_fix.c linuxusr.c nmea.c FwInter.c rinex.c
GUSRC=gpsfuncs.c gpsrcvr.c nav_fix.c interfac.c
SOFTSRC=SoftOSGPS.c gpsfuncs.c nav_fix.c rinex.c gpsisr.c gp2021.c interfac.c correlator.c display.c
# NMEA.c serport.c FwInter.c

CFLAGS=-g3 -O3 -Wall -W -pedantic -D_GNU_SOURCE -D_LARGEFILE_SOURCE -D_FILE_OFFSET_BITS=64 -DSOFT # -Werror
LDFLAGS=-lm



all: .depend SoftOSGPS 90-osgps.rules

hardware: gpsrcvr modules regtest

modules:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

modules_install:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules_install

gpsrcvr: $(SRC:.c=.o)
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

gpsuser: $(GUSRC:.c=.o)
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

SoftOSGPS:$(SOFTSRC:.c=.o)
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

90-osgps.rules:
	echo 'KERNEL=="gps.status",     NAME="gps/status", MODE="0666"' > $@
	echo 'KERNEL=="gps.measurement",NAME="gps/measurement", MODE="0666"' >> $@
	echo 'KERNEL=="gps.data",       NAME="gps/data", MODE="0666"' >> $@

install: modules_install
	cp 90-osgps.rules /etc/udev/rules.d/

clean:
	$(RM) -r *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions
	$(RM) gpsrcvr regtest SoftOSGPS *.o *~ 
	$(RM) *.lib *.obj *.exe *.exe

.depend:
	$(RM) $@
	gcc -MM $(CFLAGS) $(SRC) > $@

.PHONY: modules modules_install clean depend

include .depend

else
    # called from kernel build system: just declare what our modules are
    gp2021km-objs := linuxmod.o gp2021.o gpsisr.o interfac.o
    obj-m := gp2021km.o
endif
