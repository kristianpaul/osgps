@rem Build batch file for Turbo C version 2.01

@rem  Flag explaination:
@rem   -1    80186/80286 Instructions  (opposed to 8086 only)
@rem   -G    Generate for speed  
@rem   -a    Generate word alignment
@rem   -f87  Generate 8087 floating point instructions
@rem   -mm   Medium Memory Model

tcc -1 -G -a -f87 -ml gpsrcvr.c irq.c gpsisr.c gp2021.c nav_fix.c gpsfuncs.c interfac.c serport.c nmea.c fwinter.c

