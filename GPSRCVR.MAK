# Microsoft Visual C++ generated build script - Do not modify

PROJ = GPSRCVR
DEBUG = 0
PROGTYPE = 6
CALLER = 
ARGS = 
DLLS = 
D_RCDEFINES = -d_DEBUG
R_RCDEFINES = -dNDEBUG
ORIGIN = MSVC
ORIGIN_VER = 1.00
PROJPATH = C:\CLIFF\OPENSO~1\1~1.18A\OSGPS118\
USEMFC = 0
CC = cl
CPP = cl
CXX = cl
CCREATEPCHFLAG = 
CPPCREATEPCHFLAG = 
CUSEPCHFLAG = 
CPPUSEPCHFLAG = 
FIRSTC = PCIFIND.C   
FIRSTCPP = CMATRIX3.CPP
RC = rc
CFLAGS_D_DEXE = /nologo /G2 /W3 /Zi /AM /Od /D "_DEBUG" /D "_DOS" /D "VCPP" /FR /Fd"PCIDUMP.PDB"
CFLAGS_R_DEXE = /nologo /G3 /FPc /W3 /AL /O2 /D "NDEBUG" /D "_DOS" /D "VCPP" /U "__linux__" /FR 
LFLAGS_D_DEXE = /NOLOGO /NOI /STACK:5120 /ONERROR:NOEXE /CO 
LFLAGS_R_DEXE = /NOLOGO /NOI /STACK:15000 /ONERROR:NOEXE 
LIBS_D_DEXE = oldnames mlibce 
LIBS_R_DEXE = oldnames llibce 
RCFLAGS = /nologo
RESFLAGS = /nologo
RUNFLAGS = 
OBJS_EXT = PCI.OBJ 
LIBS_EXT = 
!if "$(DEBUG)" == "1"
CFLAGS = $(CFLAGS_D_DEXE)
LFLAGS = $(LFLAGS_D_DEXE)
LIBS = $(LIBS_D_DEXE)
MAPFILE = nul
RCDEFINES = $(D_RCDEFINES)
!else
CFLAGS = $(CFLAGS_R_DEXE)
LFLAGS = $(LFLAGS_R_DEXE)
LIBS = $(LIBS_R_DEXE)
MAPFILE = nul
RCDEFINES = $(R_RCDEFINES)
!endif
!if [if exist MSVC.BND del MSVC.BND]
!endif
SBRS = PCIFIND.SBR \
		CMATRIX3.SBR \
		FWINTER.SBR \
		GP2021.SBR \
		GPSISR.SBR \
		GPSRCVR.SBR \
		NAV_FIX.SBR \
		SERPORT.SBR \
		GPSFUNCS.SBR \
		NMEA.SBR


PCI_DEP = 

PCIFIND_DEP = c:\cliff\openso~1\1~1.18a\osgps118\pci.h


CMATRIX3_DEP = c:\cliff\openso~1\1~1.18a\osgps118\cmatrix3.h


FWINTER_DEP = c:\cliff\openso~1\1~1.18a\osgps118\fwinter.h \
	c:\cliff\openso~1\1~1.18a\osgps118\nmea.h \
	c:\cliff\openso~1\1~1.18a\osgps118\structs.h


GP2021_DEP = c:\cliff\openso~1\1~1.18a\osgps118\structs.h \
	c:\cliff\openso~1\1~1.18a\osgps118\consts.h \
	c:\cliff\openso~1\1~1.18a\osgps118\globals.h \
	c:\cliff\openso~1\1~1.18a\osgps118\gp2021.h


GPSISR_DEP = c:\cliff\openso~1\1~1.18a\osgps118\structs.h \
	c:\cliff\openso~1\1~1.18a\osgps118\consts.h \
	c:\cliff\openso~1\1~1.18a\osgps118\globals.h \
	c:\cliff\openso~1\1~1.18a\osgps118\gpsfuncs.h \
	c:\cliff\openso~1\1~1.18a\osgps118\gp2021.h


GPSRCVR_DEP = c:\cliff\openso~1\1~1.18a\osgps118\serport.h \
	c:\cliff\openso~1\1~1.18a\osgps118\nmea.h \
	c:\cliff\openso~1\1~1.18a\osgps118\gp2021.h \
	c:\cliff\openso~1\1~1.18a\osgps118\consts.h \
	c:\cliff\openso~1\1~1.18a\osgps118\structs.h \
	c:\cliff\openso~1\1~1.18a\osgps118\globals.h \
	c:\cliff\openso~1\1~1.18a\osgps118\cmatrix3.h \
	c:\cliff\openso~1\1~1.18a\osgps118\gpsfuncs.h \
	c:\cliff\openso~1\1~1.18a\osgps118\gpsrcvr.h \
	sys/poll.h


NAV_FIX_DEP = c:\cliff\openso~1\1~1.18a\osgps118\structs.h \
	c:\cliff\openso~1\1~1.18a\osgps118\consts.h \
	c:\cliff\openso~1\1~1.18a\osgps118\globals.h \
	c:\cliff\openso~1\1~1.18a\osgps118\gpsfuncs.h \
	c:\cliff\openso~1\1~1.18a\osgps118\gp2021.h


GPSFUNCS_DEP = c:\cliff\openso~1\1~1.18a\osgps118\cmatrix3.h


GPSFUNCS_DEP = c:\cliff\openso~1\1~1.18a\osgps118\gp2021.h \
	c:\cliff\openso~1\1~1.18a\osgps118\consts.h \
	c:\cliff\openso~1\1~1.18a\osgps118\structs.h \
	c:\cliff\openso~1\1~1.18a\osgps118\globals.h \
	c:\cliff\openso~1\1~1.18a\osgps118\gpsfuncs.h


NMEA_DEP = c:\cliff\openso~1\1~1.18a\osgps118\nmea.h \
	c:\cliff\openso~1\1~1.18a\osgps118\consts.h \
	c:\cliff\openso~1\1~1.18a\osgps118\structs.h \
	c:\cliff\openso~1\1~1.18a\osgps118\globals.h \
	c:\cliff\openso~1\1~1.18a\osgps118\fwinter.h \
	c:\cliff\openso~1\1~1.18a\osgps118\serport.h


all:	$(PROJ).EXE $(PROJ).BSC

PCIFIND.OBJ:	PCIFIND.C $(PCIFIND_DEP)
	$(CC) $(CFLAGS) $(CCREATEPCHFLAG) /c PCIFIND.C

CMATRIX3.OBJ:	CMATRIX3.CPP $(CMATRIX3_DEP)
	$(CPP) $(CFLAGS) $(CPPCREATEPCHFLAG) /c CMATRIX3.CPP

FWINTER.OBJ:	FWINTER.C $(FWINTER_DEP)
	$(CC) $(CFLAGS) $(CUSEPCHFLAG) /c FWINTER.C

GP2021.OBJ:	GP2021.C $(GP2021_DEP)
	$(CC) $(CFLAGS) $(CUSEPCHFLAG) /c GP2021.C

GPSISR.OBJ:	GPSISR.C $(GPSISR_DEP)
	$(CC) $(CFLAGS) $(CUSEPCHFLAG) /c GPSISR.C

GPSRCVR.OBJ:	GPSRCVR.C $(GPSRCVR_DEP)
	$(CC) $(CFLAGS) $(CUSEPCHFLAG) /c GPSRCVR.C

NAV_FIX.OBJ:	NAV_FIX.C $(NAV_FIX_DEP)
	$(CC) $(CFLAGS) $(CUSEPCHFLAG) /c NAV_FIX.C

SERPORT.OBJ:	SERPORT.C $(SERPORT_DEP)
	$(CC) $(CFLAGS) $(CUSEPCHFLAG) /c SERPORT.C

GPSFUNCS.OBJ:	GPSFUNCS.CPP $(GPSFUNCS_DEP)
	$(CPP) $(CFLAGS) $(CPPUSEPCHFLAG) /c GPSFUNCS.CPP

NMEA.OBJ:	NMEA.CPP $(NMEA_DEP)
	$(CPP) $(CFLAGS) $(CPPUSEPCHFLAG) /c NMEA.CPP

$(PROJ).EXE::	PCIFIND.OBJ CMATRIX3.OBJ FWINTER.OBJ GP2021.OBJ GPSISR.OBJ GPSRCVR.OBJ \
	NAV_FIX.OBJ SERPORT.OBJ GPSFUNCS.OBJ NMEA.OBJ $(OBJS_EXT) $(DEFFILE)
	echo >NUL @<<$(PROJ).CRF
PCIFIND.OBJ +
CMATRIX3.OBJ +
FWINTER.OBJ +
GP2021.OBJ +
GPSISR.OBJ +
GPSRCVR.OBJ +
NAV_FIX.OBJ +
SERPORT.OBJ +
GPSFUNCS.OBJ +
NMEA.OBJ +
$(OBJS_EXT)
$(PROJ).EXE
$(MAPFILE)
C:\MSVC\LIB\+
C:\MSVC\MFC\LIB\+
$(LIBS)
$(DEFFILE);
<<
	link $(LFLAGS) @$(PROJ).CRF

run: $(PROJ).EXE
	$(PROJ) $(RUNFLAGS)


$(PROJ).BSC: $(SBRS)
	bscmake @<<
/o$@ $(SBRS)
<<
