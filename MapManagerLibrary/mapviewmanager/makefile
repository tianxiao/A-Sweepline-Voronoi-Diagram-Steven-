##
## MapManager makefile
##

#############################################################

CMP = mingw32-gcc.exe

SRCD 	= ./
OBJD 	= ./
GBLK 	= ../grid/
GMAP 	= ../grid/
CDEF 	= ../commonDefs/
SUTIL 	= ../sosutil/
SLIST 	= ../list/
LOG	= ../logger/
IMG	= ../CxImage/CxImage/
MVMGR	= ../MapViewManager/
DRAW	= ../windowobjects_win32/ #THIS WILL HAVE TO CHANGE FOR LINUX

# check which OS we have 
#include $(INCD)os.h

SHELL = /bin/sh

INCLUDE = -I$(CDEF) -I$(LOG) -I$(SUTIL) -I$(SLIST) -I$(GMAP) -I$(IMG) -I$(DRAW)

#############################################################
all: $(MVMGR)MapViewManager.o
	touch all

$(MVMGR)MapViewManager.o: $(MVMGR)MapViewManager.cpp  $(MVMGR)MapViewManager.h $(MVMGR)makefile
	$(CMP) $(CFLAGS) -c $(MVMGR)MapViewManager.cpp $(INCLUDE) -o $(MVMGR)MapViewManager.o		

clean:
	/bin/rm -f *.o
	/bin/rm -f *~

backup: 
	/bin/cp -R /usr/local/Saphira /users/workarea
#/bin/cp $(OBJD)*.cpp /users/workarea/ME85
#/bin/cp $(OBJD)*.h /users/workarea/ME85	
#/bin/cp $(GMAP)*.cpp /users/workarea/gridmap
#/bin/cp $(GMAP)*.h /users/workarea/gridmap





