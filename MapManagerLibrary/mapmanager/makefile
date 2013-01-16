##
## MapManager makefile
##

#############################################################

CMP = mingw32-gcc.exe
#CMP	= g++

SRCD 	= ./
OBJD 	= ./
GBLK 	= ../grid/
GMAP 	= ../grid/
CDEF 	= ../commonDefs/
SUTIL 	= ../sosutil/
SLIST 	= ../list/
LOG	= ../logger/
USRINC	= /usr/include/
C++INC	= /usr/include/c++/3.3.3/backward/

# check which OS we have 
#include $(INCD)os.h

SHELL = /bin/sh

INCLUDE = -I$(CDEF) -I$(LOG) -I$(SUTIL) -I$(SLIST) -I$(GMAP) -I$(USRINC) -I$(C++INC)

#############################################################
all: $(SRCD)GridMapLayer.o $(SRCD)MapManager.o
	touch all

$(OBJD)GridMapLayer.o: $(SRCD)GridMapLayer.cpp  $(SRCD)GridMapLayer.h $(SRCD)makefile
	$(CMP) $(CFLAGS) -c $(SRCD)GridMapLayer.cpp $(INCLUDE) -o $(SRCD)GridMapLayer.o		

$(OBJD)MapManager.o: $(SRCD)MapManager.cpp  $(SRCD)MapManager.h $(SRCD)makefile
	$(CMP) $(CFLAGS) -c $(SRCD)MapManager.cpp $(INCLUDE) -o $(SRCD)MapManager.o		

clean:
	/bin/rm -f *.o
	/bin/rm -f *~

backup: 
	/bin/cp -R /usr/local/Saphira /users/workarea
#/bin/cp $(OBJD)*.cpp /users/workarea/ME85
#/bin/cp $(OBJD)*.h /users/workarea/ME85	
#/bin/cp $(GMAP)*.cpp /users/workarea/gridmap
#/bin/cp $(GMAP)*.h /users/workarea/gridmap





