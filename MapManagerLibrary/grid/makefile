##
## GridMap makefile
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

# check which OS we have 
#include $(INCD)os.h

SHELL = /bin/sh

INCLUDE = -I$(CDEF) -I$(LOG) -I$(SUTIL) -I$(SLIST) 

#############################################################
all: $(GMAP)GridMap.o $(GMAP)Grid3D.o $(GBLK)GridBlock.o
	touch all

$(GMAP)GridMap.o: $(GMAP)GridMap.cpp  $(GMAP)GridMap.h $(GMAP)makefile
	$(CMP) $(CFLAGS) -c $(GMAP)GridMap.cpp $(INCLUDE) -o $(GMAP)GridMap.o		

$(OBJD)Grid3D.o: $(SRCD)Grid3D.cpp  $(SRCD)Grid3D.h $(SRCD)makefile
	$(CMP) $(CFLAGS) -c $(SRCD)Grid3D.cpp $(INCLUDE) -o $(SRCD)Grid3D.o
	

$(GBLK)GridBlock.o: $(GBLK)GridBlock.cpp  $(GBLK)GridBlock.h $(GBLK)makefile
	$(CMP) $(CFLAGS) -c $(GBLK)GridBlock.cpp $(INCLUDE) -o $(GBLK)GridBlock.o

clean:
	/bin/rm -f *.o
	/bin/rm -f *~

backup: 
	/bin/cp -R /usr/local/Saphira /users/workarea
#/bin/cp $(OBJD)*.cpp /users/workarea/ME85
#/bin/cp $(OBJD)*.h /users/workarea/ME85	
#/bin/cp $(GMAP)*.cpp /users/workarea/gridmap
#/bin/cp $(GMAP)*.h /users/workarea/gridmap





