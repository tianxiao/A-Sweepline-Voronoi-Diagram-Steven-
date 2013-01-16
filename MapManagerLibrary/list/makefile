##
## GridMap makefile
##

#############################################################

CMP 	= mingw32-gcc.exe

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
all: $(SLIST)PointXYZList.o $(SLIST)SosList.o
	touch all

$(SLIST)PointXYZList.o: $(SLIST)PointXYZList.cpp  $(SLIST)PointXYZList.h $(SLIST)makefile
	$(CMP) $(CFLAGS) -c $(SLIST)PointXYZList.cpp $(INCLUDE) -o $(SLIST)PointXYZList.o		

$(SLIST)SosList.o: $(SLIST)SosList.cpp  $(SLIST)SosList.h $(SLIST)makefile
	$(CMP) $(CFLAGS) -c $(SLIST)SosList.cpp $(INCLUDE) -o $(SLIST)SosList.o		

clean:
	/bin/rm -f *.o
	/bin/rm -f *~

backup: 
	/bin/cp -R /usr/local/Saphira /users/workarea
#/bin/cp $(OBJD)*.cpp /users/workarea/ME85
#/bin/cp $(OBJD)*.h /users/workarea/ME85	
#/bin/cp $(GMAP)*.cpp /users/workarea/gridmap
#/bin/cp $(GMAP)*.h /users/workarea/gridmap





