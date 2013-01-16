##
## GridBlock makefile
##

#############################################################

CMP = mingw32-gcc.exe

SRCD = ./
SOSUTIL = ./
OBJD = ./
INCD = ../../ohandler/include/
BIND = ../../bin/
LIBD = ../../lib/
CDEF = ../commonDefs/

LOG	= ../logger/

# check which OS we have 
#include $(INCD)os.h

SHELL = /bin/sh

INCLUDE = -I$(CDEF) -I$(LOG) 
#############################################################
all: $(SOSUTIL)VoronoiDiagramGenerator.o
	touch all

$(SOSUTIL)VoronoiDiagramGenerator.o: $(SOSUTIL)VoronoiDiagramGenerator.cpp $(SOSUTIL)VoronoiDiagramGenerator.h
	$(CMP) $(CFLAGS) -c $(SOSUTIL)VoronoiDiagramGenerator.cpp $(INCLUDE) -o $(SOSUTIL)VoronoiDiagramGenerator.o


clean:
	/bin/rm -f *.o






