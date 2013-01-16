##
## GridBlock makefile
##

#############################################################

CMP = mingw32-gcc.exe

SRCD = ./
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
all: $(LOG)logger.o
	touch all

$(LOG)logger.o: $(LOG)Logger.cpp $(LOG)Logger.h
	$(CMP) $(CFLAGS) -c $(LOG)Logger.cpp $(INCLUDE) -o $(LOG)logger.o


clean:
	/bin/rm -f *.o

backup: 
	/bin/cp -R /usr/local/Saphira /users/workarea






