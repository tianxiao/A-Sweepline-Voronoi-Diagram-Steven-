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
all: $(SOSUTIL)SosUtil.o $(SOSUTIL)Threaded.o
	touch all

$(SOSUTIL)SosUtil.o: $(SOSUTIL)SosUtil.cpp $(SOSUTIL)SosUtil.h
	$(CMP) $(CFLAGS) -c $(SOSUTIL)SosUtil.cpp $(INCLUDE) -o $(SOSUTIL)SosUtil.o
	

$(SOSUTIL)Threaded.o: $(SOSUTIL)Threaded.cpp $(SOSUTIL)Threaded.h
	$(CMP) $(CFLAGS) -c $(SOSUTIL)Threaded.cpp $(INCLUDE) -o $(SOSUTIL)Threaded.o
	


clean:
	/bin/rm -f *.o

backup: 
	/bin/cp -R /usr/local/Saphira /users/workarea






