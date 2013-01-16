##
## FileParsers makefile
##

#############################################################

CMP = mingw32-gcc.exe

SRCD 	= ./
OBJD 	= ./
GBLK 	= ../grid/
GMAP 	= ../grid/
CDEF 	= ../commonDefs/
MUI 	= ../MapUpdateInterface/
SUTIL 	= ../sosutil/
SLIST 	= ../list/
LOG	= ../logger/

# check which OS we have 
#include $(INCD)os.h

SHELL = /bin/sh

INCLUDE = -I$(CDEF) -I$(LOG) -I$(SUTIL) -I$(SLIST) -I$(GMAP)

#############################################################
all: $(SRCD)StageWorldFileWriter.o $(SRCD)StageWorldFileParser.o $(SRCD)CarmenTranslator.o
	touch all

$(OBJD)StageWorldFileWriter.o: $(SRCD)StageWorldFileWriter.cpp  $(SRCD)StageWorldFileWriter.h 
	$(CMP) $(CFLAGS) -c $(SRCD)StageWorldFileWriter.cpp $(INCLUDE) -o $(SRCD)StageWorldFileWriter.o		

$(OBJD)StageWorldFileParser.o: $(SRCD)StageWorldFileParser.cpp  $(SRCD)StageWorldFileParser.h $(SRCD)makefile
	$(CMP) $(CFLAGS) -c $(SRCD)StageWorldFileParser.cpp $(INCLUDE) -o $(SRCD)StageWorldFileParser.o		

$(OBJD)CarmenTranslator.o: $(SRCD)CarmenTranslator.cpp  $(SRCD)CarmenTranslator.h $(SRCD)makefile
	$(CMP) $(CFLAGS) -c $(SRCD)CarmenTranslator.cpp $(INCLUDE) -o $(SRCD)CarmenTranslator.o
	
clean:
	/bin/rm -f *.o
	/bin/rm -f *~

backup: 
	/bin/cp -R /usr/local/Saphira /users/workarea
#/bin/cp $(OBJD)*.cpp /users/workarea/ME85
#/bin/cp $(OBJD)*.h /users/workarea/ME85	
#/bin/cp $(GMAP)*.cpp /users/workarea/gridmap
#/bin/cp $(GMAP)*.h /users/workarea/gridmap





