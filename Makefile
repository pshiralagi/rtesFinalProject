INCLUDE_DIRS = 
LIB_DIRS = 
CC=g++

CDEFS=
CFLAGS= -O0 -pg -g $(INCLUDE_DIRS) $(CDEFS)
LIBS= -lrt -lpthread -lpigpio
CPPLIBS= -L/usr/lib -lopencv_core -lopencv_flann -lopencv_video

HFILES= 
CPPFILES= finalProject.cpp alarm.cpp

SRCS= ${HFILES} ${CFILES}
CPPOBJS= ${CPPFILES:.cpp=.o}

all:	finalProject alarm

clean:
	-rm -f *.o *.d
	-rm -f finalProject alarm

finalProject: finalProject.o
	$(CC) $(LDFLAGS) $(CFLAGS) $(LIBS) -o $@ $@.o `pkg-config --libs opencv` $(CPPLIBS)

alarm: alarm.o
	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $@.o -lrt

depend:

.c.o:
	$(CC) $(CFLAGS) -c $<

.cpp.o:
	$(CC) $(CFLAGS) -c $<
