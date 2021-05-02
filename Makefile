INCLUDE_DIRS = 
LIB_DIRS = 
CC=g++

CDEFS=
CFLAGS= -O0 -pg -g $(INCLUDE_DIRS) $(CDEFS)
LIBS= -lrt -lpthread -lpigpio
CPPLIBS= -L/usr/lib -lopencv_core -lopencv_flann -lopencv_video

HFILES= 
CPPFILES= finalProject.cpp speaker.cpp

SRCS= ${HFILES} ${CFILES}
CPPOBJS= ${CPPFILES:.cpp=.o}

all:	finalProject speaker

clean:
	-rm -f *.o *.d
	-rm -f finalProject speaker

finalProject: finalProject.o
	$(CC) $(LDFLAGS) $(CFLAGS) $(LIBS) -o $@ $@.o `pkg-config --libs opencv` $(CPPLIBS)

speaker: speaker.o
	$(CC) $(LDFLAGS) $(CFLAGS) -lrt -lpigpio -o $@ $@.o

depend:

.c.o:
	$(CC) $(CFLAGS) -c $<

.cpp.o:
	$(CC) $(CFLAGS) -c $<
