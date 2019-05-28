CC      = g++
CFLAGS  = -Wall
RM      = rm -f


default: all

all: Shark

Shark: shark.cpp
	    $(CC) $(CFLAGS) -lwiringPi -o Shark shark.cpp

clean veryclean:
	    $(RM) Shark
