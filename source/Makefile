CC      = g++
CFLAGS  = -Wall `pkg-config --cflags opencv`
RM      = rm -f


default: all

all: Shark

Shark: shark.cpp
	    $(CC) $(CFLAGS) -lwiringPi shark.cpp `pkg-config --libs opencv` -o Shark

clean veryclean:
	    $(RM) Shark
