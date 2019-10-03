CC      = g++
CFLAGS  = -Wall `pkg-config --cflags opencv` -lpthread -lwiringPi `pkg-config --libs opencv` -Ilibfacedetection/src 
RM      = rm -f
DEPS    = libfacedetection/src/facedetectcnn.h


default: all

all: Shark

%.o: %.cpp $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

Shark: libfacedetection/src/facedetectcnn-int8data.o libfacedetection/src/facedetectcnn-model.o libfacedetection/src/facedetectcnn.o shark.o
	$(CC) $(CFLAGS) -o Shark libfacedetection/src/facedetectcnn-int8data.o libfacedetection/src/facedetectcnn-model.o libfacedetection/src/facedetectcnn.o shark.o

clean veryclean:
	$(RM) Shark
