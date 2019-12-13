CFLAGS=-Werror -Wall -Wno-psabi -g
OBJS=callibration.o
LIBS=-lopencv_calib3d -lopencv_features2d -lopencv_videoio -lopencv_imgcodecs -lopencv_core -l opencv_imgproc -lopencv_highgui -L /usr/local/lib

INC=-I /usr/local/include/opencv4 -I /usr/include/opencv4


GXX=g++ $(CFLAGS) $(INC)

.PHONY: all clean run

all: callibration

run: callibration
	./callibration

%.o: %.cpp
	$(GXX) -c -o $@ $< 

callibration: $(OBJS)
	$(GXX) -o $@ $(OBJS) $(LIBS) 

clean:
	rm -f *.o callibration
