CFLAGS=-Werror -Wall -Wno-psabi -g
OBJS=callibration.o
LIBS=-lcurl -lopencv_calib3d -lopencv_features2d -lopencv_imgcodecs -lopencv_core -l opencv_imgproc -lopencv_highgui -lntcore -lwpiutil -lopencv_videoio -L ~/ntcore/build/libs/ntcore/shared -L ~/wpiutil/build/libs/wpiutil/shared -L /usr/local/lib

INC=-I ~/ntcore/src/main/native/include -I ~/wpiutil/src/main/native/include -I /usr/include/llvm-3.9 -I /usr/local/include/opencv4


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
