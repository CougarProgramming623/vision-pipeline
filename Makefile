# when the following line is uncommented, debug output will be printed.
# DEBUG=-D ENABLE_DEBUG_OUTPUT


# add `-Wno-unused-variable` to make errors about unused variables go away
CFLAGS=-Werror -Wall -Wno-psabi -g

# these are all the object files that need to be generated before the binary can be generated
# 
OBJS=vision_pipeline.o

LIBS=-lcurl -lopencv_calib3d -lopencv_imgcodecs -lopencv_core -l opencv_imgproc -lopencv_highgui  -lopencv_videoio -L ~/ntcore/build/libs/ntcore/shared -L ~/wpiutil/build/libs/wpiutil/shared -L /usr/local/lib

INC=-I ~/ntcore/src/main/native/include -I ~/wpiutil/src/main/native/include -I /usr/include/llvm-3.9 -I /usr/include/opencv4 -I /usr/local/include/opencv4


GXX=g++ $(CFLAGS) $(INC)

.PHONY: all clean run

all: vision_pipeline

run: vision_pipeline
	OPENCV_VIDEOIO_DEBUG=1 OPENCV_LOG_LEVEL=debug ./vision_pipeline

%.o: %.cpp
	$(GXX) $(DEBUG) -c -o $@ $<

vision_pipeline: $(OBJS)
	$(GXX) -o $@ $(OBJS) $(LIBS) 

clean:
	rm -f *.o vision_pipeline
