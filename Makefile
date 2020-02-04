# when the following line is uncommented, debug output will be printed.
# DEBUG=-D ENABLE_DEBUG_OUTPUT


# add `-Wno-unused-variable` to make errors about unused variables go away
CFLAGS=-Wall -Wno-psabi -g -Wno-unused-variable -O3

# these are all the object files that need to be generated before the binary can be generated
# 
OBJS=vision_pipeline.o

LIBS=-lpthread -lcurl -lopencv_calib3d -lopencv_imgcodecs -lopencv_core -l opencv_imgproc -lopencv_highgui  -lopencv_videoio ~/ntcore/build/libs/ntcore/static/libntcore.a  ~/ntcore/build/dependencies/wpiutil-cpp/linuxnativearm/linux/nativearm/static/libwpiutil.a 

INC=-I /usr/include/llvm-3.9 -I /usr/include/opencv4 -I /usr/local/include/opencv4 -I ~/ntcore/src/main/native/include/ -I ~/ntcore/build/dependencies/wpiutil-cpp/headers/


GXX=g++ $(CFLAGS) 

.PHONY: all clean run

all: vision_pipeline

run: vision_pipeline
	OPENCV_VIDEOIO_DEBUG=1 OPENCV_LOG_LEVEL=debug ./vision_pipeline

%.o: %.cpp
	$(GXX) $(INC) $(DEBUG) -c -o $@ $<

vision_pipeline: $(OBJS)
	$(GXX) -o $@ $(OBJS) $(LIBS) 

clean:
	rm -f *.o vision_pipeline
