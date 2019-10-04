CFLAGS=-Werror -Wall -Wno-psabi -g
OBJS=vision_pipeline.o
LIBS=-lcurl -lopencv_imgcodecs -lopencv_core -l opencv_imgproc -lopencv_highgui -lntcore -lwpiutil -lopencv_videoio -L ~/ntcore/build/libs/ntcore/shared -L ~/wpiutil/build/libs/wpiutil/shared
INC=-I ~/ntcore/src/main/native/include -I ~/wpiutil/src/main/native/include -I /usr/include/llvm-3.9

GXX=g++ $(CFLAGS) $(INC)

.PHONY: all clean run

all: vision_pipeline

run: vision_pipeline
	./vision_pipeline

%.o: %.cpp
	$(GXX) -c -o $@ $<

vision_pipeline: $(OBJS)
	$(GXX) -o $@ $(OBJS) $(LIBS) 

clean:
	rm -f *.o vision_pipeline
