CFLAGS=-Werror -Wall -Wno-psabi -g
OBJS=vision_pipeline.o
LIBS=-lcurl -lopencv_imgcodecs -lopencv_core -l opencv_imgproc -lopencv_highgui -lopencv_videoio -L /usr/local/lib

INC=-I /usr/include/llvm-3.9 -I /usr/include/opencv4

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
