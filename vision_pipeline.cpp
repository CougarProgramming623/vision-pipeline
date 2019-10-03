
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <stdio.h>


int main(int args, char** argss){
    cv::Mat frame;
    //--- INITIALIZE VIDEOCAPTURE
    cv::VideoCapture cap;
    // open the default camera using default API
    // cap.open(0);
    // OR advance usage: select any API backend
    int deviceID = 0;             // 0 = open default camera
    int apiID = cv::CAP_ANY;      // 0 = autodetect default API
    // open selected camera using selected API
    cap.open(deviceID + apiID);
    // check if we succeeded
    if (!cap.isOpened()) {
        std::cerr << "ERROR! Unable to open camera\n";
        return -1;
    }
    frame.set(CAP_PROP_FRAME_WIDTH,960);
    frame.set(CAP_PROP_FRAME_HEIGHT,544);
/*


# Full resolution for the MS LifeCam HD-3000 is 1280x720
RESOLUTION="-r 960x544"
FPS="-f 20"
YUYV="-y"
QUALITY="-q 10" # 0 .. 10
CONTRAST="-co 5" #0 .. 10
SHARPNESS="-sh 50" # 0 .. 50
COLORBALANCE="-cb 90"
WHITEBALANCE="-wb 2800" # 2800 .. 10000
# Possible exposure values for the Microsoft LifeCam HD-3000/5000:
# 5, 9-10 (same exposure?), 19-20 (same exposure?) 39, 78, 156, 312, 625, 1250,
# 2500, 5000, 10000
# (per http://comments.gmane.org/gmane.linux.drivers.uvc.devel/5717)
EXPOSURE="-ex 9"
BACKLIGHT="-bk 0"
CHROMAGAIN="-cagc 2800" # 2800 .. 10000


*/



    //--- GRAB AND WRITE LOOP
    std::cout << "Start grabbing" << std::endl
        << "Press any key to terminate" << std::endl;
    for (;;)
    {
        // wait for a new frame from camera and store it into 'frame'
        cap.read(frame);
        // check if we succeeded
        if (frame.empty()) {
            std::cerr << "ERROR! blank frame grabbed\n";
            break;
        }
        // show live and wait for a key with timeout long enough to show images
//        imshow("Live", frame);
        // std::cout << "cols:" << frame.cols << "yeo" << std::endl;
        // std::cout << "is empty : " << frame.empty() << "" << std::endl;
        for (int r = 0;r<frame.rows;r++){
          std::cout << "[";
          for (int c = 0;c<frame.cols;c++){
            uint8_t green = frame.data[r*frame.cols*frame.channels() + c*frame.channels() + 1];
            printf("%d,",green);
            break;
          }
          std::cout << "]";
          break;
        }
        std::cout << std::endl;
//        if (cv::waitKey(5) >= 0)
//            break;
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}
