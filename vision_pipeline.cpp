
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <stdio.h>

#define SET_NO_CAP(STATEMENT) std::cout << "Preforming " << #STATEMENT << std::endl; \
    if(!STATEMENT){ \
      std::cerr << #STATEMENT << " is not supported by the video backend" << std::endl; \
    } \
    std::cout << "Done with " << #STATEMENT << std::endl;  

int main(int args, char** argss){
    cv::Mat frame;
    //--- INITIALIZE VIDEOCAPTURE
    cv::VideoCapture cap;
    // open the default camera using default API
    // cap.open(0);
    // OR advance usage: select any API backend
    int deviceID = 0;             // 0 = open default camera
    int apiID = cv::CAP_ANY;      // 0 = autodetect default API


    cap.open(deviceID + apiID);

    // set up the camera
    std::cout << "setting up camera" << std::endl;
    SET_NO_CAP(cap.set(cv::CAP_PROP_FRAME_WIDTH,544));
    SET_NO_CAP(cap.set(cv::CAP_PROP_FRAME_HEIGHT,960));
    SET_NO_CAP(cap.set(cv::CAP_PROP_FPS,20));
//    cap.set(cv::CAP_PROP_FORMAT,cv::CAP_MODE_YUYV); // opencv doesn't like this :(
    SET_NO_CAP(cap.set(cv::VIDEOWRITER_PROP_QUALITY,10));
    SET_NO_CAP(cap.set(cv::CAP_PROP_CONTRAST,5));
    SET_NO_CAP(cap.set(cv::CAP_PROP_SHARPNESS,50));
    // missing - color balance
    // not found? commenting out bc we don't have cb either // cap.set(cv::CAP_PROP_WB_TEMPERATURE,2800);    
    SET_NO_CAP(cap.set(cv::CAP_PROP_AUTO_EXPOSURE,1)); 
    SET_NO_CAP(cap.set(cv::CAP_PROP_EXPOSURE,5));
    // SET_NO_CAP(cap.set(cv::CAP_PROP_BACKLIGHT,0)); -- not really needed, as we control the backlight
    // missing - chromagain
    std::cout << "setting up done" << std::endl;
//    return 1;
    // open selected camera using selected API
    
    // check if we succeeded
    if (!cap.isOpened()) {
        std::cerr << "ERROR! Unable to open camera\n";
        return -1;
    }
    //--- GRAB AND WRITE LOOP
    for (;;)
    {
        // wait for a new frame from camera and store it into 'frame'
        cap.read(frame);
        // check if we succeeded
        if (frame.empty()) {
            std::cerr << "ERROR! blank frame grabbed\n";
            break;
        }
        //printf("channels: %d\ntype: %d\n",frame.channels(),frame.type());
        //std::cout << std::endl;
        //break;
        // show live and wait for a key with timeout long enough to show images
//        imshow("Live", frame);
        // std::cout << "cols:" << frame.cols << " rows: " << frame.rows << std::endl;
        // std::cout << "is empty : " << frame.empty() << "" << std::endl;
         //uint16_t green = frame.at<uint16_t>(0,0);
         //printf("%d",green);
         // printf("Matrix: %d %dx%d \n", frame.type(), frame.cols, frame.rows );
         cv::Mat img_hsv;
         cv::cvtColor(frame,img_hsv,CV_RGB2HSV); 
         cv::Vec3b c = img_hsv.at<cv::Vec3b>(100,100);
         printf("%d:%d:%d ",c[0],c[1],c[2]);
         
         // mask by hsvThreashold
         double hue[] = {33.99280575539568, 93.99317406143345};
	     double sat[] = {100.89928057553958, 255.0};
         double val[] = {169.69424460431654, 255.0};
         cv::Mat img_filtered;
         cv::inRange(img_hsv,cv::Scalar(hue[0],sat[0],val[0]),cv::Scalar(hue[1],sat[1],val[1]),img_filtered);
         // ASSERT img_filtered.type() == 0
         // this is a 8-bit integer with 1 channel
         // https://stackoverflow.com/questions/10167534/how-to-find-out-what-type-of-a-mat-object-is-with-mattype-in-opencv
         // printf("Filtered Matrix: %d\n",img_filtered.type());    
         // print the top row, for debugging
         //for(int i = 0;i < img_filtered.rows; i+=10){
         //  uint8_t maskedPixel = img_filtered.at<cv::Vec3b>(0,i)[0]; // get the first channel - there is only one channel
         //  printf(maskedPixel ? "1" : "0"); 
         //}
         // cv::Mat camera_matrix = cv::getOptimalNewCameraMatrix(0,0,0,0);
         std::cout << std::endl;
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}
