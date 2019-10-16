
#include "opencv2/core.hpp"
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/calib3d.hpp"
#include <iostream>
#include <stdio.h>

#define SET_WITH_CHECK(STATEMENT) \
    if(!STATEMENT){ \
      std::cerr << #STATEMENT << " is not supported by the video backend" << std::endl; \
    };  

void findContours(cv::Mat &, bool , std::vector<std::vector<cv::Point> > &);
void filterContours(std::vector<std::vector<cv::Point> > &, double , double , double , double , double , double , double [], double , double , double , double , std::vector<std::vector<cv::Point> > &);


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
    SET_WITH_CHECK(cap.set(cv::CAP_PROP_FRAME_WIDTH,544));
    SET_WITH_CHECK(cap.set(cv::CAP_PROP_FRAME_HEIGHT,960));
    SET_WITH_CHECK(cap.set(cv::CAP_PROP_FPS,20));
//    cap.set(cv::CAP_PROP_FORMAT,cv::CAP_MODE_YUYV); // opencv doesn't like this :(
    SET_WITH_CHECK(cap.set(cv::VIDEOWRITER_PROP_QUALITY,10));
    SET_WITH_CHECK(cap.set(cv::CAP_PROP_CONTRAST,5));
    SET_WITH_CHECK(cap.set(cv::CAP_PROP_SHARPNESS,50));
    // missing - color balance
    // not found? commenting out bc we don't have cb either // cap.set(cv::CAP_PROP_WB_TEMPERATURE,2800);    
    SET_WITH_CHECK(cap.set(cv::CAP_PROP_AUTO_EXPOSURE,1)); 
    SET_WITH_CHECK(cap.set(cv::CAP_PROP_EXPOSURE,5));
    // SET_WITH_CHECK(cap.set(cv::CAP_PROP_BACKLIGHT,0)); -- not really needed, as we control the backlight
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
        printf("channels: %d\ntype: %d\n",frame.channels(),frame.type());
        std::cout << std::endl;
        //break;
        // show live and wait for a key with timeout long enough to show images
//        imshow("Live", frame);
        // std::cout << "cols:" << frame.cols << " rows: " << frame.rows << std::endl;
        // std::cout << "is empty : " << frame.empty() << "" << std::endl;
         //uint16_t green = frame.at<uint16_t>(0,0);
         //printf("%d",green);
         // printf("Matrix: %d %dx%d \n", frame.type(), frame.cols, frame.rows );
         cv::Mat img_hsv;
         cv::cvtColor(frame,img_hsv,cv::COLOR_RGB2HSV); 
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
         std::vector<std::vector<cv::Point>> og_contours;
         findContours(img_filtered,false,og_contours);
         // filter contours
         std::vector<std::vector<cv::Point>> contours;
         double filterContoursMinArea = 0;  // default Double
         double filterContoursMinPerimeter = 100.0;  // default Double
         double filterContoursMinWidth = 0;  // default Double
         double filterContoursMaxWidth = 1000;  // default Double
         double filterContoursMinHeight = 0;  // default Double
         double filterContoursMaxHeight = 1000;  // default Double
         double filterContoursSolidity[] = {0, 100};
         double filterContoursMaxVertices = 1000000;  // default Double
         double filterContoursMinVertices = 0;  // default Double
         double filterContoursMinRatio = 0;  // default Double
         double filterContoursMaxRatio = 1000;  // default Double
         filterContours(og_contours, 
                        filterContoursMinArea,
                        filterContoursMinPerimeter,
                        filterContoursMinWidth,
                        filterContoursMaxWidth,
                        filterContoursMinHeight,
                        filterContoursMaxHeight,
                        filterContoursSolidity,
                        filterContoursMaxVertices,
                        filterContoursMinVertices,
                        filterContoursMinRatio,
                        filterContoursMaxRatio, contours);
         //for( uint8_t i = 0; i < contours.size(); i++){
             //cv::RotatedRect rectangle = cv::minAreaRect(contours[i]); // for testing
             //rectangle.size;
             //printf("%fx%frect turned %f degrees at %fx%f",
             //       rectangle.size.width,rectangle.size.height,
             //       rectangle.angle,
             //       rectangle.center.x, rectangle.center.y);
         //}



         // m a t h
         continue; 
         // actually, read from the conf file first
         cv::FileStorage fs;
         fs.open("param.yaml",cv::FileStorage::READ);
         cv::Mat cameraMatrix = fs["camera_matrix"].mat(); 
         continue;
         cv::Mat objectPoints = fs["grid_points"].mat();
         cv::Mat imagePoints = fs["image_points"].mat();
         cv::Mat distCoeff = fs["distortion_coefficients"].mat();
         
         cv::Mat rvec;
         cv::Mat tvec;
         continue;
         bool yes = cv::solvePnP(objectPoints,imagePoints,cameraMatrix,distCoeff,rvec,tvec);
         std::cout << yes;
         std::cout << tvec;  



         std::cout << std::endl;
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}







// these functions are, I assume, written by GRIP.
//    as they have documentation


/**
 * Finds contours in an image.
 *
 * @param input The image to find contours in.
 * @param externalOnly if only external contours are to be found.
 * @param contours vector of contours to put contours in.
 */
void findContours(cv::Mat &input, bool externalOnly, std::vector<std::vector<cv::Point> > &contours) {
	std::vector<cv::Vec4i> hierarchy;
	contours.clear();
	int mode = externalOnly ? cv::RETR_EXTERNAL : cv::RETR_LIST;
	int method = cv::CHAIN_APPROX_SIMPLE;
	cv::findContours(input, contours, hierarchy, mode, method);
}


/**
 * Filters through contours.
 * @param inputContours is the input vector of contours.
 * @param minArea is the minimum area of a contour that will be kept.
 * @param minPerimeter is the minimum perimeter of a contour that will be kept.
 * @param minWidth minimum width of a contour.
 * @param maxWidth maximum width.
 * @param minHeight minimum height.
 * @param maxHeight  maximimum height.
 * @param solidity the minimum and maximum solidity of a contour.
 * @param minVertexCount minimum vertex Count of the contours.
 * @param maxVertexCount maximum vertex Count.
 * @param minRatio minimum ratio of width to height.
 * @param maxRatio maximum ratio of width to height.
 * @param output vector of filtered contours.
 */
void filterContours(std::vector<std::vector<cv::Point> > &inputContours, double minArea, double minPerimeter, double minWidth, double maxWidth, double minHeight, double maxHeight, double solidity[], double maxVertexCount, double minVertexCount, double minRatio, double maxRatio, std::vector<std::vector<cv::Point> > &output) {
	std::vector<cv::Point> hull;
	output.clear();
	for (std::vector<cv::Point> contour: inputContours) {
		cv::Rect bb = boundingRect(contour);
		if (bb.width < minWidth || bb.width > maxWidth) continue;
		if (bb.height < minHeight || bb.height > maxHeight) continue;
		double area = cv::contourArea(contour);
		if (area < minArea) continue;
		if (arcLength(contour, true) < minPerimeter) continue;
		cv::convexHull(cv::Mat(contour, true), hull);
		double solid = 100 * area / cv::contourArea(hull);
		if (solid < solidity[0] || solid > solidity[1]) continue;
		if (contour.size() < minVertexCount || contour.size() > maxVertexCount)	continue;
		double ratio = (double) bb.width / (double) bb.height;
		if (ratio < minRatio || ratio > maxRatio) continue;
		output.push_back(contour);
	}
}

