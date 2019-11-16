#include "tuple"
#include "opencv2/core.hpp"
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/calib3d.hpp"
#include <iostream>
#include <stdio.h>
#include "math.h"

#define SET_WITH_CHECK(STATEMENT) \
    if(!STATEMENT){ \
      std::cerr << #STATEMENT << " is not supported by the video backend" << std::endl; \
    };  

void findContours(cv::Mat &, bool , std::vector<std::vector<cv::Point> > &);
void filterContours(std::vector<std::vector<cv::Point> > &, double , double , double , double , double , double , double [], double , double , double , double , std::vector<std::vector<cv::Point> > &);


//std::tuple<double,double,double>
cv::Mat findPos(cv::Mat rvec,cv::Mat tvec,int x1, int y1){
    double xCameraF[] = {static_cast<double>(x1),static_cast<double>(y1),0.0};
//    printf("tvec type: %d\n",tvec.type());
    cv::Mat XCamera(3, 1, 6, xCameraF); // CV_64F 
    cv::Mat rotMat;
    cv::Rodrigues(rvec, rotMat);
    cv::Mat tempWorld = XCamera - tvec;
//    printf("succ. sub. Type: %d\n", tempWorld.type());
    cv::Mat XWorld = rotMat.t() * (XCamera - tvec);
//    printf("XWorld type: %d\n",XWorld.type());
//    printf("XWorld:");
    //std::cout << XWorld << std::endl;
//    cv XCamera - tvec 
    return XWorld;
}

// calling this with points.size < 1 will make me sad
cv::Point findExtreme(std::vector<cv::Point> points, bool isX, bool isMax) {
    cv::Point extreme = points[0];
    for(long unsigned int i = 1; i < points.size(); i++){
        cv::Point p = points[i];
        int current = 0;
        int newVal  = 0;
        if(isX) {
            current = extreme.x;
            newVal = p.x;
        } else {
            current = extreme.y;
            newVal = p.y;
        }
        if(isMax && newVal < current) { // the origin is at the top left corrner, lower values are "higher"
            extreme = p;
        } else if(!isMax && newVal > current){ // higher values are lower, the biggest value is the lowest point (we hope)
            extreme = p;
        }
    }
    return extreme;
}
std::vector<cv::Point> contoursToPoints(std::vector<std::vector<cv::Point>> points){
   std::vector<cv::Point> cl = points[0]; // the first contour, the one that's on the left
   std::vector<cv::Point> cr = points[1]; // the second contour, the one that's on the right
   // let's assume that c1 is on the left

   std::vector<cv::Point> r; // the return vertex
   

   r.push_back(findExtreme(cl,false,false)); // left top point

   r.push_back(findExtreme(cr,false,false)); // right top point
   r.push_back(findExtreme(cr,true ,true )); // right right point
   r.push_back(findExtreme(cr,false, true));; // right bottom point


   r.push_back(findExtreme(cr,false,true )); // left bottom
   r.push_back(findExtreme(cr,true, false)); // left left 


   return r;
}

#define TARGET_WIDTH 2.0f
#define TARGET_HEIGHT 5.5f 
#define TARGET_UPPER_OFFSET 4
#define TARGET_ROTATION 14.5f
#define PI 3.14159265

cv::Point3f flip(cv::Point3f point){
    return cv::Point3f(-1 * point.x,point.y,point.z);
}
void generateConstants(){
    double cosine = cos(TARGET_ROTATION * PI / 180.0);
    double sine   = sin(TARGET_ROTATION * PI / 180.0);
    
    //std::vector<cv::Point3f> rightTarget;
    float manipulator[2];
    manipulator[0] = TARGET_UPPER_OFFSET;
    manipulator[1] = 0;
    cv::Point3f right2 = cv::Point3f(manipulator[0], manipulator[1], 0.0f); // point 2
    manipulator[0] += TARGET_HEIGHT  * sine;
    manipulator[1] -= TARGET_HEIGHT  * cosine;
    cv::Point3f right3 = cv::Point3f(manipulator[0], manipulator[1], 0.0f); // point 3
    manipulator[0] -= TARGET_WIDTH * sine;
    manipulator[1] -= TARGET_WIDTH * cosine;
    cv::Point3f right4 = cv::Point3f(manipulator[0], manipulator[1], 0.0f); // point 4
    manipulator[0] += TARGET_HEIGHT * sine;
    manipulator[1] -= TARGET_HEIGHT * cosine;
    cv::Point3f rightX = cv::Point3f(manipulator[0],manipulator[1], 0.0f); // point X
    
    std::cout << "right2 " << right2 << std::endl;
    std::cout << "right3 " << right3 << std::endl;
    std::cout << "right4 " << right4 << std::endl;
    std::cout << "rightX " << rightX << std::endl;


    
    std::vector<cv::Point3f> fullTarget;
    fullTarget.push_back(flip(right2)); // point 1
    fullTarget.push_back(right2); //       point 2
    fullTarget.push_back(right3); //       point 3
    fullTarget.push_back(right4); //       point 4
    fullTarget.push_back(flip(right4)); // point 5
    fullTarget.push_back(flip(right3)); // point 6

    int point = 1;
    for(cv::Point3f x : fullTarget){
        std::cout << "point " << point << "  :  " << x << std::endl;
        point++;
    }
}




int main(int args, char** argss){
    std::cout << "" <<
       "=========" << std::endl <<
       " Vision Pipeline, 2019" << std::endl <<      
       " Contributors: Carson Graham" << std::endl <<
       "=========" << std::endl;
    generateConstants();

    std::cout << "premature return" << std::endl;
    return 0;

    std::cout << "readnig from param.yaml" << std::endl;

    cv::FileStorage fs;
    fs.open("param.yaml",cv::FileStorage::READ);
    cv::Mat cameraMatrix = fs["camera_matrix"].mat();// 6: CV_64F
    
	std::vector<cv::Point3f> objectPoints;
    fs["grid_points_real"] >> objectPoints;
    // std::cout << objectPoints;
    std::vector<cv::Point2f> imagePoints;
    fs["image_points_real"] >> imagePoints;
  
    //cv::Mat imagePoints = fs["image_points"].mat(); // 13: CV_32F
    cv::Mat distCoeff = fs["distortion_coefficients"].mat(); // 6: CV_64F  
   
    //cv::Mat testImagePoints(imagePoints);
    //printf("test type:%d\n",testImagePoints.type());
    //imagePoints = testImagePoints; 
     std::cout << "obj " << objectPoints << std::endl << "img " << imagePoints << std::endl; 
    cv::Mat opoints = ((cv::InputArray)objectPoints).getMat();
    printf("opoints:%d\n",opoints.type());// 21: CV_32FC3
    cv::Mat ipoints = ((cv::InputArray)imagePoints).getMat();
    int npoints = std::max(opoints.checkVector(3,/*CV_32F*/5), opoints.checkVector(3, /*CV_64F*/6));
    bool a = (npoints >= 4);
    bool b = false;
    bool c = npoints == std::max(ipoints.checkVector(2,/*CV_32F*/5), ipoints.checkVector(2,/*CV_64F*/6));

    std::cout << "a:" << a << " b:" << b <<  " c:" << c << std::endl;
    
    printf("opoints.checkVector(3,5): %d\n",opoints.checkVector(3,5));
    printf("opoints.checkVector(3,6): %d\n",opoints.checkVector(3,6));
    printf("npoints:%d\n",npoints);
    printf("ipoints.checkVector(2,5): %d\n",ipoints.checkVector(2,5));
    printf("ipoints.checkVector(2,6): %d\n",ipoints.checkVector(2,6)); 
    printf("opoints -- rows:%d col:%d\n",opoints.rows,opoints.cols);
    printf("ipoints -- rows:%d col:%d\n",ipoints.rows,ipoints.cols);
    printf("cameraMatrix: %d iPoints: %d distCoeff: %d opoints: %d\n",
            cameraMatrix.type(), ipoints.type(), distCoeff.type(),opoints.type());

    //cv::Mat rvec2;
    //cv::Mat tvec2;
    //bool no = cv::solvePnP(objectPoints,imagePoints,cameraMatrix,distCoeff,rvec2,tvec2);

    //std::cout << "no: " << no << std::endl;
    //std::cout << "rvec2: " << rvec2 << std::endl;
    //std::cout << "tvec2: " << tvec2 << std::endl;
   
    //findPos(rvec2,tvec2,0,0);
    //printf("distance: %f\nangle1:%f\nangle2:%f\n",
    //        std::get<0>(out),std::get<1>(out),std::get<2>(out)); 
    // return 0;
     // YAY!
    
    //std::cout << imagePoints;
    //std::cout << distCoeff;



    cv::Mat frame;
    //--- INITIALIZE VIDEOCAPTURE
    cv::VideoCapture cap;
    // open the default camera using default API
    // cap.open(0);
    // OR advance usage: select any API backend
    int deviceID = 0;             // 0 = open default camera
    int apiID = cv::CAP_V4L2;      // 0 = autodetect default API


    cap.open(deviceID + apiID);
    std::cout << "api: " << apiID << std::endl;
    // set up the camera
    std::cout << "setting up camera" << std::endl;
    SET_WITH_CHECK(cap.set(cv::CAP_PROP_FRAME_WIDTH,288));
    SET_WITH_CHECK(cap.set(cv::CAP_PROP_FRAME_HEIGHT,352));
    SET_WITH_CHECK(cap.set(cv::CAP_PROP_FPS,20));
   // cap.set(cv::CAP_PROP_CONVERT_RGB,0);
    SET_WITH_CHECK(cap.set(cv::VIDEOWRITER_PROP_QUALITY,10));
    SET_WITH_CHECK(cap.set(cv::CAP_PROP_CONTRAST,5));
    SET_WITH_CHECK(cap.set(cv::CAP_PROP_SHARPNESS,50));
    // missing - color balance
    // not found? commenting out bc we don't have cb either // cap.set(cv::CAP_PROP_WB_TEMPERATURE,2800);    
    SET_WITH_CHECK(cap.set(cv::CAP_PROP_AUTO_EXPOSURE,1)); 
    SET_WITH_CHECK(cap.set(cv::CAP_PROP_EXPOSURE,9));
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
   //     printf("reading frame...\n");  
      // wait for a new frame from camera and store it into 'frame'
        cap.read(frame);
        // check if we succeeded
        if (frame.empty()) {
            std::cerr << "ERROR! blank frame grabbed\n";
            //break;
            continue;
        }
 //       printf("channels: %d\ntype: %d\n",frame.channels(),frame.type());
 //       std::cout << std::endl;
//        continue;
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
         frame.release();
 //        cv::Vec3b c = img_hsv.at<cv::Vec3b>(100,100);
 //        printf("%d:%d:%d",c[0],c[1],c[2]);
         // continue;         
         // mask by hsvThreashold
         double hue[] = {33.99280575539568, 93.99317406143345};
	     double sat[] = {100.89928057553958, 255.0};
         double val[] = {169.69424460431654, 255.0};
         cv::Mat img_filtered;
         cv::inRange(img_hsv,cv::Scalar(hue[0],sat[0],val[0]),cv::Scalar(hue[1],sat[1],val[1]),img_filtered);
         img_hsv.release();
         // ASSERT img_filtered.type() == 0
         // this is a 8-bit integer with 1 channel
         // https://stackoverflow.com/questions/10167534/how-to-find-out-what-type-of-a-mat-object-is-with-mattype-in-opencv
         std::vector<std::vector<cv::Point>> og_contours;
         findContours(img_filtered,false,og_contours);
         img_filtered.release();
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
         // first one
         if( contours.size() == 0){
           printf(" No contours\n");
           continue;
         }
         if( contours.size() == 1){
           printf(" One Contour found\n");
           continue; 
         }

         if( contours.size() != 2){
//          std::tuple<int,int> xy = findContours       
            printf("more then one contour found\n");
            continue;
         }
         
         std::vector<cv::Point> points = contoursToPoints(contours);
//         int x = std::get<0>(xy);
//         int y = std::get<1>(xy);
         std::cout << "(" <<
             points[0]<<","<<points[1]<<","<<points[2]<<","<<points[3] << "  " << 
             points[4]<<","<<points[5]<<","<<points[6]<<","<<points[7];          
//         continue;
         std::cout << ")";
         std::cout << std::endl;
         continue;
         // m a t h
         //cv::Mat rvec;
         //cv::Mat tvec;
         //cv::solvePnP(objectPoints,imagePoints,cameraMatrix,distCoeff,rvec,tvec);
         //std::cout << "yes: " << yes << std::endl;
         //std::cout << "tvec: " << tvec << std::endl;
         //std::cout << "rvec: " << rvec << std::endl;
         //cv::RotatedRect rect = cv::minAreaRect(contours[0]);
         //int x = rect.center.x;
         //int y = rect.center.y;  
//         printf("(%10d,%10d)",x,y);
//         std::cout << std::endl;
//         continue;                            
        //cv::Mat xWorld = findPos(rvec, tvec, x, y);
 //       std::cout << "[" << xWorld.at<double>(0) << 
 //                    "," << xWorld.at<double>(1) << 
 //                    "," << xWorld.at<double>(2) << "]";
       //printf("[%10f,%10f,%10f]",xWorld.at<double>(0),
       //                          xWorld.at<double>(1),
       //                          xWorld.at<double>(2)); 
       //xWorld.release();
       
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
//		cv::Rect bb = boundingRect(contour);
//		if (bb.width < minWidth || bb.width > maxWidth) continue;
//		if (bb.height < minHeight || bb.height > maxHeight) continue;
//		double area = cv::contourArea(contour);
//		if (area < minArea) continue;
//		if (arcLength(contour, true) < minPerimeter) continue;
//		cv::convexHull(cv::Mat(contour, true), hull);
//		double solid = 100 * area / cv::contourArea(hull);
//		if (solid < solidity[0] || solid > solidity[1]) continue;
//       if (contour.size() < minVertexCount || contour.size() > maxVertexCount) 
//            continue;
//        double ratio = (double) bb.width / (double) bb.height;
//        if (ratio < minRatio || ratio > maxRatio) continue;
        output.push_back(contour);
    }
}

