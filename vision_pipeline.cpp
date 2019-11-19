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
/*
 *
 * Things to improve:
 *
 * - use defalt size values for std::vector when the size is going to be the same every time
 *
 *
 */
void findContours(cv::Mat &, bool , std::vector<std::vector<cv::Point> > &);
void filterContours(std::vector<std::vector<cv::Point> > &, double , double , double , double , double , double , double [], double , double , double , double , std::vector<std::vector<cv::Point> > &);

// TODO: test
// returns: x, z, distance, angle1, angle2
std::vector<double> findPos(cv::Mat rvec,cv::Mat tvec){
    double x = tvec.at<double>(0,0); 
    double z = tvec.at<double>(2,0); // there's a chance this should be 0,2
    double distance = sqrt(pow(x,2) + pow(z,2));
    double angle1 = atan2(x,z);
    cv::Mat rotMatrix;
    cv::Rodrigues(rvec,rotMatrix);
    std::cout << rotMatrix.type() << std::endl; // TODO make sure this mat is used with the right types
    cv::Mat worldMat = rotMatrix.inv() * (-tvec);
   
    double angle2 = atan2(worldMat.at<double>(0,0), worldMat.at<double>(2,0));// again with the 0,2 possibility
    
    std::vector<double> retur;
    retur.push_back(x);
    retur.push_back(z);
    retur.push_back(distance);
    retur.push_back(angle1);
    retur.push_back(angle2);
    return retur;
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
#define TARGET_UPPER_OFFSET 4.0f
#define TARGET_ROTATION 14.5f
#define PI 3.14159265

cv::Point3f flip(cv::Point3f point){
       return cv::Point3f(-1 * point.x,point.y,point.z);
}


// generate world cords. This method has been rigorously tested by pasting polygons into desmos and wolfram alpha
// to see if they look like the targets. I don't know 
std::vector<cv::Point3f> generateWorldConstant(){
    std::cout << "generating world constants...";
    double cosine = cos(TARGET_ROTATION * PI / 180.0);
    double sine   = sin(TARGET_ROTATION * PI / 180.0);
    
    //std::vector<cv::Point3f> rightTarget;
    cv::Point3f right1 = cv::Point3f(TARGET_UPPER_OFFSET, 0.0f, 0.0f); // the top-most point. Point 2 of the target
    
    cv::Point3f right2 = cv::Point3f(right1.x + sine   * TARGET_HEIGHT,//the right-most point. Point 3 of the target
                                     right1.y - cosine * TARGET_HEIGHT, 0.0f);

    cv::Point3f right3 = cv::Point3f(right2.x - cosine * TARGET_WIDTH,
                                     right2.y - sine   * TARGET_WIDTH, 0.0f);

    cv::Point3f right4 = cv::Point3f(right3.x - sine   * TARGET_HEIGHT,
                                     right3.y + cosine * TARGET_HEIGHT, 0.0f);
    
    std::cout << "right1 " << right1 << std::endl;
    std::cout << "right2 " << right2 << std::endl;
    std::cout << "right3 " << right3 << std::endl;
    std::cout << "right4 " << right4 << std::endl;


    std::vector<cv::Point3f> fullTarget;
    fullTarget.push_back(flip(right1)); // point 1
    fullTarget.push_back(right1); //       point 2
    fullTarget.push_back(right2); //       point 3
    fullTarget.push_back(right3); //       point 4
    fullTarget.push_back(flip(right3)); // point 5
    fullTarget.push_back(flip(right2)); // point 6
    std::cout << "done" << std::endl;
    printf("Target points in world cords:\n"); 
    int point = 1;
    for(cv::Point3f x : fullTarget){
        std::cout << "point " << point << "  :  " << x << std::endl;
        point++;
    }
    // for easy copy-paste into desmos to make sure it's decent
    
    std::cout << "polygon(";
    for(cv::Point3f x : fullTarget) {
        printf("(%f,%f),",x.x,x.y); // remember to remove the last ,
    }
    std::cout << ")" << std::endl;
    return fullTarget;
}




int main(int args, char** argss){
    std::cout << "" <<
       "=========" << std::endl <<
       " Vision Pipeline, 2019" << std::endl <<      
       " Contributors: Carson Graham" << std::endl <<
       "=========" << std::endl;
    std::vector<cv::Point3f> worldTarget = generateWorldConstant();


    std::cout << "reading from param.yaml" << std::endl;

    cv::FileStorage fs;
    fs.open("param.yaml",cv::FileStorage::READ);
    cv::Mat cameraMatrix = fs["camera_matrix"].mat();// 6: CV_64F
    
    
    cv::Mat distCoeff = fs["distortion_coefficients"].mat(); // 6: CV_64F  
   


    cv::Mat frame; // the next avalible frame is put here
    cv::VideoCapture cap;
    
    int deviceID = 0;             // 0 = open default camera
    int apiID = cv::CAP_V4L2;      // 0 = autodetect default API


    cap.open(deviceID + apiID);
    std::cout << "api in use: " << apiID << std::endl;
    // set up the camera
    std::cout << "setting up camera" << std::endl;
    SET_WITH_CHECK(cap.set(cv::CAP_PROP_FRAME_WIDTH,288));
    SET_WITH_CHECK(cap.set(cv::CAP_PROP_FRAME_HEIGHT,352));
    SET_WITH_CHECK(cap.set(cv::CAP_PROP_FPS,20));
   // cap.set(cv::CAP_PROP_CONVERT_RGB,0);
    SET_WITH_CHECK(cap.set(cv::VIDEOWRITER_PROP_QUALITY,10));
    SET_WITH_CHECK(cap.set(cv::CAP_PROP_CONTRAST,5));
    SET_WITH_CHECK(cap.set(cv::CAP_PROP_SHARPNESS,50));
    SET_WITH_CHECK(cap.set(cv::CAP_PROP_AUTO_EXPOSURE,1)); 
    SET_WITH_CHECK(cap.set(cv::CAP_PROP_EXPOSURE,9));
    std::cout << "camera setup done" << std::endl;
    
    // check if we succeeded
    if (!cap.isOpened()) {
        std::cerr << "ERROR! Unable to open camera\n";
        return -1;
    }

  
    // global loop, never terminates
    for (;;){
      // wait for a new frame from camera and store it into 'frame'
      // this is a blocking call
        cap.read(frame);
        // check if we succeeded
        if (frame.empty()) {
            std::cerr << "ERROR! blank frame grabbed\n";
            //break;
            continue;
        }
 //       printf("channels: %d\ntype: %d\n",frame.channels(),frame.type());
        ;
        // std::cout << "cols:" << frame.cols << " rows: " << frame.rows << std::endl;
         cv::Mat img_hsv;
         cv::cvtColor(frame,img_hsv,cv::COLOR_RGB2HSV); 
         frame.release(); // release it from memory, saving memory

         double hue[] = {33.99280575539568, 93.99317406143345}; // these are the hue values it must fall between
	     double sat[] = {100.89928057553958, 255.0}; // sateration is just "amount". Means we need a lot of green
         double val[] = {169.69424460431654, 255.0};
         cv::Mat img_filtered;
         cv::inRange(img_hsv,cv::Scalar(hue[0],sat[0],val[0]),cv::Scalar(hue[1],sat[1],val[1]),img_filtered);
         img_hsv.release();
         // ASSERT img_filtered.type() == 0
         // this is a 8-bit integer with 1 channel
         // https://stackoverflow.com/questions/10167534/how-to-find-out-what-type-of-a-mat-object-is-with-mattype-in-opencv
         std::vector<std::vector<cv::Point>> og_contours;// original contours
         findContours(img_filtered,false,og_contours);
         img_filtered.release();
         // filter contours
         std::vector<std::vector<cv::Point>> contours;
         double filterContoursMinArea = 0;  
         double filterContoursMinPerimeter = 100.0;  
         double filterContoursMinWidth = 0;  
         double filterContoursMaxWidth = 1000;  
         double filterContoursMinHeight = 0;  
         double filterContoursMaxHeight = 1000;  
         double filterContoursSolidity[] = {0, 100};
         double filterContoursMaxVertices = 1000000;  
         double filterContoursMinVertices = 0;  
         double filterContoursMinRatio = 0;  
         double filterContoursMaxRatio = 1000;
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
         
         // TODO make sure it still works when it can see more then one contour
         if( contours.size() == 0){
           printf(" No contours\n");
           continue;
         }
         if( contours.size() == 1){
           printf(" One Contour found\n");
           continue; 
         }

         if( contours.size() != 2){
             printf("more then one contour found\n");
            continue;
         }
         
         std::vector<cv::Point> points = contoursToPoints(contours);
         std::cout << "(" <<
             points[0]<<","<<points[1]<<","<<points[2]<<","<<points[3] << "  " << 
             points[4]<<","<<points[5]<<","<<points[6]<<","<<points[7];          
//         continue;
         std::cout << ")";
         std::cout << std::endl;
         continue;
         // m a t h
         cv::Mat rvec;
         cv::Mat tvec;
         bool solvePnPSucc = cv::solvePnP(worldTarget,points,cameraMatrix,distCoeff,rvec,tvec);
         std::cout << "solvePnPSucc: " << solvePnPSucc << std::endl;
         std::cout << "tvec: " << tvec << std::endl;
         std::cout << "rvec: " << rvec << std::endl;
         // TODO make sure that the assumptions made about the types of these mats is correct 
         std::cout << "tvec type: " << tvec.type() << std::endl;
         std::cout << "rvec type: " << rvec.type() << std::endl;
         std::vector<double> pos = findPos(rvec, tvec);
         double x        = pos[0];
         double z        = pos[1];
         double distance = pos[2];
         double angle1   = pos[3]; 
         double angle2   = pos[4];

         printf("x:%10f z:%10f distance: %10f angle1: %10f angle2: %10f",x,z,distance,angle1,angle2);
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

