#include "tuple"
#include "opencv2/core.hpp"
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/calib3d.hpp"
#include <iostream>
#include <stdio.h>
#include "math.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTable.h"


#define SET_WITH_CHECK(STATEMENT) \
		if(!STATEMENT){ \
				std::cerr << #STATEMENT << " is not supported by the video backend" << std::endl; \
		};  

void findContours(cv::Mat &, bool , std::vector<std::vector<cv::Point> > &);
void filterContours(std::vector<std::vector<cv::Point> > &, double , double , double , double , double , double , double [], double , double , double , double , std::vector<std::vector<cv::Point> > &);

std::vector<double> findPos(cv::Mat rvec,cv::Mat tvec){
		double x = tvec.at<double>(0,0); 
		double z = tvec.at<double>(2,0); // there's a chance this should be 0,2
		double distance = sqrt(pow(x,2) + pow(z,2));
		double angle1 = atan2(x,z);
		cv::Mat rotMatrix;
		cv::Rodrigues(rvec,rotMatrix);
		// TYPE: CV_64F
		
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

// calling this with points.size() < 1 will make me sad
cv::Point findMaxXPoint(std::vector<cv::Point> points, bool wantMax){
    cv::Point extreme = points[0];
    for(long unsigned int i = 1; i < points.size(); i++){
        cv::Point p = points[i];
        if(wantMax && p.x > extreme.x) extreme = p;
        else if (!wantMax && p.x < extreme.x) extreme = p;
    }
    return extreme;
} 

int findMaxY(std::vector<cv::Point> points){
    int max = points[0].y;
    for(long unsigned int i = 1; i < points.size(); i++){
        if(points[i].y > max) max = points[i].y;
    }
    return max;
}

cv::Point closestPoint(int xBase, int yBase, std::vector<cv::Point> points) {
    cv::Point closest = points[0]; 
    float closestDistance = -1;
    for(unsigned int i = 0; i < points.size(); i++){
        cv::Point p = points[i];
        int distanceX = std::abs(xBase - p.x);
        int distanceY = std::abs(yBase - p.y);
        float distance = std::sqrt(std::pow(distanceX,2) + std::pow(distanceY,2));
        if(distance < closestDistance) {
            closestDistance = distance;
            closest = p;
        }
    }
    return closest;
}
//std::vector<cv::Point2f> approx(std::vector<cv::Point2f> points, int backoff){
    
std::vector<cv::Point2f> contoursToPoints(std::vector<cv::Point> points){
    //std::vector<cv::Point> r; // the return vertex
    cv::Point topLeft = findMaxXPoint(points, false);
    cv::Point topRight = findMaxXPoint(points, true); 
    //std::cout << "topLeft: " << topLeft << "\ntopRight: " << topRight << "\n"; 
    //int properY = findMaxY(points) - (topRight.y + topLeft.y)/2;
    //int bottomY = (topRight.y + topLeft.y)/2 + properY*2;
    //std::cout << "properY: " << properY << " bottomY: " << bottomY << "\n";
    // get the point nearest to (topLeft.x, bottomY) for the bottom left value
    // get the point nearest to (topRight.x, bottomY) for the bottom-right value
//    r.push_back(topLeft);
//    r.push_back(topRight);
    
    //cv::Point bottomLeftPoint = closestPoint(topLeft.x, bottomY,points);
    //cv::Point bottomRightPoint = closestPoint(topRight.x, bottomY,points);
    
    //r.push_back(bottomRightPoint);
    //r.push_back(bottomLeftPoint);
    int middleX = (topRight.x + topLeft.x) / 2;
    std::vector<cv::Point> hull;
    cv::convexHull(points, hull, false);
    //std::cout << "hull: " << hull << "\n";
    std::vector<cv::Point2f> approxPoly;
    cv::approxPolyDP(hull, approxPoly, 10.0,true);  
    return approxPoly;
}


// generate world cords. This method has been rigorously tested by pasting polygons into desmos and wolfram alpha
// to see if they look like the targets. I don't know 
std::vector<cv::Point3f> generateWorldConstant(){
    std::cout << "generating world constants...";
    
    std::vector<cv::Point3f> fullTarget;
    //int off = 81.25;
    fullTarget.push_back(cv::Point3f(-18.4725,0, 0));
    fullTarget.push_back(cv::Point3f( 18.4725,0, 0));
    fullTarget.push_back(cv::Point3f(9.8125, -17, 0));
    fullTarget.push_back(cv::Point3f(-9.8125,-17, 0));

    printf("Target points in world cords:\n"); 
    int point = 1;
    for(cv::Point3f x : fullTarget){
        std::cout << "point " << point << "  :  " << x << std::endl;
        point++;
    }
    // for easy copy-paste into WA to make sure it's decent
    
    std::cout << "polygon(";
    for(cv::Point3f x : fullTarget) {
        printf("(%f,%f),",x.x,x.y); // remember to remove the last ,
    }
    std::cout << ")" << std::endl;
    return fullTarget;
}


std::shared_ptr<nt::NetworkTable> startNetworkTable() {
    nt::NetworkTableInstance init = nt::NetworkTableInstance::GetDefault();
    init.StartClient("10.6.23.2");
    return init.GetTable("vision");
}

int bitCount(unsigned int);
void pushValues(std::shared_ptr<nt::NetworkTable>, double,double,double,double,double);

// MAIN
int main(int args, char** argss){
    std::cout << "" <<
       "=========" << std::endl <<
       " Vision Pipeline, 2019" << std::endl <<      
       " Contributors: Carson Graham" << std::endl <<
       "=========" << std::endl;
    std::vector<cv::Point3f> worldTarget = generateWorldConstant();

    std::cout << "Starting Network Tables\n"; 
    std::shared_ptr<nt::NetworkTable> table = startNetworkTable();
    

    
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
    SET_WITH_CHECK(cap.set(cv::CAP_PROP_FRAME_HEIGHT,288));
    SET_WITH_CHECK(cap.set(cv::CAP_PROP_FRAME_WIDTH,352));
    SET_WITH_CHECK(cap.set(cv::CAP_PROP_FPS,20));
   // cap.set(cv::CAP_PROP_CONVERT_RGB,0);
    SET_WITH_CHECK(cap.set(cv::VIDEOWRITER_PROP_QUALITY,10));
    SET_WITH_CHECK(cap.set(cv::CAP_PROP_CONTRAST,5));
    SET_WITH_CHECK(cap.set(cv::CAP_PROP_SHARPNESS,50));
    SET_WITH_CHECK(cap.set(cv::CAP_PROP_AUTO_EXPOSURE,1)); 
    SET_WITH_CHECK(cap.set(cv::CAP_PROP_EXPOSURE,9));
    SET_WITH_CHECK(cap.set(cv::CAP_PROP_BRIGHTNESS, 30));
    std::cout << "camera setup done" << std::endl;

    // check if we succeeded
    if (!cap.isOpened()) {
        std::cerr << "ERROR! Unable to open camera\n";
        return -1;
    }
    

        unsigned int noContoursHit = 0;  
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
            //std::cout << "size: " << frame.size() << std::endl;
   //       printf("channels: %d\ntype: %d\n",frame.channels(),frame.type());
        ;
        // std::cout << "cols:" << frame.cols << " rows: " << frame.rows << std::endl;
         cv::Mat img_hsv;
         cv::cvtColor(frame,img_hsv,cv::COLOR_BGR2HSV); 
         frame.release(); // release it from memory, saving memory

         double hue[] = {60.0, 107.0}; // these are the hue values it must fall between
	     double sat[] = {147.0, 255.0}; // means we need a lot of green
         double val[] = {100.0, 255.0}; // also the same as saturation but different?
         cv::Mat img_filtered;
         cv::inRange(img_hsv,cv::Scalar(hue[0],sat[0],val[0]),cv::Scalar(hue[1],sat[1],val[1]),img_filtered);
         
        

         // ASSERT img_filtered.type() == 0
         // this is a 8-bit integer with 1 channel
         // https://stackoverflow.com/questions/10167534/how-to-find-out-what-type-of-a-mat-object-is-with-mattype-in-opencv
         std::vector<std::vector<cv::Point>> o_contours;// original contours
         findContours(img_filtered,false,o_contours);
         //std::cout << "raw contours: " << og_contours.size() <<  "\n";
         img_filtered.release();
         // filter contours
         std::vector<std::vector<cv::Point>> contours;
         /*double filterContoursMinArea = 0;  
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
         
         */
        
         for(unsigned int i = 0; i < o_contours.size(); i++){
             std::vector<cv::Point> points = o_contours[i];
             if(arcLength(points, true) > 100) {
                 contours.push_back(points);
             } 
	        // std::cout << "contour " << i << " : " << points << "\n"; 
            
          }

         
         if( contours.size() == 0){
           noContoursHit++;
           if(bitCount(noContoursHit) == 1) {
             std::cout << noContoursHit << "\n";
           } else if(noContoursHit < 2){
             printf(" No contours\n");
           }
           pushValues(table,-1,-1,-1,-1,-1);
           continue;
         }
         noContoursHit = 0;
         if( contours.size() == 1){
         //  printf(" One Contour found\n");
         //  continue; 
         }
         
         if( contours.size() > 1){
             printf("more then one contour found\n");
            continue;
         }
         //std::cout << "contour " << contours[0] << "\n";
         bool printPoints = false;
         std::vector<cv::Point2f> points = contoursToPoints(contours[0]);
         if(printPoints){
           std::cout << points[0] << "," << points[1] << ", " << points[2] << "," <<points[3];
           std::cout << "";
           std::cout << std::endl;          
           continue; 
         }

         cv::Mat opoints = cv::InputArray(worldTarget).getMat();
         cv::Mat ipoints = cv::InputArray(points).getMat();
         //std::cout << "world target " << opoints.type() << " " << worldTarget 
         //          << "  points  "  << ipoints.type() << " " << points << std::endl;
         // continue;
         // m a t h
         if(points.size() != 4){
             printf("Found %i points, expecting 4\n", points.size());
             continue;
         }
         cv::Mat rvec;
         cv::Mat tvec;
         bool solvePnPSucc = cv::solvePnP(cv::InputArray(worldTarget),cv::InputArray(points),cameraMatrix,distCoeff,rvec,tvec);
         
         //std::cout << "solvePnPSucc: " << solvePnPSucc << std::endl;
        // std::cout << "tvec: " << tvec << std::endl;
         //std::cout << "rvec: " << rvec << std::endl;
         // TODO make sure that the assumptions made about the types of these mats is correct 
         //std::cout << "tvec type: " << tvec.type() << std::endl; //6 CV_64F
         //std::cout << "rvec type: " << rvec.type() << std::endl; //6 CV_64F
         std::vector<double> pos = findPos(rvec, tvec);
         double x        = pos[0];
         double z        = pos[1];
         double distance = pos[2];
         double angle1   = pos[3] * 180.0 / M_PI; 
         double angle2   = pos[4] * 180.0 / M_PI;
         
         printf("x:%10f     z:%10f      distance:      %10f angle1:      %10f angle2:      %10f",x,z,distance,angle1,angle2);
         std::cerr << angle1 << std::endl;
         //return 0;
         //table->GetEntry("x").SetDouble(x);
         //table->GetEntry("z").SetDouble(z);
         pushValues(table,x,z,distance,angle1,angle2);
         //printf("%f",angle1);
 
         std::cout << std::endl;
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}

void pushValues(std::shared_ptr<nt::NetworkTable> table, double x, double y, double dis, double ang1, double ang2){
    table->GetEntry("x").SetDouble(x);
    table->GetEntry("y").SetDouble(y);
    table->GetEntry("dis").SetDouble(dis);
    table->GetEntry("angle1").SetDouble(ang1);
    table->GetEntry("angle2").SetDouble(ang2);
}

int bitCount(unsigned int n){
    n = ((0xaaaaaaaa & n) >> 1) + (0x55555555 & n);
    n = ((0xcccccccc & n) >> 2) + (0x33333333 & n);
    n = ((0xf0f0f0f0 & n) >> 4) + (0x0f0f0f0f & n);
    n = ((0xff00ff00 & n) >> 8) + (0x00ff00ff & n);
    n = ((0xffff0000 & n) >> 16) + (0x0000ffff & n);
    return n;
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
        if (contour.size() < minVertexCount || contour.size() > maxVertexCount) 
              continue;
          double ratio = (double) bb.width / (double) bb.height;
        if (ratio < minRatio || ratio > maxRatio) continue;
        output.push_back(contour);
    }
}

