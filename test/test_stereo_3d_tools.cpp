/**
 * @file test_stereo_3d_tools.cpp
 * @brief Unit test of the stereo_3d_tools methods
 */
#include "stereo_3d_tools.hpp"
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace stereo_camera_calibrator;

int main(int argc, char **argv){
    if(argc < 4){
        std::cout << "Need to specify the disparity image and calibration file to be evaluated" << std::endl;
        std::cout << "./testStereo3DTools <calibrationfile> <disparity image> <color image> <empty space disparity>" << std::endl;
        return 1;
    }

    cv::FileStorage fs(argv[1], cv::FileStorage::READ);
    cv::Mat Q;
    fs["disparity_to_depth"] >> Q;
    cv::Mat disparity_map = cv::imread(argv[2], cv::IMREAD_GRAYSCALE);
    cv::Mat rgb_img = cv::imread(argv[3]);
    cv::threshold(disparity_map, disparity_map, 50, 255, cv::THRESH_TOZERO);
    cv::imshow("Disparity", disparity_map);
    cv::waitKey(10);

    std::cout<<"Testing object contrusctor" <<std::endl;
    Stereo3DTools s3dtools(disparity_map, Q);

    std::cout<<"Testing rotation" << std::endl;
    //s3dtools.rotatePoints(110.0, X_AXIS);

    std::cout<<"Testing outside the box removal"<<std::endl;
    cv::Point3f ctl_corner(134,-400,-70);
    cv::Point3f fbr_corner(-42, 100, -270);
    char c = 0;
    cv::Mat processed_disp;
    while(c!='q'){
        s3dtools.returnToOriginalPoints();
        s3dtools.rotatePoints(110.0, X_AXIS);

        s3dtools.removeOutsideTheBox(ctl_corner, fbr_corner);
        s3dtools.rotatePoints(110.0, X_AXIS);
        processed_disp = s3dtools.getDisparityMap();
        cv::imshow("Filtered_disp", processed_disp);
        c = cv::waitKey(0);
        switch(c){
            case 'a':ctl_corner.x++;break;
            case 'z':ctl_corner.x--;break;
            case 's':ctl_corner.y++;break;
            case 'x':ctl_corner.y--;break;
            case 'd':ctl_corner.z++;break;
            case 'c':ctl_corner.z--;break;
            case 'f':fbr_corner.x++;break;
            case 'v':fbr_corner.x--;break;
            case 'g':fbr_corner.y++;break;
            case 'b':fbr_corner.y--;break;
            case 'h':fbr_corner.z++;break;
            case 'n':fbr_corner.z--;break;

            case 'p':std::cout << "Point CTL: " << ctl_corner.x<<","<<ctl_corner.y<<","<<ctl_corner.z<<std::endl;
                     std::cout << "Point FBR: " << fbr_corner.x<<","<<fbr_corner.y<<","<<fbr_corner.z<<std::endl;
                     break;
        };
    }
    std::cout<<"Testing the erode3d"<<std::endl;
    //s3dtools.erode3d(5,6);

    //std::cout<<"Testing translation"<<std::endl;
    //s3dtools.translatePoints(1000, Z_AXIS);
    
    std::cout<<"Testing get lowest points"<<std::endl;
    std::vector<cv::Point3f> points;
    points = s3dtools.getHighestPoint(Y_AXIS, 20);
    std::cout<<points.size()<<std::endl;

    std::cout<<"Testing the meshlab save function" <<std::endl;
    s3dtools.saveComputedPoints(std::string("testmesh.ply"), rgb_img, points);

    return 0;
}
