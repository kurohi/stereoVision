/**
 * @file stereo_capture.cpp
 * @brief Implementation of the StereoCapture class
 */
#include "stereo_capture.hpp"
#include <iostream>
#include <stdexcept>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

StereoCapture::StereoCapture(int left_id, int right_id): left_control(left_id), right_control(right_id){
    if(!setLeftCameraId(left_id)){
        throw(std::runtime_error("Invalid camera index for left_camera"));
    }
    if(!setRightCameraId(right_id)){
        throw(std::runtime_error("Invalid camera index for right_camera"));
    }  
    //just deactivating auto gain, exposure and white balance 
    //but not setting it to any particular value
    left_control.loadFromCamera();
    right_control.loadFromCamera();
    left_control.setExposureTime(left_control.getExposureTime());
    right_control.setExposureTime(left_control.getExposureTime());
    //left_control.applySavedCommand();
    //right_control.applySavedCommand();
}

StereoCapture::~StereoCapture(){
    camera_left.release();
    camera_right.release();
}

void StereoCapture::matchExposureToLeftCamera(){
    left_control.loadFromCamera();
    right_control.loadFromCamera();
    left_control.setExposureTime(left_control.getExposureTime());
    right_control.setExposureTime(left_control.getExposureTime());
    left_control.applySavedCommand();
    right_control.applySavedCommand();
}

void StereoCapture::backToAutoExposure(){
    left_control.setGain(-1);
    left_control.setWhiteBalance(-1,-1,-1);
    left_control.setExposureTime(-1);
    left_control.applySavedCommand();

    right_control.setGain(-1);
    right_control.setWhiteBalance(-1,-1,-1);
    right_control.setExposureTime(-1);
    right_control.applySavedCommand();
}

cv::Mat StereoCapture::captureLeftImage(){
    if(!camera_left.isOpened()){
        camera_left.open(left_camera_id);
    }
    cv::Mat img;
    camera_left >> img;
    //camera_left.release();
    return img;
}

cv::Mat StereoCapture::captureRightImage(){
    if(!camera_right.isOpened()){
        camera_right.open(right_camera_id);
    }
    cv::Mat img;
    camera_right >> img;
    //camera_right.release();
    return img;
}

void StereoCapture::captureDoubleImages(cv::Mat &left_img, cv::Mat &right_img, bool with_rotation){
    left_img = captureLeftImage();
    right_img = captureRightImage();
    StereoCapture::rotate180(right_img);
}

bool StereoCapture::setLeftCameraId(int new_id){
    if(!camera_left.open(new_id)){
        std::cerr<<"The provided Id for the left camera does not exist"<<std::endl;
        return false;
    }
    left_camera_id = new_id;
    camera_left.release();
    return true;
}

bool StereoCapture::setRightCameraId(int new_id){
    if(!camera_right.open(new_id)){
        std::cerr<<"The provided Id for the right camera does not exist"<<std::endl;
        return false;
    }
    right_camera_id = new_id;
    camera_right.release();
    return true;
}

int StereoCapture::getLeftCameraId(){
    return left_camera_id;
}

int StereoCapture::getRightCameraId(){
    return right_camera_id;
}

void StereoCapture::saveImages(std::string folder_root, cv::Mat left_img, cv::Mat right_img){
    cv::imwrite(folder_root+"left.png", left_img);
    cv::imwrite(folder_root+"right.png", right_img);
}

void StereoCapture::saveImages(std::string folder_root){
    cv::Mat left_img, right_img;
    captureDoubleImages(left_img, right_img);
    saveImages(folder_root, left_img, right_img);
}

void StereoCapture::identifyLeftRight(){
    cv::Mat img1, img2;
    captureDoubleImages(img1, img2, false);
    identifyLeftRight(img1,img2, true);
}

void StereoCapture::identifyLeftRight(cv::Mat img1, cv::Mat img2, bool with_rotation){
    if(!StereoCapture::identifyLeftRightIndependent(img1, img2, with_rotation)){
        //If the image I rotated ended up on the left, that means that left and right are switched
        int aux = left_camera_id;
        left_camera_id = right_camera_id;
        right_camera_id = aux;
        //else, means that the present settings of left and right is right (ba dum tzzzz)
    }
}

bool StereoCapture::identifyLeftRightIndependent(cv::Mat img1, cv::Mat img2, bool with_rotation){
    if((img1.channels()>1)||(img2.channels()>1)){
        cv::cvtColor(img1, img1, cv::COLOR_RGB2GRAY);
        cv::cvtColor(img2, img2, cv::COLOR_RGB2GRAY);
    }
    if(with_rotation){
        StereoCapture::rotate180(img1);
    }
    std::vector<cv::Point2f> projection1, projection2;
    for(int x=0; x<img1.cols; x++){
        ulong counter1, counter2;
        counter1 = counter2 = 0;
        for(int y=0; y<img1.rows; y++){
            counter1 += img1.at<uchar>(y,x);
            counter2 += img2.at<uchar>(y,x);
        }
        projection1.push_back(cv::Point(x,counter1));
        projection2.push_back(cv::Point(x,counter2));
    }
    cv::Moments moments;
    moments = cv::moments(projection1, false);
    cv::Point2f center1(moments.m10/moments.m00, moments.m01/moments.m00);
    moments = cv::moments(projection2, false);
    cv::Point2f center2(moments.m10/moments.m00, moments.m01/moments.m00);
    std::cout<<center1<<std::endl;
    std::cout<<center2<<std::endl;
    
    //TODO: what if the two moments do not have the same height?
    //If the center1 was on the left
    if(center1.x < center2.x){
        if(with_rotation){
            //If the image I rotated ended up on the left, that means that left and right are switched
            return false;
        }//else, means that the present settings of left and right is right (ba dum tzzzz)
    }else{
        if(!with_rotation){
            //if the image1 is on the right without rotation, then they need to be switched
            return false;
        }
    }
    return true;
}


void StereoCapture::rotate180(cv::Mat &src){
    cv::Mat rotation_matrix = cv::getRotationMatrix2D(cv::Point(src.cols/2, src.rows/2), 180, 1);
    cv::warpAffine(src,src,rotation_matrix,src.size());
}

