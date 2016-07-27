/**
 * @file stereo_3d_tools.cpp
 * @brief Library for dealing with a cloud of points and disparity maps 
 */
#include "stereo_3d_tools.hpp"
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp> 

#include <opencv2/highgui/highgui.hpp>

namespace stereo_camera_calibrator {

    Stereo3DTools::Stereo3DTools(cv::Mat disparity, cv::Mat Q){
        cv::reprojectImageTo3D(disparity, projected_points, Q);
        Qmatrix = Q;
        orig_disp_img = disparity;
        disp_img = disparity;
    }

    Stereo3DTools::Stereo3DTools() {
    }

    Stereo3DTools::~Stereo3DTools(){

    }

    bool Stereo3DTools::isProjectedReady(){
        return !(projected_points.empty());
    }
    
    void Stereo3DTools::updateProjectedPoints(cv::Mat disparity, cv::Mat Q){
        computed_points.release();
        cv::reprojectImageTo3D(disparity, projected_points, Q);
        Qmatrix = Q;
        orig_disp_img = disparity;
    }

    void Stereo3DTools::manualProjectedPoints(cv::Mat points_matrix, cv::Mat disparity){
        if(points_matrix.type() != CV_32FC3){
            throw std::runtime_error("Invalid point_cloud received on manualProjectedPoints");
        }
        computed_points.release();
        points_matrix.copyTo(projected_points);
        orig_disp_img = disparity;
    }

    cv::Mat Stereo3DTools::getOriginalPointMatrix(){
        return projected_points;
    }

    cv::Mat Stereo3DTools::getModifiedPointMatrix(){
        if(!(computed_points.empty())){
            return projected_points;
        }

        for(int y=0;y<computed_points.rows;y++)
            for(int x=0;x<computed_points.cols;x++){
                std::cout<<computed_points.at<cv::Point3f>(y,x)<<std::endl;
            }
        return computed_points;
    }


    void Stereo3DTools::rotatePoints(float angle, axis_index axis){
        //transform the angle into radians
        float rad_angle = angle*CV_PI/180;
        cv::Mat rotation_matrix;
        switch(axis){
            case X_AXIS_INV:
                rad_angle *= -1;
            case X_AXIS:
                rotation_matrix = (cv::Mat_<float>(3, 3) <<
                                    1,          0,           0,
                                    0, cos(rad_angle), -sin(rad_angle),
                                    0, sin(rad_angle),  cos(rad_angle));
                break;
            case Y_AXIS_INV:
                rad_angle *= -1;
            case Y_AXIS:
                rotation_matrix = (cv::Mat_<float>(3, 3) <<
                                    cos(rad_angle),  0,   sin(rad_angle),
                                    0,              1,   0,
                                    -sin(rad_angle),0,   cos(rad_angle));
                break;
            case Z_AXIS_INV:
                rad_angle *= -1;
            case Z_AXIS:
                rotation_matrix = (cv::Mat_<float>(3, 3) <<
                                    cos(rad_angle), -sin(rad_angle),0,
                                    sin(rad_angle), cos(rad_angle), 0,
                                    0,              0,              1);
                break;
            default:
                rotation_matrix = (cv::Mat_<float>(3, 3) <<
                                    1, 0, 0,
                                    0, 1, 0,
                                    0, 0, 1);
        };
        if(computed_points.empty()){
            projected_points.copyTo(computed_points);
        }
        for(int y=0; y<projected_points.rows; y++)
            for(int x=0; x<projected_points.cols; x++){
                cv::Mat aux = rotation_matrix * cv::Mat(projected_points.at<cv::Vec3f>(y,   x), false);
                aux.copyTo(cv::Mat(computed_points.at<cv::Vec3f>(y,x), false));
            }
    }

    void Stereo3DTools::translatePoints(float distance, axis_index axis){
        cv::Mat translation_matrix;
        int point_coord_index = 0;
        switch(axis){
            case X_AXIS_INV:
                distance *= -1;
            case X_AXIS:
                point_coord_index = 0;
                break;
            case Y_AXIS_INV:
                distance *= -1;
            case Y_AXIS:
                point_coord_index = 1;
                break;
            case Z_AXIS_INV:
                distance *= -1;
            case Z_AXIS:
                point_coord_index = 2;
                break;
            default:
                distance = 0;
        };
        if(computed_points.empty()){
            projected_points.copyTo(computed_points);
        }
        for(int y=0; y<projected_points.rows; y++)
            for(int x=0; x<projected_points.cols; x++){
                computed_points.at<cv::Vec3f>(y,x)[point_coord_index] += distance;
            }
    }

    void Stereo3DTools::returnToOriginalPoints(){
        computed_points.release();
        orig_disp_img.copyTo(disp_img);
    }

    void Stereo3DTools::removePointsCloseTo(cv::Mat target_points_disparity, float distance_threshold){
        if(computed_points.empty()){
            projected_points.copyTo(computed_points);
            orig_disp_img.copyTo(disp_img);
        }
        cv::Mat target_points;
        cv::reprojectImageTo3D(target_points_disparity, target_points, Qmatrix);
        for(int y=distance_threshold; y<computed_points.rows-distance_threshold; y++)
            for(int x=distance_threshold; x<computed_points.cols-distance_threshold; x++){
                cv::Point3f reference_point = target_points.at<cv::Point3f>(y,x);
                for(int inner_y=y-distance_threshold; inner_y<=y+distance_threshold; inner_y++)
                    for(int inner_x=x-distance_threshold; inner_x<=x+distance_threshold; inner_x++){
                        if(pointDistance(reference_point, computed_points.at<cv::Point3f>(inner_y, inner_x))<=distance_threshold) {
                            disp_img.at<uchar>(inner_y,inner_x)=0;
                        }
                    }
            }
        cv::reprojectImageTo3D(disp_img, computed_points, Qmatrix);
    }

    void Stereo3DTools::removeOutsideTheBox(cv::Point3f close_top_right_corner, cv::Point3f far_botton_left_corner){
        if(computed_points.empty()){
            projected_points.copyTo(computed_points);
            orig_disp_img.copyTo(disp_img);
        }
        for(int y=0; y<disp_img.rows; y++)
            for(int x=0; x<disp_img.cols; x++){
                cv::Vec3f voxel = computed_points.at<cv::Vec3f>(y,x);
                if( (voxel[0]<far_botton_left_corner.x)||
                    (voxel[0]>close_top_right_corner.x)||
                    (voxel[1]>far_botton_left_corner.y)||
                    (voxel[1]<close_top_right_corner.y)||
                    (voxel[2]>close_top_right_corner.z)||
                    (voxel[2]<far_botton_left_corner.z)){
    
                    disp_img.at<uchar>(y,x) = 0; 
                }

            }
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3), cv::Point(2,2));
        cv::erode(disp_img,disp_img,kernel,cv::Point(-1,-1), 4);
        cv::dilate(disp_img,disp_img,kernel,cv::Point(-1,-1), 4);
        cv::reprojectImageTo3D(disp_img, computed_points, Qmatrix);
    }

    std::vector<cv::Point3f> Stereo3DTools::getHighestPoint(axis_index axis, int distance_threshold){
        if(computed_points.empty()){
            projected_points.copyTo(computed_points);
            orig_disp_img.copyTo(disp_img);
        }
        int orientation = 1;
        int direction = 0;
        float highest_level = 0;
        std::vector<cv::Point3f> highest_points;

        switch(axis){
            case X_AXIS_INV:
                orientation = -1;
            case X_AXIS:
                direction = 0; break;
            case Y_AXIS_INV:
                orientation = -1;
            case Y_AXIS:
                highest_level = -999;
                direction = 1; break;
            case Z_AXIS_INV:
                orientation = -1;
            case Z_AXIS:
                direction = 2; break;
            default:
                throw std::runtime_error("An invalid axis value was used on getHighestPoint method");
        };

        for(int y=0; y<disp_img.rows; y++)
            for(int x=0; x<disp_img.cols; x++){
                if(disp_img.at<uchar>(y,x) == 0){
                    continue;
                }
                if(abs(computed_points.at<cv::Vec3f>(y,x)[direction] - highest_level)<=distance_threshold){
                    highest_points.push_back(computed_points.at<cv::Vec3f>(y,x));
                }else if(((orientation>0)&&(computed_points.at<cv::Vec3f>(y,x)[direction] > (highest_level+distance_threshold))) || ((orientation<0)&&(computed_points.at<cv::Vec3f>(y,x)[direction] < (highest_level+distance_threshold)))){
                    highest_level = computed_points.at<cv::Vec3f>(y,x)[direction];
                    highest_points.clear();
                    highest_points.push_back(computed_points.at<cv::Vec3f>(y,x));
                }
            }
        return  highest_points;
    }


    void Stereo3DTools::saveComputedPoints(std::string filename, cv::Mat rgb_img){
        std::ofstream plyfile(filename.c_str());
        if(!plyfile.is_open()){
            throw std::runtime_error("File not found on saveComputedPoints");
        }
        if(computed_points.empty()){
            projected_points.copyTo(computed_points);
            orig_disp_img.copyTo(disp_img);
        }
        int failed_pixel = 0;
        for(int y=0; y<disp_img.rows; y++)
            for(int x=0; x<disp_img.cols; x++){
                if(disp_img.at<uchar>(y,x)==0){
                    failed_pixel++;
                }
            }
        int point_num = (int) computed_points.total();

        //saving headers for the ply file
        plyfile << "ply" << std::endl;
        plyfile << "format ascii 1.0" << std::endl;
        plyfile << "element vertex " << point_num - failed_pixel << std::endl;
        plyfile << "property float x" << std::endl;
        plyfile << "property float y" << std::endl;
        plyfile << "property float z" << std::endl;
        plyfile << "property uchar red" << std::endl;
        plyfile << "property uchar green" << std::endl;
        plyfile << "property uchar blue" << std::endl;
        plyfile << "end_header" << std::endl;

        cv::Point3f point;
        for(int y=0; y<computed_points.rows; y++)
            for(int x=0; x<computed_points.cols; x++){
                //ignore pixels that are completely black on the disparity image
                if(disp_img.at<uchar>(y,x)==0){
                    continue;
                }
                //save every point per line in the order XYZ RGB
                point = computed_points.at<cv::Point3f>(y,x);
                plyfile << point.x << " " << point.y << " " << point.z << " ";
                plyfile << (int)rgb_img.at<cv::Vec3b>(y,x)[2] << " " <<
                (int) rgb_img.at<cv::Vec3b>(y,x)[1] << " " << (int)rgb_img.at<cv::Vec3b>(y,x)[0] << std::endl;
            }
        plyfile.close();
    }


    void Stereo3DTools::saveComputedPoints(std::string filename, cv::Mat rgb_img, std::vector<cv::Point3f> pointlist){
        std::ofstream plyfile(filename.c_str());
        if(!plyfile.is_open()){
            throw std::runtime_error("File not found on saveComputedPoints");
        }
        if(computed_points.empty()){
            projected_points.copyTo(computed_points);
            orig_disp_img.copyTo(disp_img);
        }
        int failed_pixel = 0;
        for(int y=0; y<disp_img.rows; y++)
            for(int x=0; x<disp_img.cols; x++){
                if(disp_img.at<uchar>(y,x)==0){
                    failed_pixel++;
                }
            }
        int point_num = (int) computed_points.total();

        //saving headers for the ply file
        plyfile << "ply" << std::endl;
        plyfile << "format ascii 1.0" << std::endl;
        plyfile << "element vertex " << point_num - failed_pixel + pointlist.size() << std::endl;
        plyfile << "property float x" << std::endl;
        plyfile << "property float y" << std::endl;
        plyfile << "property float z" << std::endl;
        plyfile << "property uchar red" << std::endl;
        plyfile << "property uchar green" << std::endl;
        plyfile << "property uchar blue" << std::endl;
        plyfile << "end_header" << std::endl;

        cv::Point3f point;
        if(pointlist.size()>0){
            for(int i=0; i<pointlist.size(); i++){
                point = pointlist[i];
                plyfile << point.x << " " << point.y << " " << point.z << " ";
                plyfile << "255 0 0" <<std::endl;
            }
        }
        for(int y=0; y<computed_points.rows; y++)
            for(int x=0; x<computed_points.cols; x++){
                //ignore pixels that are completely black on the disparity image
                if(disp_img.at<uchar>(y,x)==0){
                    continue;
                }
                //save every point per line in the order XYZ RGB
                point = computed_points.at<cv::Point3f>(y,x);
                plyfile << point.x << " " << point.y << " " << point.z << " ";
                plyfile << (int)rgb_img.at<cv::Vec3b>(y,x)[2] << " " <<
                (int) rgb_img.at<cv::Vec3b>(y,x)[1] << " " << (int)rgb_img.at<cv::Vec3b>(y,x)[0] << std::endl;
            }
        plyfile.close();
    }

    void Stereo3DTools::erode3d(int cube_size, int minimum_number){
        if(computed_points.empty()){
            projected_points.copyTo(computed_points);
            orig_disp_img.copyTo(disp_img);
        }
        for(int y=cube_size/2; y<disp_img.rows-cube_size/2; y++)
            for(int x=cube_size/2; x<disp_img.cols-cube_size/2; x++){
                if(disp_img.at<uchar>(y,x)==0){
                    continue;
                }
                int close_counter = 0;
                for(int inner_y = y-cube_size/2; inner_y<y+cube_size/2; inner_y++)
                    for(int inner_x = x-cube_size/2; inner_x<x+cube_size/2; inner_x++){
                        if(pointDistance(computed_points.at<cv::Point3f>(y,x), computed_points.at<cv::Point3f>(inner_y, inner_x)) <= cube_size/2+1){
                            close_counter++;
                        }
                    }
                if(close_counter < minimum_number){
                    disp_img.at<uchar>(y,x) = 0;
                }
            }
        cv::imshow("newdisp", disp_img);
        cv::waitKey(0);
        cv::reprojectImageTo3D(disp_img, computed_points, Qmatrix);
    }

    float Stereo3DTools::pointDistance(cv::Point3f pt1, cv::Point3f pt2){
        return sqrt(pow(pt1.x-pt2.x,2)+pow(pt1.y-pt2.y,2)+pow(pt1.z-pt2.z,2));
    }

    cv::Mat Stereo3DTools::getDisparityMap(){
        if(computed_points.empty()){
            return orig_disp_img;
        }
        return disp_img;
    }

    std::vector<cv::Point3f> Stereo3DTools::shiftCoordinates(std::vector<cv::Point3f> original_points, cv::Point3f new_origin, cv::Point3f new_sense){
        std::vector<cv::Point3f> output_vector;
        for(int i=0; i<original_points.size(); i++){
            cv::Point3f aux_point = shitftOnePoint(original_points[i], new_origin, new_sense);
            output_vector.push_back(aux_point);
        }
        return output_vector; 
    }

    cv::Point3f Stereo3DTools::shitftOnePoint(cv::Point3f target, cv::Point3f new_origin, cv::Point3f new_sense){
        cv::Point3f shifted_point;
        shifted_point.x = (target.x - new_origin.x)*new_sense.x;
        shifted_point.y = (target.y - new_origin.y)*new_sense.y;
        shifted_point.z = (target.z - new_origin.z)*new_sense.z;

        return shifted_point;
    }
}
