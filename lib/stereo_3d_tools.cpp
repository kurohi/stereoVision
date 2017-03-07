/**
 * @file stereo_3d_tools.cpp
 * @brief Library for dealing with a cloud of points and disparity maps 
 */
#include "../include/stereo_3d_tools.hpp"
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp> 

#include <opencv2/highgui/highgui.hpp>

namespace{
    inline double powBy2(double a){
        return a*a;
    }
    inline double powBy2(float a){
        return a*a;
    }

    inline double pointSqDistance(const cv::Point3f &pt1, const cv::Point3f &pt2){
        return powBy2(pt1.x - pt2.x) + powBy2(pt1.y - pt2.y) + powBy2(pt1.z - pt2.z);
    }

}

namespace stereo_camera_calibrator {

    Stereo3DTools::Stereo3DTools(cv::Mat disparity, const cv::Mat &Q, cv::Mat BG_mask){
        if(BG_mask.cols>0){
            disparity.setTo(0, BG_mask == 0);
            if(cv::countNonZero(disparity) == 0){
                throw std::runtime_error("ERROR-Stereo3DTools::Stereo3DTools: The background mask erased the whole disparity");
            }
        }
        cv::reprojectImageTo3D(disparity, projected_points, Q, true);
        Qmatrix = Q;
        disparity.copyTo(orig_disp_img);
        disparity.copyTo(disp_img);
    }

    Stereo3DTools::Stereo3DTools() {
    }

    Stereo3DTools::~Stereo3DTools(){

    }

    bool Stereo3DTools::isProjectedReady(){
        return !(projected_points.empty());
    }

    void Stereo3DTools::updateProjectedPoints(const cv::Mat &disparity, const cv::Mat &Q){
        computed_points.release();
        projected_points.release();
        orig_disp_img.release();
        if(!disp_img.empty())
            disp_img.release();
        cv::reprojectImageTo3D(disparity, projected_points, Q, true);
        Qmatrix = Q;
        disparity.copyTo(orig_disp_img);
        disparity.copyTo(disp_img);
    }

    void Stereo3DTools::updateComputedPoints(const cv::Mat &disparity, const cv::Mat &Q){
        computed_points.release();
        if(!disp_img.empty())
            disp_img.release();
        cv::reprojectImageTo3D(disparity, computed_points, Q, true);
        Qmatrix = Q;
        disparity.copyTo(disp_img);
    }

    void Stereo3DTools::manualProjectedPoints(const cv::Mat &points_matrix, const cv::Mat &disparity){
        if(points_matrix.type() != CV_32FC3){
            throw std::runtime_error("ERROR-Stereo3DTools::manualProjectedPoints: Invalid point_cloud received on manualProjectedPoints");
        }
        computed_points.release();
        points_matrix.copyTo(projected_points);
        disparity.copyTo(orig_disp_img);
    }

    cv::Mat Stereo3DTools::getOriginalPointMatrix(){
        return projected_points;
    }

    cv::Mat Stereo3DTools::getModifiedPointMatrix(){
        if(computed_points.empty()){
            return projected_points;
        }
        return computed_points;
    }


    void Stereo3DTools::rotatePoints(const float angle, const axis_index axis){
        cv::Mat rotation_matrix;
        getRotationMatrix(angle, axis, rotation_matrix);
        applyRotationMatrix(rotation_matrix);
    }

    void Stereo3DTools::getRotationMatrix(const float angle, const axis_index axis, cv::Mat &rotation_matrix){
        //transform the angle into radians
        float rad_angle = angle*CV_PI/180;
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
    }

    void Stereo3DTools::applyRotationMatrix(const cv::Mat &rotation_matrix){
        if(computed_points.empty()){
            projected_points.copyTo(computed_points);
        }
        #pragma omp parallel for collapse(2)
        for(int y=0; y<projected_points.rows; y++){
            for(int x=0; x<projected_points.cols; x++){
                cv::Mat aux = rotation_matrix * cv::Mat(computed_points.at<cv::Vec3f>(y,   x), false);
                aux.copyTo(cv::Mat(computed_points.at<cv::Vec3f>(y,x), false));
            }
        }
    }

    cv::Point3f Stereo3DTools::applyRotationMatrix(const cv::Point& point, const cv::Mat &rotation_matrix, bool on_computed_points){
        cv::Point3f point3f;
        if(on_computed_points){
            point3f = computed_points.at<cv::Vec3f>(point.y, point.x);
        }else{
            point3f = projected_points.at<cv::Vec3f>(point.y, point.x);
        }
        cv::Mat aux = rotation_matrix * cv::Mat(point3f, false);
        aux.copyTo(cv::Mat(point3f, false));
        return point3f;
    }

    cv::Point3f Stereo3DTools::applyRotationMatrix(const cv::Point3f& point, const cv::Mat &rotation_matrix){
        cv::Point3f point3f(point);
        cv::Mat aux = rotation_matrix * cv::Mat(point3f, false);
        aux.copyTo(cv::Mat(point3f, false));
        return point3f;
    }

    void Stereo3DTools::translatePoints(const float distance_in, const axis_index axis){
        float distance = distance_in;
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
        #pragma omp parallel for collapse(2)
        for(int y=0; y<projected_points.rows; y++){
            for(int x=0; x<projected_points.cols; x++){
                computed_points.at<cv::Vec3f>(y,x)[point_coord_index] += distance;
            }
        }
    }

    void Stereo3DTools::returnToOriginalPoints(){
        computed_points.release();
        orig_disp_img.copyTo(disp_img);
    }

    void Stereo3DTools::removePointsCloseTo(const cv::Mat &target_points_disparity, const float distance_threshold){
        if(computed_points.empty()){
            projected_points.copyTo(computed_points);
            orig_disp_img.copyTo(disp_img);
        }
        cv::Mat target_points;
        cv::reprojectImageTo3D(target_points_disparity, target_points, Qmatrix, true);
        int ylimit = computed_points.rows-distance_threshold;
        int xlimit = computed_points.cols-distance_threshold;
        double sqDistance_threshold = distance_threshold * distance_threshold;
        #pragma omp parallel for collapse(2)
        for(int y=distance_threshold; y<ylimit; y++){
            for(int x=distance_threshold; x<xlimit; x++){
                cv::Point3f reference_point = target_points.at<cv::Point3f>(y,x);
                for(int inner_y=y-distance_threshold; inner_y<=y+distance_threshold; inner_y++)
                    for(int inner_x=x-distance_threshold; inner_x<=x+distance_threshold; inner_x++){
                        if (pointSqDistance(reference_point, computed_points.at<cv::Point3f>(inner_y, inner_x)) <= sqDistance_threshold) {
                            disp_img.at<short>(inner_y,inner_x)=0;
                        }
                    }
            }
        }
        cv::reprojectImageTo3D(disp_img, computed_points, Qmatrix, true);
        if(cv::countNonZero(disp_img) == 0){
            throw std::runtime_error("ERROR-Stereo3DTools::removePointsCloseTo: removed the whole point cloud");
        }
    }

    void Stereo3DTools::removeOutsideTheBox(const cv::Point3f &close_top_right_corner, const cv::Point3f &far_botton_left_corner){
        if(computed_points.empty()){
            projected_points.copyTo(computed_points);
            orig_disp_img.copyTo(disp_img);
        }
        #pragma omp parallel for collapse(2)
        for(int y=0; y<disp_img.rows; y++){
            for(int x=0; x<disp_img.cols; x++){
                cv::Vec3f cuboid = computed_points.at<cv::Vec3f>(y,x);
                if( (cuboid[0]<far_botton_left_corner.x)||
                    (cuboid[0]>close_top_right_corner.x)||
                    (cuboid[1]>far_botton_left_corner.y)||
                    (cuboid[1]<close_top_right_corner.y)||
                    (cuboid[2]>close_top_right_corner.z)||
                    (cuboid[2]<far_botton_left_corner.z)){

                    disp_img.at<short>(y,x) = 0; 
                }

            }
        }
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3), cv::Point(2,2));
        cv::erode(disp_img,disp_img,kernel,cv::Point(-1,-1), 4);
        cv::dilate(disp_img,disp_img,kernel,cv::Point(-1,-1), 4);
        cv::reprojectImageTo3D(disp_img, computed_points, Qmatrix, true);
        if(cv::countNonZero(disp_img) == 0){
            throw std::runtime_error("ERROR-Stereo3DTools::removeOutsideTheBox: removed the whole point cloud");
        }

    }

    std::vector<cv::Point3f> Stereo3DTools::getHighestPoint(const axis_index axis, const int distance_threshold){
        if(computed_points.empty()){
            projected_points.copyTo(computed_points);
            orig_disp_img.copyTo(disp_img);
        }
        int orientation = 1;
        int direction = 0;
        std::vector<cv::Point3f> highest_points;

        switch(axis){
            case X_AXIS_INV:
                orientation = -1;
            case X_AXIS:
                direction = 0; break;
            case Y_AXIS_INV:
                orientation = -1;
            case Y_AXIS:
                direction = 1; break;
            case Z_AXIS_INV:
                orientation = -1;
            case Z_AXIS:
                direction = 2; break;
            default:
                throw std::runtime_error("ERROR-Stereo3DTools::getHighestPoint: An invalid axis value was used on getHighestPoint method");
        };
        float highest_level = -999*orientation;

        cv::Mat disparity8;
        double minVal, maxVal;
        cv::minMaxLoc( disp_img, &minVal, &maxVal );
        disp_img.convertTo( disparity8, CV_8UC1, 255/(maxVal - minVal));

        for(int y=0; y<disp_img.rows; y++)
            for(int x=0; x<disp_img.cols; x++){
                if(disparity8.at<uchar>(y,x) == 0){
                    continue;
                }
                if(((orientation>0)&&(computed_points.at<cv::Vec3f>(y,x)[direction] > highest_level)) || ((orientation<0)&&(computed_points.at<cv::Vec3f>(y,x)[direction] < highest_level))){
                    highest_level = computed_points.at<cv::Vec3f>(y,x)[direction];
                }
            }

        #pragma omp parallel for collapse(2)
        for(int y=0; y<disp_img.rows; y++){
            for(int x=0; x<disp_img.cols; x++){
                if(disparity8.at<uchar>(y,x) == 0){
                    continue;
                }
                //removing the points that have a Z too big
                if(computed_points.at<cv::Vec3f>(y,x)[2] > 900){
                    continue;
                }
                if(std::abs(computed_points.at<cv::Vec3f>(y,x)[direction] - highest_level)<=distance_threshold){
                    #pragma omp critical(highest_point_update) 
                    highest_points.push_back(computed_points.at<cv::Vec3f>(y,x));
                }
            }
        }

        return  highest_points;
    }


    void Stereo3DTools::saveComputedPoints(const std::string &filename, const cv::Mat &rgb_img){
        std::ofstream plyfile(filename.c_str());
        if(!plyfile.is_open()){
            throw std::runtime_error("ERROR-Stereo3DTools::saveComputedPoints: File not found on saveComputedPoints");
        }
        if(computed_points.empty()){
            projected_points.copyTo(computed_points);
            orig_disp_img.copyTo(disp_img);
        }
        int failed_pixel = 0;
        for(int y=0; y<disp_img.rows; y++)
            for(int x=0; x<disp_img.cols; x++){
                if(disp_img.at<short>(y,x)==0){
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
                if(disp_img.at<short>(y,x)==0){
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


    void Stereo3DTools::saveComputedPoints(const std::string &filename, const cv::Mat &rgb_img, const std::vector<cv::Point3f> &pointlist){
        std::ofstream plyfile(filename.c_str());
        if(!plyfile.is_open()){
            throw std::runtime_error("ERROR-Stereo3DTools::saveComputedPoints: File not found on saveComputedPoints");
        }
        if(computed_points.empty()){
            projected_points.copyTo(computed_points);
            orig_disp_img.copyTo(disp_img);
        }
        int failed_pixel = 0;
        for(int y=0; y<disp_img.rows; y++)
            for(int x=0; x<disp_img.cols; x++){
                if(disp_img.at<short>(y,x)==0){
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
        size_t pointlistsize = pointlist.size();
        if(pointlistsize>0){
            for(size_t i=0; i<pointlistsize; i++){
                point = pointlist[i];
                plyfile << point.x << " " << point.y << " " << point.z << " ";
                plyfile << "255 0 0" <<std::endl;
            }
        }
        for(int y=0; y<computed_points.rows; y++)
            for(int x=0; x<computed_points.cols; x++){
                //ignore pixels that are completely black on the disparity image
                if(disp_img.at<short>(y,x)==0){
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

    void Stereo3DTools::erode3d(const int cube_size, const int minimum_number){
        if(computed_points.empty()){
            projected_points.copyTo(computed_points);
            orig_disp_img.copyTo(disp_img);
        }
        #pragma omp parallel for collapse(2)
        for(int y=cube_size/2; y<disp_img.rows-cube_size/2; y++){
            for(int x=cube_size/2; x<disp_img.cols-cube_size/2; x++){
                if(disp_img.at<short>(y,x)==0){
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
                    disp_img.at<short>(y,x) = 0;
                }
            }
        }
        cv::reprojectImageTo3D(disp_img, computed_points, Qmatrix, true);
        if(cv::countNonZero(disp_img) == 0){
            throw std::runtime_error("ERROR-Stereo3DTools::erode3d: The whole point cloud was eroded");
        }
    }

    double Stereo3DTools::pointDistance(const cv::Point3f &pt1, const cv::Point3f &pt2){
        return sqrt(powBy2(pt1.x - pt2.x) + powBy2(pt1.y - pt2.y) + powBy2(pt1.z - pt2.z));
    }

    cv::Mat Stereo3DTools::getDisparityMap(){
        if(computed_points.empty()){
            return orig_disp_img;
        }
        return disp_img;
    }

    std::vector<cv::Point3f> Stereo3DTools::shiftCoordinates(const std::vector<cv::Point3f> &original_points, const cv::Point3f &new_origin, const cv::Point3f &new_sense){
        size_t original_pointssize = original_points.size();
        std::vector<cv::Point3f> output_vector(original_pointssize, cv::Point3f(0,0,0));
        for(unsigned int i=0; i<original_pointssize; i++){
            cv::Point3f aux_point = shitftOnePoint(original_points[i], new_origin, new_sense);
            output_vector[i] = aux_point;
        }
        return output_vector;
    }

    cv::Point3f Stereo3DTools::shitftOnePoint(const cv::Point3f &target, const cv::Point3f &new_origin, const cv::Point3f &new_sense){
        cv::Point3f shifted_point;
        shifted_point.x = (target.x - new_origin.x)*new_sense.x;
        shifted_point.y = (target.y - new_origin.y)*new_sense.y;
        shifted_point.z = (target.z - new_origin.z)*new_sense.z;

        return shifted_point;
    }

    cv::Point Stereo3DTools::retraceXYfrom3D(const cv::Point3f &search_point){
        if(computed_points.empty()){
            projected_points.copyTo(computed_points);
            orig_disp_img.copyTo(disp_img);
        }
        double f, a, b, Cx, Cy;
        f = Qmatrix.at<double>(2,3);
        a = Qmatrix.at<double>(3,2);
        b = Qmatrix.at<double>(3,3);
        Cx = -Qmatrix.at<double>(0,3);
        Cy = -Qmatrix.at<double>(1,3);
        double d = (f - search_point.z*b)/(search_point.z*a);
        int x = search_point.x*(d*a+b)+Cx;
        int y = search_point.y*(d*a+b)+Cy;

        return cv::Point(x,y);
    }
    cv::Point Stereo3DTools::retraceXYfrom3D(const cv::Point3f &search_point, const float range){
        if(computed_points.empty()){
            projected_points.copyTo(computed_points);
            orig_disp_img.copyTo(disp_img);
        }
        bool finished = false;
        cv::Point found_point;
        double sqRange = range*range;
        #pragma omp parallel for collapse(2)
        for (int y = 0; y<computed_points.rows; y++){
            for (int x = 0; x<computed_points.cols; x++){
                if (finished)
                    continue;
                double sqDist = powBy2(search_point.x - computed_points.at<cv::Point3f>(y, x).x) +
                    powBy2(search_point.y - computed_points.at<cv::Point3f>(y, x).y) +
                    powBy2(search_point.z - computed_points.at<cv::Point3f>(y, x).z);
                if (sqDist < sqRange){
                    #pragma omp critical(found_point) 
                    {
                        found_point = cv::Point(x, y);
                        finished = true;
                    }
                }
            }
        }
        if(finished){
            return found_point;
        }
        throw std::runtime_error("ERROR-Stereo3DTools::retraceXYfrom3D: Retrace point not found");
    }

    cv::Point Stereo3DTools::retraceXYfrom3D_Zindependent(const cv::Point3f &search_point, const float range){
        if(computed_points.empty()){
            projected_points.copyTo(computed_points);
            orig_disp_img.copyTo(disp_img);
        }
        bool finished = false;
        cv::Point found_point;
        double sqRange = range*range;
        #pragma omp parallel for collapse(2)
        for (int y = 0; y<computed_points.rows; y++){
            for (int x = 0; x<computed_points.cols; x++){
                if (finished)
                    continue;
                double sqDist = powBy2(search_point.x - computed_points.at<cv::Point3f>(y, x).x) +
                    powBy2(search_point.y - computed_points.at<cv::Point3f>(y, x).y);
                if (sqDist < sqRange){
                    #pragma omp critical(found_point_zindependent)
                    {
                        found_point = cv::Point(x, y);
                        finished = true;
                    }
                }
            }
        }
        if(finished){
            return found_point;
        }
        throw std::runtime_error("ERROR-Stereo3DTools::retraceXYfrom3D_Zindependent: Retrace point not found");
    }

    cv::Point3f Stereo3DTools::retraceXYZfrom2D(const cv::Point &search_point){
        if(computed_points.empty()){
            projected_points.copyTo(computed_points);
            orig_disp_img.copyTo(disp_img);
        }
        return computed_points.at<cv::Point3f>(search_point.y, search_point.x);
    }

    void Stereo3DTools::filterToBiggestRegion(){
        if(computed_points.empty()){
            projected_points.copyTo(computed_points);
            orig_disp_img.copyTo(disp_img);
        }
        cv::Mat binarized_disp;
        cv::threshold(disp_img, binarized_disp, 10, 255, cv::THRESH_BINARY);
        cv::Mat binarized8;
        double minVal;
        double maxVal;
        cv::minMaxLoc( binarized_disp, &minVal, &maxVal );
        binarized_disp.convertTo( binarized8, CV_8UC1, 255/(maxVal - minVal));
        cv::Mat kernel2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(15, 15), cv::Point(2, 2));
        cv::dilate(binarized8, binarized8, kernel2, cv::Point(-1, -1), 2);

        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(binarized8, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

        double biggest_area = 0;
        int biggest_index = -1;
        size_t contourssize = contours.size();
        for(size_t i=0; i<contourssize; i=hierarchy[i][0]){
            double area = contourArea(contours[i]);
            if(area > biggest_area){
                biggest_area = area;
                biggest_index = i;
            }
        }
        if(biggest_index == -1){
            throw std::runtime_error("ERROR-Stereo3DTools::filterToBiggestRegion: No contour found");
        }

        cv::Rect biggest_area_rect = cv::boundingRect(contours[biggest_index]);
        cv::Mat filtered_disp = cv::Mat::zeros(disp_img.size(), disp_img.type());

        #pragma omp parallel for collapse(2)
        for(int y=biggest_area_rect.y; y < biggest_area_rect.y+biggest_area_rect.height; y++){
            for(int x=biggest_area_rect.x; x < biggest_area_rect.x+biggest_area_rect.width; x++){
                if(cv::pointPolygonTest(contours[biggest_index], cv::Point2f(x,y), false) >= 0){
                    filtered_disp.at<short>(y,x) = disp_img.at<short>(y,x);
                }
            }
        }

        //updateProjectedPoints(filtered_disp, Qmatrix);
        updateComputedPoints(filtered_disp, Qmatrix);
    }

    void Stereo3DTools::removeAllFromPoint(const cv::Point3f &start_pt, const axis_index axis){
        if(computed_points.empty()){
            projected_points.copyTo(computed_points);
        }
        int direction_axis;
        int direction = 1;
        switch(axis){
            case X_AXIS_INV:
                 direction *= -1;
            case X_AXIS:
                 direction_axis = 0;
                break;
            case Y_AXIS_INV:
                direction *= -1;
                direction_axis = 1;
            case Y_AXIS:
                break;
            case Z_AXIS_INV:
                direction *= -1;
            case Z_AXIS:
                direction_axis = 2;
                break;
        };
        cv::Vec3f vec_pt(start_pt.x, start_pt.y, start_pt.z);
        #pragma omp parallel for collapse(2)
        for(int y=0; y<projected_points.rows; y++){
            for(int x=0; x<projected_points.cols; x++){
                if(computed_points.at<cv::Vec3f>(y,x)[direction_axis] > direction*vec_pt[direction_axis]){
                    disp_img.at<short>(y,x) = 0;
                }
            }
        }

        cv::reprojectImageTo3D(disp_img, computed_points, Qmatrix, true);
        if(cv::countNonZero(disp_img) == 0){
            throw std::runtime_error("ERROR-Stereo3DTools::removeAllFromPoint: removed the whole point cloud");
        }
    }

}
