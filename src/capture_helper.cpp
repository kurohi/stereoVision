#include "stereo_capture.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>

void captureSingleOnly(stereo_tools::StereoCapture &camera_handler, const std::string &root, cv::Size board_size, const std::string &side, bool circlegrid=false){
     cv::Mat average_img, captured_img, output_img;
     captured_img = camera_handler.captureLeftImage();
     average_img = cv::Mat::zeros(captured_img.size(), CV_8UC1);

     std::cout<<"Capturing images for the "<<side<<" camera"<<std::endl;
     char c=0;
     unsigned int img_index = 0;
     while(c!='q'){
        if(side == "left"){
            captured_img = camera_handler.captureLeftImage();
        }else if(side == "right"){
            captured_img = camera_handler.captureRightImage();
        }else{
            throw(std::runtime_error("Side not recognized"));
        }
        captured_img.copyTo(output_img);
        bool found_it = false;
        std::vector<cv::Point2f> found_points;

        if(circlegrid){
            try{
                found_it = cv::findCirclesGrid(captured_img, board_size, found_points, cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING);
            }catch(cv::Exception &exp){
                continue;
            }
            if(found_it){
                cv::drawChessboardCorners(output_img, board_size, found_points, found_it);
            }
        }else{
            found_it = cv::findChessboardCorners(captured_img, board_size, found_points, cv::CALIB_CB_ADAPTIVE_THRESH|cv::CALIB_CB_NORMALIZE_IMAGE); 
            if(found_it){
                cv::Mat gray_img;
                cv::cvtColor(captured_img, gray_img, CV_BGR2GRAY);
                cv::cornerSubPix(gray_img, found_points, cv::Size(7,7), cv::Size(-1,-1), cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 30, 0.01));
                cv::drawChessboardCorners(output_img, board_size, found_points, found_it);
            }
        }

        cv::imshow("camera feed - s to save, q to finish", output_img);
        c = cv::waitKey(10);
        if((c == 's')&&(found_it)){
            cv::Mat newpoints = cv::Mat::zeros(average_img.size(), average_img.type());
            for(size_t i=0; i<found_points.size(); i++){
                cv::circle(newpoints, found_points.at(i), 5, 50, -1);
            }
            cv::blur(newpoints, newpoints, cv::Size(5,5));
            average_img = average_img + newpoints;
            cv::Mat heat_map(average_img.size(), CV_8UC3);
            cv::applyColorMap(average_img, heat_map, cv::COLORMAP_HOT);
            cv::imshow("Points heatmap", heat_map);
            std::stringstream sstream;
            sstream << root << "/" << side << "_" << std::setw(3) << std::setfill('0') << img_index << ".png";
            std::cout<<"Saving image: "<<sstream.str()<<std::endl;
            imwrite(sstream.str(), captured_img);
            img_index++;
        }
     }
}

void captureStereo(stereo_tools::StereoCapture &camera_handler, const std::string &root, cv::Size board_size, bool circlegrid=false){
     cv::Mat average_img, captured_left_img, captured_right_img, output_left_img, output_right_img;
     captured_left_img = camera_handler.captureLeftImage();
     average_img = cv::Mat::zeros(captured_left_img.size(), CV_8UC1);

     std::cout<<"Capturing images for the camera pair"<<std::endl;
     char c=0;
     unsigned int img_index = 0;
     while(c!='q'){
        camera_handler.captureDoubleImages(captured_left_img, captured_right_img);
        captured_left_img.copyTo(output_left_img);
        captured_right_img.copyTo(output_right_img);
        bool found_it_left = false;
        bool found_it_right = false;
        std::vector<cv::Point2f> found_left_points;
        std::vector<cv::Point2f> found_right_points;

        if(circlegrid){
            found_it_left = cv::findCirclesGrid(captured_left_img, board_size, found_left_points, cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING);
            found_it_right = cv::findCirclesGrid(captured_right_img, board_size, found_right_points, cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING);
            if(found_it_left){
                cv::drawChessboardCorners(output_left_img, board_size, found_left_points, found_it_left);
            }
            if(found_it_right){
                cv::drawChessboardCorners(output_right_img, board_size, found_right_points, found_it_right);
            }
        }else{
            found_it_left = cv::findChessboardCorners(captured_left_img, board_size, found_left_points, cv::CALIB_CB_ADAPTIVE_THRESH|cv::CALIB_CB_NORMALIZE_IMAGE); 
            found_it_right = cv::findChessboardCorners(captured_right_img, board_size, found_right_points, cv::CALIB_CB_ADAPTIVE_THRESH|cv::CALIB_CB_NORMALIZE_IMAGE); 
            if(found_it_left){
                cv::Mat gray_img;
                cv::cvtColor(captured_left_img, gray_img, CV_BGR2GRAY);
                cv::cornerSubPix(gray_img, found_left_points, cv::Size(7,7), cv::Size(-1,-1), cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 30, 0.01));
                cv::drawChessboardCorners(output_left_img, board_size, found_left_points, found_it_left);
            }
            if(found_it_right){
                cv::Mat gray_img;
                cv::cvtColor(captured_right_img, gray_img, CV_BGR2GRAY);
                cv::cornerSubPix(gray_img, found_right_points, cv::Size(7,7), cv::Size(-1,-1), cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 30, 0.01));
                cv::drawChessboardCorners(output_right_img, board_size, found_right_points, found_it_right);
            }
        }
        cv::Mat concated_img;
        cv::hconcat(output_left_img, output_right_img, concated_img);

        cv::imshow("camera feed - s to save, q to finish", concated_img);
        c = cv::waitKey(10);
        if(c == 's'){
            if(found_it_left && found_it_right){
                cv::Mat newpoints = cv::Mat::zeros(average_img.size(), average_img.type());
                for(size_t i=0; i<found_left_points.size(); i++){
                    cv::circle(newpoints, found_left_points.at(i), 5, 50, -1);
                }
                cv::blur(newpoints, newpoints, cv::Size(5,5));
                average_img = average_img + newpoints;
                cv::Mat heat_map(average_img.size(), CV_8UC3);
                cv::applyColorMap(average_img, heat_map, cv::COLORMAP_HOT);
                cv::imshow("Points heatmap", heat_map);
                std::stringstream sstream;
                sstream << root << "/stereo_left_" << std::setw(3) << std::setfill('0') << img_index << ".png";
                std::cout<<"Saving image: "<<sstream.str()<<std::endl;
                imwrite(sstream.str(), captured_left_img);
                std::stringstream sstream2;
                sstream2 << root << "/stereo_right_" << std::setw(3) << std::setfill('0') << img_index << ".png";
                std::cout<<"Saving image: "<<sstream2.str()<<std::endl;
                imwrite(sstream2.str(), captured_right_img);
                img_index++;
            }
        }
     }
}

int main(int argc, char **argv){
    if(argc < 4){
        std::cout<<"need to specify the folder to save the images together with the width and height of the board"<<std::endl;
        std::cout<<"example:"<<std::endl;
        std::cout<<"./capture_helper <savefolder> <width of board> <height of board> <-t for triple_set> <-c for circle grid instead of chessboard>"<<std::endl;
        return 1;
    }
    bool triple_mode = false;
    bool circle_grid = false;
    int board_w = atoi(argv[2]);
    int board_h = atoi(argv[3]);
    cv::Size board_size(board_w, board_h);
    for(int i=4; i<argc; i++){
        if(std::string(argv[i]) == "-t"){
            std::cout<<"Triple capture mode: ON"<<std::endl;
            triple_mode = true;
        }else if(std::string(argv[i]) == "-c"){
            std::cout<<"Circle grid mode: ON"<<std::endl;
            circle_grid = true;
        }
    }
    try{
        stereo_tools::StereoCapture camera_handler(1,2);
    
        if(triple_mode){
            captureSingleOnly(camera_handler, std::string(argv[1]), board_size, "left", circle_grid);
            captureSingleOnly(camera_handler, std::string(argv[1]), board_size, "right", circle_grid);
        }
        captureStereo(camera_handler, std::string(argv[1]), board_size, circle_grid);
    }catch(std::runtime_error &exp){
        std::cout<<"Could not open the devices"<<std::endl;
        std::cout<<exp.what()<<std::endl;
        return 1;
    }

    return 0;
}
