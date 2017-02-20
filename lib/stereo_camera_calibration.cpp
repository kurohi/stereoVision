/**
* @file stereo_camera_calibration.cpp
* @brief library for stereo camera camera calibration.
*/
#include "../include/stereo_camera_calibration.hpp"
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <sstream>
#include <string>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace stereo_tools {

StereoCameraCalibration::StereoCameraCalibration(const std::string& config_file_path, const bool triple_set_calib){
  triple_set = triple_set_calib;
  // open configuration YAML file for stereo calibrartion.
  openConfigFile(config_file_path);

}

StereoCameraCalibration::~StereoCameraCalibration() {
}

void StereoCameraCalibration::openConfigFile(const std::string& config_file_path) {
  cv::FileStorage config_file_storage(config_file_path,
                                          cv::FileStorage::READ);
  if (!config_file_storage.isOpened()) {
    throw std::runtime_error("ERROR-StereoCameraCalibration::openConfigFile: Failed to open config yaml file.");
  }

  image_file_directory_path_ = static_cast<std::string>(
      config_file_storage["image_directory_path"]);

  image_pair_num_ = static_cast<int>(
      config_file_storage["image_pair_num"]);

  image_format_ = static_cast<std::string>(
      config_file_storage["image_format"]);

  chart_corner_rows_num_ = static_cast<int>(
      config_file_storage["chart_corner_rows_num"]);

  chart_corner_cols_num_ = static_cast<int>(
      config_file_storage["chart_corner_cols_num"]);

  chart_square_size_ = static_cast<float>(
      config_file_storage["chart_square_size"]);
  if(triple_set){
    image_left_num_ = static_cast<int>(
        config_file_storage["image_left_num"]);
    image_right_num_ = static_cast<int>(
        config_file_storage["image_right_num"]);
  }


  std::cout << "----------------------------------------" << std::endl;
  std::cout << "config.yaml includes following infomation." << std::endl;
  std::cout << "config_file_path = " << config_file_path << std::endl;
  std::cout << "image_file_directory_path_ = " << image_file_directory_path_ << std::endl;
  std::cout << "image_pair_num_ = " << image_pair_num_ << std::endl;
  if(triple_set){
    std::cout << "image_left_num_ = " << image_left_num_ << std::endl;
    std::cout << "image_right_num_ = " << image_right_num_ << std::endl;
  }
  std::cout << "chart_corner_rows_num_ = " << chart_corner_rows_num_ << std::endl;
  std::cout << "chart_corner_cols_num_ = " << chart_corner_cols_num_ << std::endl;
  std::cout << "chart_square_size_ = " << chart_square_size_ << std::endl;
  std::cout << "----------------------------------------" << std::endl;
}

void StereoCameraCalibration::calibrate(const bool with_view) {
  try {
    // read stereo image pairs
    if(!triple_set){
        readImageFiles();
    }else{
        readTripleImageFiles();
    }

    // detect corner points in the calibration chart.
    findChartCornerPoint(with_view);
    if(triple_set){
        findChartCornerPoint_split(with_view);
    }

    // execute stereo calibrartion assuming images are already read.
    calibStereoCamera();
  } catch (const std::runtime_error& e) {
    std::cout << e.what() << std::endl;
    throw std::runtime_error("ERROR-StereoCameraCalibration::calibrate: Failed to Calibration.");
  }
}

void StereoCameraCalibration::calibrate_circlegrid(const bool with_view) {
  try {
    // read stereo image pairs
    if(!triple_set){
        readImageFiles();
    }else{
        readTripleImageFiles();
    }

    // detect corner points in the calibration chart.
    findChartCirclePoint(with_view);
    if(triple_set){
        findChartCirclePoint_split(with_view);
    }

    // execute stereo calibrartion assuming images are already read.
    calibStereoCamera();
  } catch (const std::runtime_error& e) {
    std::cout << e.what() << std::endl;
    throw std::runtime_error("ERROR-StereoCameraCalibration::calibrate_circlegrid: Failed to Calibration.");
  }
}

void StereoCameraCalibration::writeCalibrationFiles() {
  // left camera yaml file
  std::string camera_param_left_path = image_file_directory_path_ + "camera_param_left.yaml";
  cv::FileStorage camera_param_left(camera_param_left_path,
                                        cv::FileStorage::WRITE);
  if (!camera_param_left.isOpened()) {
    throw std::runtime_error("ERROR-StereoCameraCalibration::writeCalibrationFiles: Failed to open left camera yaml file.");
  }
  camera_param_left << "width" << left_images_.at(0).size().width;
  camera_param_left << "height" << left_images_.at(0).size().height;
  camera_param_left << "intrinsic" << left_intrinsic_;
  camera_param_left << "distortion" << left_distortion_;
  camera_param_left << "extrinsic_rotation" << extrinsic_rotation_;
  camera_param_left << "extrinsic_position" << extrinsic_position_;
  //rectification parameters
  camera_param_left << "rect_rotate" << rect_left_rotate_;
  camera_param_left << "rect_position" << rect_left_position_;
  camera_param_left << "disparity_to_depth" << disp_to_depth_;
  camera_param_left.release();

  // right camera yaml file
  std::string camera_param_right_path = image_file_directory_path_ + "camera_param_right.yaml";
  cv::FileStorage camera_param_right(camera_param_right_path,
                                         cv::FileStorage::WRITE);
  if (!camera_param_right.isOpened()) {
    throw std::runtime_error("ERROR-StereoCameraCalibration::writeCalibrationFiles: Failed to open right camera yaml file.");
  }
  camera_param_right << "width" << right_images_.at(0).size().width;
  camera_param_right << "height" << right_images_.at(0).size().height;
  camera_param_right << "intrinsic" << right_intrinsic_;
  camera_param_right << "distortion" << right_distortion_;
  camera_param_right << "extrinsic_rotation" << extrinsic_rotation_;
  camera_param_right << "extrinsic_position" << extrinsic_position_;
  camera_param_right << "rect_rotate" << rect_right_rotate_;
  camera_param_right << "rect_position" << rect_right_position_;
  camera_param_right << "disparity_to_depth" << disp_to_depth_;
  camera_param_right.release();

  std::cout << "calibration files are generated as bellow." << std::endl;
  std::cout << camera_param_left_path  << std::endl;
  std::cout << camera_param_right_path  << std::endl;
}

void StereoCameraCalibration::readImageFiles() {
  std::cout << "start reading stereo image pairs." << std::endl;
  for (int i = 0; i < image_pair_num_; ++i) {
    // ファイルパスを整形
    std::stringstream left_file_path_stream;
    left_file_path_stream << image_file_directory_path_ <<
        "left_" << std::setw(3) << std::setfill('0') << i << "." << image_format_;

    std::stringstream right_file_path_stream;
    right_file_path_stream << image_file_directory_path_ <<
        "right_" << std::setw(3) << std::setfill('0') << i << "." << image_format_;

    // 画像を読み込みcv::Mat -> std::vectorへと詰める
    //This is how much percent of the image will be ignored 
    //It is commented out to use the program with normal fisheyed camera that dont 
    //need to have its FOV reduced
    /*
    int radio = 200;
    cv::Mat left_image;
    cv::Mat left_image_aux = cv::imread(left_file_path_stream.str());
    cv::Mat mask = cv::Mat::zeros(left_image_aux.size(), CV_8UC3);
    cv::circle(mask, cv::Point(mask.cols/2, mask.rows/2), radio, cv::Scalar(255,255,255), -1, 8);
    int w = left_image_aux.cols;
    int h = left_image_aux.rows;
    left_image = left_image_aux & mask;
    */
    cv::Mat left_image = cv::imread(left_file_path_stream.str());

    if (left_image.empty()) {
      std::cout << left_file_path_stream.str() << std::endl;
      throw std::runtime_error("ERROR-StereoCameraCalibration::readImageFiles: Failed to open left image file.");
    }
    left_images_.push_back(left_image);

    //same preprocessing prototype for the right image
    /*
    cv::Mat right_image_aux = cv::imread(right_file_path_stream.str());
    w = right_image_aux.cols;
    h = right_image_aux.rows;
    cv::Mat right_image;
    right_image = right_image_aux & mask;
    */
    cv::Mat right_image = cv::imread(right_file_path_stream.str());

    if (right_image.empty()) {
      throw std::runtime_error("ERROR-StereoCameraCalibration::readImageFiles: Failed to open right image file.");
    }
    right_images_.push_back(right_image);

    std::cout << "Open file: " << left_file_path_stream.str() << std::endl;
    std::cout << "Open file: " << right_file_path_stream.str() << std::endl;
  }

  std::cout << "finished reading stereo image pairs." << std::endl;
  std::cout << "----------------------------------------" << std::endl;
}

void StereoCameraCalibration::readTripleImageFiles() {
  std::cout << "start reading stereo image pairs." << std::endl;
  for (int i = 0; i < image_left_num_; ++i) {
    // ファイルパスを整形
    std::stringstream left_file_path_stream;
    left_file_path_stream << image_file_directory_path_ <<
        "left_" << std::setw(3) << std::setfill('0') << i << "." << image_format_;

    cv::Mat left_image = cv::imread(left_file_path_stream.str());

    if (left_image.empty()) {
      std::cout << left_file_path_stream.str() << std::endl;
      throw std::runtime_error("ERROR-StereoCameraCalibration::readTripleImageFiles: Failed to open left image file.");
    }
    left_images_.push_back(left_image);

    std::cout << "Open file: " << left_file_path_stream.str() << std::endl;
  }
  for (int i = 0; i < image_right_num_; ++i) {
    // ファイルパスを整形
    std::stringstream right_file_path_stream;
    right_file_path_stream << image_file_directory_path_ <<
        "right_" << std::setw(3) << std::setfill('0') << i << "." << image_format_;

    cv::Mat right_image = cv::imread(right_file_path_stream.str());

    if (right_image.empty()) {
      std::cout << right_file_path_stream.str() << std::endl;
      throw std::runtime_error("ERROR-StereoCameraCalibration::readTripleImageFiles: Failed to open right image file.");
    }
    right_images_.push_back(right_image);

    std::cout << "Open file: " << right_file_path_stream.str() << std::endl;
  }
  for (int i = 0; i < image_pair_num_; ++i) {
    //left portion of the stereo pair
    std::stringstream stereo_left_file_path_stream;
    stereo_left_file_path_stream << image_file_directory_path_ <<
        "stereo_left_" << std::setw(3) << std::setfill('0') << i << "." << image_format_;

    cv::Mat stereo_left_image = cv::imread(stereo_left_file_path_stream.str());

    if (stereo_left_image.empty()) {
      std::cout << stereo_left_file_path_stream.str() << std::endl;
      throw std::runtime_error("ERROR-StereoCameraCalibration::readTripleImageFiles: Failed to open stereo image file.");
    }
    stereo_left_images_.push_back(stereo_left_image);

    std::cout << "Open file: " << stereo_left_file_path_stream.str() << std::endl;

    //right portion of the stereo pair
    std::stringstream stereo_right_file_path_stream;
    stereo_right_file_path_stream << image_file_directory_path_ <<
        "stereo_right_" << std::setw(3) << std::setfill('0') << i << "." << image_format_;

    cv::Mat stereo_right_image = cv::imread(stereo_right_file_path_stream.str());

    if (stereo_right_image.empty()) {
      std::cout << stereo_right_file_path_stream.str() << std::endl;
      throw std::runtime_error("ERROR-StereoCameraCalibration::readTripleImageFiles: Failed to open stereo image file.");
    }
    stereo_right_images_.push_back(stereo_right_image);

    std::cout << "Open file: " << stereo_right_file_path_stream.str() << std::endl;

  }
  std::cout << "finished reading stereo image pairs." << std::endl;
  std::cout << "----------------------------------------" << std::endl;
}

void StereoCameraCalibration::findChartCornerPoint_split(const bool with_view) {
  std::cout << "start detection of corner points in the chart for the left and right only." << std::endl;

  // キャリブレーションチャートの格子点の数(横、縦)
  cv::Size chart_corner_num = cv::Size(chart_corner_cols_num_,
                                          chart_corner_rows_num_);
  // 2次元コーナー点を検出する
  int end = image_left_num_;
  for (int i = 0; i < end; ++i) {

    bool left_pattern_found = false;
    #ifdef WITH_FISHEYE
    std::vector<cv::Point2d> left_corner_points_aux;
    #else
    std::vector<cv::Point2f> left_corner_points_aux;
    #endif

    try {
      left_pattern_found = cv::findChessboardCorners(
          left_images_.at(i), chart_corner_num, left_corner_points_aux, cv::CALIB_CB_ADAPTIVE_THRESH|cv::CALIB_CB_NORMALIZE_IMAGE);
    } catch(cv::Exception& e) {
        std::cout<<"chessboard not found on left image with error"<<std::endl;
        image_left_num_--;
        continue;
    }
    

    // コーナー点が見つかったら、サブピクセル精度でさらにコーナーを探す
    if (left_pattern_found) {
      // BGRの3チャンネルの画像をグレースケールの1チャンネルに変換
      cv::Mat left_gray_image;
      cv::cvtColor(left_images_.at(i), left_gray_image, CV_BGR2GRAY);
      size_t left_corner_points_auxsize = left_corner_points_aux.size();
      #ifdef WITH_FISHEYE
      std::vector<cv::Point2d> left_corner_points_float;
      for(size_t j=0; j<left_corner_points_auxsize; j++){
        left_corner_points_float.push_back(cv::Point2d(left_corner_points_aux.at(j).x, left_corner_points_aux.at(j).y));
      }
      #else
      std::vector<cv::Point2f> left_corner_points_float;
      for(size_t j=0; j<left_corner_points_auxsize; j++){
        left_corner_points_float.push_back(cv::Point2f(left_corner_points_aux.at(j).x, left_corner_points_aux.at(j).y));
      }
      #endif

      //サブピクセル精度でさらにコーナーを探す
      cv::cornerSubPix(left_gray_image,
                         left_corner_points_float,
                         cv::Size(7, 7),
                         cv::Size(-1, -1),
                         cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 30, 0.01));

      // コーナー点を画像に描写
      if(with_view){
          cv::drawChessboardCorners(left_images_.at(i), chart_corner_num,
              left_corner_points_aux, left_pattern_found);

          // ステレオ画像を左右に並べて表示
          std::cout<<"now on index for left: "<<i<<std::endl;
          cv::namedWindow("show corner points", cv::WINDOW_NORMAL);
          cv::imshow("show corner points", left_images_.at(i));
          cv::waitKey(0);
      }

      for(unsigned int j=0; j<left_corner_points_auxsize; j++){
        #ifdef WITH_FISHEYE
        left_corner_points_aux.at(j) = cv::Point2d(left_corner_points_float.at(j).x, left_corner_points_float.at(j).y);
        #else
        left_corner_points_aux.at(j) = cv::Point2f(left_corner_points_float.at(j).x, left_corner_points_float.at(j).y);
        #endif
      }
      left_corner_points_.push_back(left_corner_points_aux);  
    } else {
      // コーナー点が見つからない場合は、エラーを返す
      image_left_num_--;
      continue;
    }
  }
  std::cout<<"Number of valid left images: "<<image_left_num_<<std::endl;

  end = image_right_num_;
  for (int i = 0; i < end; ++i) {

    bool right_pattern_found = false;
    #ifdef WITH_FISHEYE
    std::vector<cv::Point2d> right_corner_points_aux;
    #else
    std::vector<cv::Point2f> right_corner_points_aux;
    #endif

    try {
      right_pattern_found = cv::findChessboardCorners(
          right_images_.at(i), chart_corner_num, right_corner_points_aux, cv::CALIB_CB_ADAPTIVE_THRESH|cv::CALIB_CB_NORMALIZE_IMAGE);
    } catch(cv::Exception& e) {
        std::cout<<"chessboard not found on left image with error"<<std::endl;
        image_left_num_--;
        continue;
    }
    

    // コーナー点が見つかったら、サブピクセル精度でさらにコーナーを探す
    if (right_pattern_found) {
      // BGRの3チャンネルの画像をグレースケールの1チャンネルに変換
      cv::Mat right_gray_image;
      cv::cvtColor(right_images_.at(i), right_gray_image, CV_BGR2GRAY);
      size_t right_corner_points_auxsize = right_corner_points_aux.size();
      #ifdef WITH_FISHEYE
      std::vector<cv::Point2d> right_corner_points_float;
      for(size_t j=0; j<right_corner_points_auxsize; j++){
        right_corner_points_float.push_back(cv::Point2d(right_corner_points_aux.at(j).x, right_corner_points_aux.at(j).y));
      }
      #else
      std::vector<cv::Point2f> right_corner_points_float;
      for(size_t j=0; j<right_corner_points_auxsize; j++){
        right_corner_points_float.push_back(cv::Point2f(right_corner_points_aux.at(j).x, right_corner_points_aux.at(j).y));
      }
      #endif

      //サブピクセル精度でさらにコーナーを探す
      cv::cornerSubPix(right_gray_image,
                         right_corner_points_float,
                         cv::Size(7, 7),
                         cv::Size(-1, -1),
                         cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 30, 0.01));

      // コーナー点を画像に描写
      if(with_view){
          cv::drawChessboardCorners(right_images_.at(i), chart_corner_num,
              right_corner_points_aux, right_pattern_found);

          // ステレオ画像を左右に並べて表示
          std::cout<<"now on index for right: "<<i<<std::endl;
          cv::namedWindow("show corner points", cv::WINDOW_NORMAL);
          cv::imshow("show corner points", right_images_.at(i));
          cv::waitKey(0);
      }

      for(unsigned int j=0; j<right_corner_points_auxsize; j++){
        #ifdef WITH_FISHEYE
        right_corner_points_aux.at(j) = cv::Point2d(right_corner_points_float.at(j).x, right_corner_points_float.at(j).y);
        #else
        right_corner_points_aux.at(j) = cv::Point2f(right_corner_points_float.at(j).x, right_corner_points_float.at(j).y);
        #endif
      }
      right_corner_points_.push_back(right_corner_points_aux);
    } else {
      // コーナー点が見つからない場合は、エラーを返す
      image_right_num_--;
      continue;
    }
  }
  std::cout<<"Number of valid right images: "<<image_right_num_<<std::endl;


  std::cout << "finished detection of corner points in the chart." << std::endl;
  std::cout << "----------------------------------------" << std::endl;
}

void StereoCameraCalibration::findChartCornerPoint(const bool with_view) {
  std::cout << "start detection of corner points in the chart." << std::endl;

  // キャリブレーションチャートの格子点の数(横、縦)
  cv::Size chart_corner_num = cv::Size(chart_corner_cols_num_,
                                          chart_corner_rows_num_);

  // 2次元コーナー点を検出する
  int end = image_pair_num_;
  for (int i = 0; i < end; ++i) {

    bool left_pattern_found = false;
    bool right_pattern_found = false;
    #ifdef WITH_FISHEYE
    std::vector<cv::Point2d> stereo_left_corner_points_aux;
    std::vector<cv::Point2d> stereo_right_corner_points_aux;
    #else
    std::vector<cv::Point2f> stereo_left_corner_points_aux;
    std::vector<cv::Point2f> stereo_right_corner_points_aux;
    #endif

    try {
      cv::Mat left_img,right_img;
      if(triple_set){
        left_img = stereo_left_images_.at(i);
        right_img = stereo_right_images_.at(i);
      }else{
        left_img = left_images_.at(i);
        right_img = right_images_.at(i);
      }
      left_pattern_found = cv::findChessboardCorners(
          left_img, chart_corner_num, stereo_left_corner_points_aux, cv::CALIB_CB_ADAPTIVE_THRESH|cv::CALIB_CB_NORMALIZE_IMAGE);
      right_pattern_found = cv::findChessboardCorners(
          right_img, chart_corner_num, stereo_right_corner_points_aux, cv::CALIB_CB_ADAPTIVE_THRESH|cv::CALIB_CB_NORMALIZE_IMAGE);
    } catch(cv::Exception& e) {
        std::cout<<"chessboards not found with error"<<std::endl;
        image_pair_num_--;
        continue;
    }
    

    // コーナー点が見つかったら、サブピクセル精度でさらにコーナーを探す
    if (left_pattern_found && right_pattern_found) {
      // BGRの3チャンネルの画像をグレースケールの1チャンネルに変換
      cv::Mat left_gray_image;
      cv::Mat right_gray_image;
      if(triple_set){
        cv::cvtColor(stereo_left_images_.at(i), left_gray_image, CV_BGR2GRAY);
        cv::cvtColor(stereo_right_images_.at(i), right_gray_image, CV_BGR2GRAY);
      }else{
        cv::cvtColor(left_images_.at(i), left_gray_image, CV_BGR2GRAY);
        cv::cvtColor(right_images_.at(i), right_gray_image, CV_BGR2GRAY);
      }
      size_t stereo_left_corner_points_auxsize = stereo_left_corner_points_aux.size();
      size_t stereo_right_corner_points_auxsize = stereo_right_corner_points_aux.size();
      #ifdef WITH_FISHEYE
      std::vector<cv::Point2d> stereo_left_corner_points_float;
      std::vector<cv::Point2d> stereo_right_corner_points_float;
      for(size_t j=0; j<stereo_left_corner_points_auxsize; j++){
        stereo_left_corner_points_float.push_back(cv::Point2d(stereo_left_corner_points_aux.at(j).x, stereo_left_corner_points_aux.at(j).y));
      }
      for(size_t j=0; j<stereo_right_corner_points_auxsize; j++){
        stereo_right_corner_points_float.push_back(cv::Point2d(stereo_right_corner_points_aux.at(j).x, stereo_right_corner_points_aux.at(j).y));
      }
      #else
      std::vector<cv::Point2f> stereo_left_corner_points_float;
      std::vector<cv::Point2f> stereo_right_corner_points_float;
      for(size_t j=0; j<stereo_left_corner_points_auxsize; j++){
        stereo_left_corner_points_float.push_back(cv::Point2f(stereo_left_corner_points_aux.at(j).x, stereo_left_corner_points_aux.at(j).y));
      }
      for(size_t j=0; j<stereo_right_corner_points_auxsize; j++){
        stereo_right_corner_points_float.push_back(cv::Point2f(stereo_right_corner_points_aux.at(j).x, stereo_right_corner_points_aux.at(j).y));
      }
      #endif

      //サブピクセル精度でさらにコーナーを探す
      cv::cornerSubPix(left_gray_image,
                         stereo_left_corner_points_float,
                         cv::Size(7, 7),
                         cv::Size(-1, -1),
                         cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 30, 0.01));
      cv::cornerSubPix(right_gray_image,
                         stereo_right_corner_points_float,
                         cv::Size(7, 7),
                         cv::Size(-1, -1),
                         cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 30, 0.01));

      // コーナー点を画像に描写
      if(with_view){
          if(triple_set){
              cv::drawChessboardCorners(stereo_left_images_.at(i), chart_corner_num,
                  stereo_left_corner_points_aux, left_pattern_found);
              cv::drawChessboardCorners(stereo_right_images_.at(i), chart_corner_num,
                  stereo_right_corner_points_aux, right_pattern_found);

          }else{
              cv::drawChessboardCorners(left_images_.at(i), chart_corner_num,
                  stereo_left_corner_points_aux, left_pattern_found);
              cv::drawChessboardCorners(right_images_.at(i), chart_corner_num,
                  stereo_right_corner_points_aux, right_pattern_found);
          }

          // ステレオ画像を左右に並べて表示
          std::cout<<"now on index: "<<i<<std::endl;
          cv::Mat show_image;
          cv::namedWindow("show corner points", cv::WINDOW_NORMAL);
          if(triple_set){
              cv::hconcat(stereo_left_images_.at(i), stereo_right_images_.at(i), show_image);
          }else{
              cv::hconcat(left_images_.at(i), right_images_.at(i), show_image);
          }
          cv::imshow("show corner points", show_image);
          cv::waitKey(0);
      }

      #ifdef WITH_FISHEYE
      for(size_t j=0; j<stereo_left_corner_points_auxsize; j++){
        stereo_left_corner_points_aux.at(j) = cv::Point2d(stereo_left_corner_points_float.at(j).x, stereo_left_corner_points_float.at(j).y);
      }
      for(size_t j=0; j<stereo_right_corner_points_auxsize; j++){
        stereo_right_corner_points_aux.at(j) = cv::Point2d(stereo_right_corner_points_float.at(j).x, stereo_right_corner_points_float.at(j).y);
      }
      #else
      for(size_t j=0; j<stereo_left_corner_points_auxsize; j++){
        stereo_left_corner_points_aux.at(j) = cv::Point2f(stereo_left_corner_points_float.at(j).x, stereo_left_corner_points_float.at(j).y);
      }
      for(size_t j=0; j<stereo_right_corner_points_auxsize; j++){
        stereo_right_corner_points_aux.at(j) = cv::Point2f(stereo_right_corner_points_float.at(j).x, stereo_right_corner_points_float.at(j).y);
      }
      #endif
      if(triple_set){
          stereo_left_corner_points_.push_back(stereo_left_corner_points_aux);
          stereo_right_corner_points_.push_back(stereo_right_corner_points_aux);
      }else{
          left_corner_points_.push_back(stereo_left_corner_points_aux);
          right_corner_points_.push_back(stereo_right_corner_points_aux);
      }
    } else {
      // コーナー点が見つからない場合は、エラーを返す
      image_pair_num_--;
      continue;
    }
  }
  std::cout<<"Number of valid images: "<<image_pair_num_<<std::endl;


  // 3次元でのコーナー点を生成
  //float width = (chart_corner_num.width-1)*chart_square_size_;
  //float height = (chart_corner_num.height-1)*chart_square_size_;
  for (int i = 0; i < image_pair_num_; i++) {
    #ifdef WITH_FISHEYE
    std::vector<cv::Point3d> corner_points_3d_aux;
    float half_width = (chart_corner_num.width-1)*chart_square_size_/2.0;
    float half_height = (chart_corner_num.height-1)*chart_square_size_/2.0;
    #else
    std::vector<cv::Point3f> corner_points_3d_aux;
    #endif
    for(int k = 0; k < chart_corner_num.height; k++) {
      for(int j = 0; j < chart_corner_num.width; j++ ){
        #ifdef WITH_FISHEYE
        corner_points_3d_aux.push_back(
            cv::Point3d((j * chart_square_size_)-half_width, (k * chart_square_size_)-half_height, 0.0));
        #else
        corner_points_3d_aux.push_back(
            cv::Point3f((j * chart_square_size_), (k * chart_square_size_), 0.0));
        #endif
      }
    }
    corner_points_3d_.push_back(corner_points_3d_aux);
  }

  std::cout << "finished detection of corner points in the chart." << std::endl;
  std::cout << "----------------------------------------" << std::endl;
}

void StereoCameraCalibration::findChartCirclePoint_split(const bool with_view) {
  std::cout << "start detection of corner points in the chart for the left and right only." << std::endl;

  // キャリブレーションチャートの格子点の数(横、縦)
  cv::Size chart_pattern_size = cv::Size(chart_corner_cols_num_,
                                          chart_corner_rows_num_);
  // 2次元コーナー点を検出する
  int end = image_left_num_;
  for (int i = 0; i < end; ++i) {

    bool left_pattern_found = false;
    #ifdef WITH_FISHEYE
    std::vector<cv::Point2d> left_corner_points_aux;
    #else
    std::vector<cv::Point2f> left_corner_points_aux;
    #endif

    try {
      left_pattern_found = cv::findCirclesGrid(
          left_images_.at(i), chart_pattern_size, left_corner_points_aux, cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING);
    } catch(cv::Exception& e) {
        std::cout<<"chessboard not found on left image with error"<<std::endl;
        image_left_num_--;
        continue;
    }
    

    // コーナー点が見つかったら、サブピクセル精度でさらにコーナーを探す
    if (left_pattern_found) {
      // コーナー点を画像に描写
      if(with_view){
          cv::drawChessboardCorners(left_images_.at(i), chart_pattern_size,
              left_corner_points_aux, left_pattern_found);

          // ステレオ画像を左右に並べて表示
          std::cout<<"now on index for left: "<<i<<std::endl;
          cv::namedWindow("show corner points", cv::WINDOW_NORMAL);
          cv::imshow("show corner points", left_images_.at(i));
          cv::waitKey(0);
      }
      left_corner_points_.push_back(left_corner_points_aux);
    } else {
      // コーナー点が見つからない場合は、エラーを返す
      image_left_num_--;
      continue;
    }
  }
  std::cout<<"Number of valid left images: "<<image_left_num_<<std::endl;

  end = image_right_num_;
  for (int i = 0; i < end; ++i) {

    bool right_pattern_found = false;
    #ifdef WITH_FISHEYE
    std::vector<cv::Point2d> right_corner_points_aux;
    #else
    std::vector<cv::Point2f> right_corner_points_aux;
    #endif

    try {
      right_pattern_found = cv::findCirclesGrid(
          right_images_.at(i), chart_pattern_size, right_corner_points_aux, cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING);
    } catch(cv::Exception& e) {
        std::cout<<"chessboard not found on left image with error"<<std::endl;
        image_left_num_--;
        continue;
    }
    

    // コーナー点が見つかったら、サブピクセル精度でさらにコーナーを探す
    if (right_pattern_found) {
      // コーナー点を画像に描写
      if(with_view){
          cv::drawChessboardCorners(right_images_.at(i), chart_pattern_size,
              right_corner_points_aux, right_pattern_found);

          // ステレオ画像を左右に並べて表示
          std::cout<<"now on index for right: "<<i<<std::endl;
          cv::namedWindow("show corner points", cv::WINDOW_NORMAL);
          cv::imshow("show corner points", right_images_.at(i));
          cv::waitKey(0);
      }
      right_corner_points_.push_back(right_corner_points_aux);
    } else {
      // コーナー点が見つからない場合は、エラーを返す
      image_right_num_--;
      continue;
    }
  }
  std::cout<<"Number of valid right images: "<<image_right_num_<<std::endl;


  std::cout << "finished detection of corner points in the chart." << std::endl;
  std::cout << "----------------------------------------" << std::endl;
}
void StereoCameraCalibration::findChartCirclePoint(const bool with_view) {
  std::cout << "start detection of corner points in the chart." << std::endl;

  // キャリブレーションチャートの格子点の数(横、縦)
  cv::Size chart_pattern_size = cv::Size(chart_corner_cols_num_,
                                          chart_corner_rows_num_);
  // 2次元コーナー点を検出する
  int end = image_pair_num_;
  for (int i = 0; i < end; ++i) {

    bool left_pattern_found = false;
    bool right_pattern_found = false;
    #ifdef WITH_FISHEYE
    std::vector<cv::Point2d> stereo_left_corner_points_aux;
    std::vector<cv::Point2d> stereo_right_corner_points_aux;
    #else
    std::vector<cv::Point2f> stereo_left_corner_points_aux;
    std::vector<cv::Point2f> stereo_right_corner_points_aux;
    #endif

    try {
      cv::Mat left_img,right_img;
      if(triple_set){
        left_img = stereo_left_images_.at(i);
        right_img = stereo_right_images_.at(i);
      }else{
        left_img = left_images_.at(i);
        right_img = right_images_.at(i);
      }left_pattern_found = cv::findCirclesGrid(
          left_img, chart_pattern_size, stereo_left_corner_points_aux, cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING);
      right_pattern_found = cv::findCirclesGrid(
          right_img, chart_pattern_size, stereo_right_corner_points_aux, cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING);
    } catch(cv::Exception& e) {
        std::cout<<"chessboards not found with error"<<std::endl;
        image_pair_num_--;
        continue;
    }
    

    if (left_pattern_found && right_pattern_found) {

      if(with_view){
          cv::Mat show_image;
          if(triple_set){
              cv::drawChessboardCorners(stereo_left_images_.at(i), chart_pattern_size,
                  stereo_left_corner_points_aux, left_pattern_found);
              cv::drawChessboardCorners(stereo_right_images_.at(i), chart_pattern_size,
                  stereo_right_corner_points_aux, right_pattern_found);
              cv::hconcat(stereo_left_images_.at(i), stereo_right_images_.at(i), show_image);
          }else{
              cv::drawChessboardCorners(left_images_.at(i), chart_pattern_size,
                  stereo_left_corner_points_aux, left_pattern_found);
              cv::drawChessboardCorners(right_images_.at(i), chart_pattern_size,
                  stereo_right_corner_points_aux, right_pattern_found);
              cv::hconcat(left_images_.at(i), right_images_.at(i), show_image);
          }

          std::cout<<"now on index: "<<i<<std::endl;
          cv::namedWindow("show center points", cv::WINDOW_NORMAL);
          cv::imshow("show center points", show_image);
          cv::waitKey(0);
      }

      stereo_left_corner_points_.push_back(stereo_left_corner_points_aux);
      stereo_right_corner_points_.push_back(stereo_right_corner_points_aux);
    } else {
      // コーナー点が見つからない場合は、エラーを返す
      image_pair_num_--;
      continue;
    }
  }
  std::cout<<"Number of valid images: "<<image_pair_num_<<std::endl;


  // 3次元でのコーナー点を生成
  //float width = (chart_corner_num.width-1)*chart_square_size_;
  //float height = (chart_corner_num.height-1)*chart_square_size_;
  int biggest_num = image_left_num_;
  if(image_pair_num_ > biggest_num){
        biggest_num = image_pair_num_;
  }
  if(image_right_num_ > biggest_num){
        biggest_num = image_right_num_;
  }
  
  for (int i = 0; i < biggest_num; i++) {
    #ifdef WITH_FISHEYE
    std::vector<cv::Point3d> corner_points_3d_aux;
    #else
    std::vector<cv::Point3f> corner_points_3d_aux;
    #endif
    for(int k = 0; k < chart_pattern_size.height; k++) {
      for(int j = 0; j < chart_pattern_size.width; j++ ){
        #ifdef WITH_FISHEYE
        corner_points_3d_aux.push_back(
            cv::Point3d((j * chart_square_size_), (k * chart_square_size_), 0.0));
        #else
        corner_points_3d_aux.push_back(
            cv::Point3f((j * chart_square_size_), (k * chart_square_size_), 0.0));
        #endif
      }
    }
    corner_points_3d_.push_back(corner_points_3d_aux);
  }

  std::cout << "finished detection of center points in the chart." << std::endl;
  std::cout << "----------------------------------------" << std::endl;
}

void StereoCameraCalibration::calibStereoCamera() {
  std::cout << "start calibration." << std::endl;

  // 外部パラメータは左カメラを基準とし、キャリブ結果には右から見た左カメラの位置・姿勢が入る
  try {
    if(!triple_set){
        image_left_num_ = image_right_num_ = image_pair_num_;
    }
    #ifdef WITH_FISHEYE
    std::vector<std::vector<cv::Point3d> >::const_iterator first = corner_points_3d_.begin();
    std::vector<std::vector<cv::Point3d> >::const_iterator last = corner_points_3d_.begin()+image_left_num_;
    std::vector<std::vector<cv::Point3d> >corner_points_3d_adjusted(first,last);
    #else
    std::vector<std::vector<cv::Point3f> >::const_iterator first = corner_points_3d_.begin();
    std::vector<std::vector<cv::Point3f> >::const_iterator last = corner_points_3d_.begin()+image_left_num_;
    std::vector<std::vector<cv::Point3f> >corner_points_3d_adjusted(first,last);
    #endif

    double error;
    int flag = 0;
    #ifdef WITH_FISHEYE
    //flag |= cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
    //flag |= cv::fisheye::CALIB_CHECK_COND;
    //flag |= cv::fisheye::CALIB_FIX_SKEW;
    //flag |= cv::fisheye::CALIB_FIX_K1;
    //flag |= cv::fisheye::CALIB_FIX_K2;
    //flag |= cv::fisheye::CALIB_FIX_K3;
    //flag |= cv::fisheye::CALIB_FIX_K4;
    error = cv::fisheye::calibrate(corner_points_3d_adjusted,
                                    left_corner_points_,
                                    left_images_.at(0).size(),
                                    left_intrinsic_,
                                    left_distortion_,
                                    extrinsic_rotation_,
                                    extrinsic_position_,
                                    flag,
                                    cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-8));
    #else
    flag |= CV_CALIB_RATIONAL_MODEL;
    //flag |= cv::CALIB_THIN_PRISM_MODEL;
    flag |= cv::CALIB_TILTED_MODEL;
    error = cv::calibrateCamera(corner_points_3d_adjusted,
                                 left_corner_points_,
                                 left_images_.at(0).size(),
                                 left_intrinsic_,
                                 left_distortion_,
                                 extrinsic_rotation_,
                                 extrinsic_position_,
                                 flag,
                                 cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 1000, 1e-12));
    #endif
    std::cout<<"Left camera calibration error: " << error <<std::endl;

    last = corner_points_3d_.begin()+image_right_num_;

    #ifdef WITH_FISHEYE
    corner_points_3d_adjusted = std::vector<std::vector<cv::Point3d> >(first,last);
    error = cv::fisheye::calibrate(corner_points_3d_adjusted,
                                    right_corner_points_,
                                    right_images_.at(0).size(),
                                    right_intrinsic_,
                                    right_distortion_,
                                    extrinsic_rotation_,
                                    extrinsic_position_,
                                    flag,
                                    cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-8));
    #else
    corner_points_3d_adjusted = std::vector<std::vector<cv::Point3f> >(first,last);
    error = cv::calibrateCamera(corner_points_3d_adjusted,
                                right_corner_points_,
                                right_images_.at(0).size(),
                                right_intrinsic_,
                                right_distortion_,
                                extrinsic_rotation_,
                                extrinsic_position_,
                                flag,
                                cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 1000, 1e-12));
    #endif
    std::cout<<"Right camera calibration error: " << error <<std::endl; 


    cv::Mat E,F;
    last = corner_points_3d_.begin()+image_pair_num_;
    #ifdef WITH_FISHEYE
    corner_points_3d_adjusted = std::vector<std::vector<cv::Point3d> >(first,last);
    flag |= cv::fisheye::CALIB_USE_INTRINSIC_GUESS;
    if(triple_set){
        error = cv::fisheye::stereoCalibrate(corner_points_3d_adjusted,
                                            stereo_left_corner_points_,
                                            stereo_right_corner_points_,
                                            left_intrinsic_,
                                            left_distortion_,
                                            right_intrinsic_,
                                            right_distortion_,
                                            stereo_left_images_.at(0).size(),
                                            extrinsic_rotation_,
                                            extrinsic_position_,
                                            flag,
                                            cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 1000, 1e-12));

    }else{
        error = cv::fisheye::stereoCalibrate(corner_points_3d_adjusted,
                                            left_corner_points_,
                                            right_corner_points_,
                                            left_intrinsic_,
                                            left_distortion_,
                                            right_intrinsic_,
                                            right_distortion_,
                                            left_images_.at(0).size(),
                                            extrinsic_rotation_,
                                            extrinsic_position_,
                                            flag,
                                            cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 1000, 1e-12));

    }
    #else
    corner_points_3d_adjusted = std::vector<std::vector<cv::Point3f> >(first,last);
    flag |= cv::CALIB_USE_INTRINSIC_GUESS;
    if(triple_set){
        error = cv::stereoCalibrate(corner_points_3d_adjusted,
                                stereo_left_corner_points_,
                                stereo_right_corner_points_,
                                left_intrinsic_,
                                left_distortion_,
                                right_intrinsic_,
                                right_distortion_,
                                stereo_left_images_.at(0).size(),
                                extrinsic_rotation_,
                                extrinsic_position_,
                                E,
                                F,
                                flag,
                                cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 1000, 1e-12));
    }else{
        error = cv::stereoCalibrate(corner_points_3d_adjusted,
                                left_corner_points_,
                                right_corner_points_,
                                left_intrinsic_,
                                left_distortion_,
                                right_intrinsic_,
                                right_distortion_,
                                left_images_.at(0).size(),
                                extrinsic_rotation_,
                                extrinsic_position_,
                                E,
                                F,
                                flag,
                                cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 1000, 1e-12));

    }
    #endif
    std::cout<<"Stereo camera error: "<<error<<std::endl;

    calculateRectificationMatrixes();
  } catch(cv::Exception& e) {
    std::cout << e.what() << std::endl;
    throw std::runtime_error("ERROR-StereoCameraCalibration::calibStereoCamera: Failed to cv::stereoCalibrate.");
  }

  std::cout << "finished calibration." << std::endl;
  std::cout << "----------------------------------------" << std::endl;
}


void StereoCameraCalibration::calculateRectificationMatrixes(){
    #ifdef WITH_FISHEYE
    cv::fisheye::stereoRectify(left_intrinsic_, left_distortion_, right_intrinsic_, right_distortion_,
                           left_images_.at(0).size(), extrinsic_rotation_, extrinsic_position_,
                           rect_left_rotate_, rect_right_rotate_, rect_left_position_, rect_right_position_,
                           disp_to_depth_, CV_CALIB_ZERO_DISPARITY);
    #else
    cv::stereoRectify(left_intrinsic_, left_distortion_, right_intrinsic_, right_distortion_, 
                        left_images_.at(0).size(), extrinsic_rotation_, extrinsic_position_,
                        rect_left_rotate_, rect_right_rotate_, rect_left_position_, rect_right_position_, 
                        disp_to_depth_, CV_CALIB_ZERO_DISPARITY, 1);
    #endif
    
}

void StereoCameraCalibration::rectifyImages(){
    cv::Mat undistorted_left, undistorted_right;
    cv::Mat showimg;

    //variable for holding the 2 remap matrix for each camera                            
    cv::Mat rmap[2][2];

    //get the remap matrix using the rectification results
    #ifdef WITH_FISHEYE
    cv::fisheye::initUndistortRectifyMap(left_intrinsic_, left_distortion_,
                                        rect_left_rotate_, rect_left_position_, left_images_.at(0).size(), CV_16SC2, rmap[0][0], rmap[0][1]);
    cv::fisheye::initUndistortRectifyMap(right_intrinsic_, right_distortion_,
                                        rect_right_rotate_, rect_right_position_, right_images_.at(0).size(), CV_16SC2, rmap[1][0], rmap[0][1]);
    #else
    cv::initUndistortRectifyMap(left_intrinsic_, left_distortion_, 
                                        rect_left_rotate_, rect_left_position_, left_images_.at(0).size(), CV_16SC2, rmap[0][0], rmap[0][1]);
    cv::initUndistortRectifyMap(right_intrinsic_, right_distortion_, 
                                        rect_right_rotate_, rect_right_position_, right_images_.at(0).size(), CV_16SC2, rmap[1][0], rmap[1][1]);
    #endif

    //remap the camera images into the coordinates for only one picture
    if(triple_set){
        cv::remap(stereo_left_images_.at(0), undistorted_left, rmap[0][0], rmap[0][1], CV_INTER_LINEAR);
        cv::remap(stereo_right_images_.at(0), undistorted_right, rmap[1][0], rmap[1][1], CV_INTER_LINEAR);
    }else{
        cv::remap(left_images_.at(0), undistorted_left, rmap[0][0], rmap[0][1], CV_INTER_LINEAR);
        cv::remap(right_images_.at(0), undistorted_right, rmap[1][0], rmap[1][1], CV_INTER_LINEAR);
    }
    
    //attach the left and right pictures together
    cv::hconcat(undistorted_left, undistorted_right, showimg);
    for(int i=0; i<10; i++){
        cv::line(showimg, cv::Point(0, (showimg.rows-1)*i/10), cv::Point(showimg.cols-1, (showimg.rows-1)*i/10), cv::Scalar(0,255,0), 2,8,0);
    }
    cv::imshow("Undistorted images", showimg);
    cv::imwrite("rectified_images.png", showimg);
    cv::waitKey();
}
}  // end of namespace stereo_tools
