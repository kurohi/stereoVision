#include "volume_substraction.hpp"
#include <iostream>
#include <opencv2/highgui.hpp>

int main(int argc, char **argv){
  if(argc<4){
    std::cout<<"Need to specify the calibration file and the 2 disparity images to compare"<<std::endl;
    std::cout<<"./testVolumeSubstraction <calibration_file> <disparity1> <disparity2>"<<std::endl;
    return 1;
  }
  cv::Mat disparity1_8u = cv::imread(argv[2], cv::IMREAD_GRAYSCALE);
  cv::Mat disparity2_8u = cv::imread(argv[3], cv::IMREAD_GRAYSCALE);
  cv::FileStorage fs(argv[1], cv::FileStorage::READ);

  cv::Mat q_matrix;
  fs["Q"] >> q_matrix;
  fs.release();

  cv::Mat disparity1, disparity2;
  double minVal, maxVal;
  cv::minMaxLoc(disparity1_8u, &minVal, &maxVal);
  disparity1_8u.convertTo(disparity1, CV_16SC1, 0xEFFF*(maxVal-minVal));

  cv::minMaxLoc(disparity2_8u, &minVal, &maxVal);
  disparity2_8u.convertTo(disparity2, CV_16SC1, 0xEFFF*(maxVal-minVal));

  cv::Mat volume1, volume2;
  cv::reprojectImageTo3D(disparity1, volume1, q_matrix, true);
  cv::reprojectImageTo3D(disparity2, volume2, q_matrix, true);
  
  std::cout<<"Testing volume difference: ";
  std::cout<<VolumeSubstraction::getVolumeDifference_ml(volume1, volume2)<<std::endl;

  std::cout<<"Testing volume Ratio: ";
  std::cout<<VolumeSubstraction::getVolumeRatio(volume1, volume2)<<std::endl;

  std::cout<<"Testing volume percentage: ";
  std::cout<<VolumeSubstraction::getVolumePercentage(volume1, volume2)<<std::endl;
  return 0;
}
