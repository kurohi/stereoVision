#include "volume_substraction.hpp"
#include <iostream>
#include <opencv2/highgui.hpp>

int main(int argc, char **argv){
  if(argc<4){
    std::cout<<"Need to specify the calibration file and the 2 disparity images to compare"<<std::endl;
    std::cout<<"./testVolumeSubstraction <calibration_file> <disparity1> <disparity2>"<<std::endl;
    return 1;
  }
  cv::FileStorage disparity1_fs(argv[2], cv::FileStorage::READ);
  cv::FileStorage disparity2_fs(argv[3], cv::FileStorage::READ);
  cv::FileStorage fs(argv[1], cv::FileStorage::READ);

  cv::Mat q_matrix;
  fs["disparity_to_depth"] >> q_matrix;
  fs.release();

  cv::Mat disparity1, disparity2;
  disparity1_fs["dframe"] >> disparity1;
  disparity2_fs["dframe"] >> disparity2;

  cv::Mat volume1, volume2;
  cv::reprojectImageTo3D(disparity1, volume1, q_matrix, false);
  cv::reprojectImageTo3D(disparity2, volume2, q_matrix, false);
  
  std::cout<<"Testing volume difference: ";
  std::cout<<VolumeSubstraction::getVolumeDifference_ml(volume1, volume2)<<std::endl;

  std::cout<<"Testing volume Ratio: ";
  std::cout<<VolumeSubstraction::getVolumeRatio(volume1, volume2)<<std::endl;

  std::cout<<"Testing volume percentage: ";
  std::cout<<VolumeSubstraction::getVolumePercentage(volume1, volume2)<<std::endl;
  return 0;
}
