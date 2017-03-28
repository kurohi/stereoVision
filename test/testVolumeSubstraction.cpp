#include "volume_substraction.hpp"
#include "stereo_3d_tools.hpp"
#include <iostream>
#include <opencv2/highgui.hpp>

using namespace stereo_camera_calibrator;

int main(int argc, char **argv){
  if(argc<5){
    std::cout<<"Need to specify the calibration file and the 2 disparity images to compare"<<std::endl;
    std::cout<<"./testVolumeSubstraction <calibration_file> <disparity1> <disparity2> <rgb_img>"<<std::endl;
    return 1;
  }
  cv::FileStorage disparity1_fs(argv[2], cv::FileStorage::READ);
  cv::FileStorage disparity2_fs(argv[3], cv::FileStorage::READ);
  cv::FileStorage fs(argv[1], cv::FileStorage::READ);
  cv::Mat rgb_img = cv::imread(argv[4]);

  cv::Mat q_matrix;
  fs["disparity_to_depth"] >> q_matrix;
  fs.release();

  cv::Mat disparity1, disparity2;
  disparity1_fs["dframe"] >> disparity1;
  disparity2_fs["dframe"] >> disparity2;

  cv::Mat volume1, volume2;
  Stereo3DTools s3dtools1(disparity1, q_matrix);
  Stereo3DTools s3dtools2(disparity2, q_matrix);

  cv::Point3f ctr_corner(115,-5,720);
  cv::Point3f fbl_corner(-115,200,90);
  cv::Mat rotation_matrix, rotation_matrix_aux;
  s3dtools1.getRotationMatrix(26.5, X_AXIS_INV, rotation_matrix);
  //s3dtools1.getRotationMatrix(180.0, Y_AXIS_INV, rotation_matrix_aux);

  s3dtools1.applyRotationMatrix(rotation_matrix);
  s3dtools2.applyRotationMatrix(rotation_matrix);
  s3dtools1.removeOutsideTheBox(ctr_corner, fbl_corner);
  s3dtools2.removeOutsideTheBox(ctr_corner, fbl_corner);

  //rotation_matrix = rotation_matrix_aux * rotation_matrix;
  s3dtools1.applyRotationMatrix(rotation_matrix);
  s3dtools2.applyRotationMatrix(rotation_matrix);

  s3dtools2.saveComputedPoints("meshpoint.ply", rgb_img);

  volume1 = s3dtools1.getModifiedPointMatrix();
  volume2 = s3dtools2.getModifiedPointMatrix();

  float closest_z = 999.9;
  for(int y=0; y<volume1.rows; y++)
      for(int x=0; x<volume1.cols; x++){
          float voxel = volume1.at<cv::Vec3f>(y,x)[2];
          if((!std::isinf(voxel))&&(!std::isnan(voxel))&&(voxel<closest_z)){
              closest_z = voxel;
          }
      }
  #pragma omp parallel for collapse(2)
  for(int y=0; y<volume1.rows; y++)
      for(int x=0; x<volume1.cols; x++){
          volume1.at<cv::Vec3f>(y,x)[2] -= closest_z;
          volume2.at<cv::Vec3f>(y,x)[2] -= closest_z;
      }

  //cv::reprojectImageTo3D(disparity1, volume1, q_matrix, false);
  //cv::reprojectImageTo3D(disparity2, volume2, q_matrix, false);

  //std::cout<<"Testing volume difference: ";
  //std::cout<<VolumeSubstraction::getVolumeDifference_ml(volume1, volume2)<<std::endl;

  std::cout<<"Testing volume Ratio: ";
  std::cout<<VolumeSubstraction::getVolumeRatio(volume1, volume2)<<std::endl;

  std::cout<<"Testing volume percentage: ";
  std::cout<<100-VolumeSubstraction::getVolumePercentage(volume1, volume2)<<std::endl;
  return 0;
}
