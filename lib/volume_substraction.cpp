#include "volume_substraction.hpp"

double VolumeSubstraction::getVolumeDifference_ml(const cv::Mat &volume1, const cv::Mat &volume2){
    double total_volume1, total_volume2;
    getVolumes(volume1, volume2, total_volume1, total_volume2);
    return abs(total_volume1 - total_volume2);
}

double VolumeSubstraction::getVolumeRatio(const cv::Mat &volume1, const cv::Mat &volume2){
    double total_volume1, total_volume2;
    getVolumes(volume1, volume2, total_volume1, total_volume2);
    return total_volume1/total_volume2;
}

double VolumeSubstraction::getVolumePercentage(const cv::Mat &volume1, const cv::Mat &volume2){
    double total_volume1, total_volume2;
    getVolumes(volume1, volume2, total_volume1, total_volume2);
    return total_volume2/total_volume1*100;
}

void VolumeSubstraction::getVolumes(const cv::Mat &volume1, const cv::Mat &volume2, double &total_volume1, double &total_volume2){
    total_volume1 = total_volume2 =0.0;
    for(int y=0; y<volume1.rows; y++)
        for(int x=0; x<volume1.cols; x++){
            total_volume1 += volume1.at<cv::Point3d>(y,x).z*16;
            total_volume2 += volume2.at<cv::Point3d>(y,x).z*16; 
        }
    std::cout<<std::endl;
    std::cout<<"volume1: "<<total_volume1<<std::endl;
    std::cout<<"volume2: "<<total_volume2<<std::endl;

}
