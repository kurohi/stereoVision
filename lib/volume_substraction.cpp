#include "volume_substraction.hpp"

double VolumeSubstraction::angle = 1.0*M_PI/180; 

double VolumeSubstraction::getVolumeDifference_ml(const cv::Mat &volume1, const cv::Mat &volume2){
    double total_volume1, total_volume2;
    getVolumes(volume1, volume2, total_volume1, total_volume2);
    return abs(total_volume1 - total_volume2);
}

double VolumeSubstraction::getVolumeRatio(const cv::Mat &volume1, const cv::Mat &volume2){
    double total_volume1, total_volume2;
    getVolumes(volume1, volume2, total_volume1, total_volume2);
    return total_volume2/total_volume1;
}

double VolumeSubstraction::getVolumePercentage(const cv::Mat &volume1, const cv::Mat &volume2){
    double total_volume1, total_volume2;
    getVolumes(volume1, volume2, total_volume1, total_volume2);
    return total_volume2/total_volume1*100;
}

void VolumeSubstraction::getVolumes(const cv::Mat &volume1, const cv::Mat &volume2, double &total_volume1, double &total_volume2){
    total_volume1 = total_volume2 =0.0001;
    for(int y=0; y<volume1.rows; y++)
        for(int x=0; x<volume1.cols; x++){
            double distance1, distance2;
            distance1 = distance2 = 0;
            if((!std::isnan(volume1.at<cv::Point3f>(y,x).z))&&(!std::isinf(volume1.at<cv::Point3f>(y,x).z))&&(std::abs(volume1.at<cv::Point3f>(y,x).z)>0)){
                distance1 = volume1.at<cv::Point3f>(y,x).z/1000;
            }
            if((!std::isnan(volume2.at<cv::Point3f>(y,x).z))&&(!std::isinf(volume2.at<cv::Point3f>(y,x).z))&&(std::abs(volume2.at<cv::Point3f>(y,x).z)>0)){
                distance2 = volume2.at<cv::Point3f>(y,x).z/1000;
            }
            if(distance1 != distance2){
                double ratio1 = tan(angle)*distance1;
                double ratio2 = tan(angle)*distance2;
                total_volume1 += M_PI*ratio1*ratio1*distance1/3;
                total_volume2 += M_PI*ratio2*ratio2*distance2/3;
            }
        }
}
