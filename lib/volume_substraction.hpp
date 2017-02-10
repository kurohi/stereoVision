#include "volume_substraction.hpp"

double VolumeSubstraction::getVolumeDifference_ml(const cv::Mat &volume1, const cv::Mat &volume2){
    double volume =0.0;
    for(int y=0; y<volume1.rows; y++)
        for(int x=0; x<volume1.cols; x++){
            
        }
}

double VolumeSubstraction::getVolumeRatio(const cv::Mat &volume1, const cv::Mat &volume2){
    double vol1,vol2;
    vol1 = vol2 = 0.0;
    vol1 = cv::sum(volume1);
}
