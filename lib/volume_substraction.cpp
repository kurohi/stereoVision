#include "volume_substraction.hpp"

double VolumeSubstraction::angle = 0.11*M_PI/180; 
double VolumeSubstraction::unseen_volume = 588372.3;

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
    double percent_tmp = total_volume2/total_volume1*100;
    if((percent_tmp == 100)){
        return 0;
    }else if((percent_tmp < 100)&&(percent_tmp >= 85)){
        return percent_tmp*8/85;
    }else if((percent_tmp < 85)&&(percent_tmp >= 70)){
        return percent_tmp*16/70;
    }else if((percent_tmp < 70)&&(percent_tmp >= 57)){
        return percent_tmp*24/57;
    }else if((percent_tmp < 57)&&(percent_tmp >= 46)){
        return percent_tmp*32/46;
    }else if((percent_tmp < 46)&&(percent_tmp >= 41)){
        return percent_tmp*40/41;
    }else if((percent_tmp < 41)&&(percent_tmp >= 34)){
        return percent_tmp*48/34;
    }else{
        return percent_tmp*64/30;
    }
}

void VolumeSubstraction::getVolumes(const cv::Mat &volume1, const cv::Mat &volume2, double &total_volume1, double &total_volume2){
    total_volume1 = total_volume2 =0.0001;
    for(int y=0; y<volume1.rows; y++)
        for(int x=0; x<volume1.cols; x++){
            double distance1, distance2;
            distance1 = distance2 = 0;
            if((!std::isnan(volume1.at<cv::Point3f>(y,x).z))&&(!std::isinf(volume1.at<cv::Point3f>(y,x).z))&&(std::abs(volume1.at<cv::Point3f>(y,x).z)>0)){
                distance1 = volume1.at<cv::Point3f>(y,x).z;
            }
            if((!std::isnan(volume2.at<cv::Point3f>(y,x).z))&&(!std::isinf(volume2.at<cv::Point3f>(y,x).z))&&(std::abs(volume2.at<cv::Point3f>(y,x).z)>0)){
                distance2 = volume2.at<cv::Point3f>(y,x).z;
            }
            if(distance1 != distance2){
                double ratio1 = tan(angle)*distance1;
                double ratio2 = tan(angle)*distance2;
                total_volume1 += M_PI*ratio1*ratio1*distance1/3;
                total_volume2 += M_PI*ratio2*ratio2*distance2/3;
            }
        }
    total_volume1 += unseen_volume;
    total_volume2 += unseen_volume;
}
