#include <general.hpp>

class VolumeSubstraction{
    public:
        static double getVolumeDifference_ml(const cv::Mat &volume1, const cv::Mat &volume2);
        static double getVolumeRatio(const cv::Mat &volume1, const cv::Mat &volume2);
        static double getVolumePercentage(const cv::Mat &volume1, const cv::Mat &volume2);

    private:
        VolumeSubstraction() {}
        ~VolumeSubstraction(){}

        static double max_volume;
        static void getVolumes(const cv::Mat &volume1, const cv::Mat &volume2, double &total_volume1, double &total_volume2);
};
