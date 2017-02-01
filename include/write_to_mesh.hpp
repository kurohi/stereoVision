#ifndef WRTIE_TO_MESH_HPP_
#define WRTIE_TO_MESH_HPP_
#include "general.hpp"
#include <fstream>
#include <opencv2/calib3d/calib3d.hpp>

class WriteToMesh{
    public:
        static void writeDisparityToMeshRaw(cv::Mat disparity, cv::Mat Qmatrix, std::string filename);
        static void writeDisparityToMeshProcessed(cv::Mat point_cloud, cv::Mat disparity, std::string filename);
        static void writeWithColorToMeshRaw(cv::Mat disparity, cv::Mat rgb_img, cv::Mat Qmatrix, std::string filename);
        static void writeWithColorToMeshProcessed(cv::Mat point_cloud, cv::Mat disparity, cv::Mat rgb_img, std::string filename);
    private:
        WriteToMesh() { }
};


#endif
