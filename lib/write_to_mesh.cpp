#include "write_to_mesh.hpp"

void WriteToMesh::writeDisparityToMeshProcessed(cv::Mat point_cloud, cv::Mat disparity, std::string filename){
    std::ofstream plyfile(filename.c_str());
    if(!plyfile.is_open()){
        throw std::runtime_error("File not found on saveComputedPoints");
    }
    int failed_pixel = 0;
    for(int y=0; y<disparity.rows; y++)
        for(int x=0; x<disparity.cols; x++){
            if(disparity.at<uchar>(y,x)==0){
                failed_pixel++;
            }
        }
    int point_num = (int)point_cloud.total();
    plyfile << "ply" << std::endl;
    plyfile << "format ascii 1.0" << std::endl;
    plyfile << "element vertex " << point_num - failed_pixel << std::endl;
    plyfile << "property float x" << std::endl;
    plyfile << "property float y" << std::endl;
    plyfile << "property float z" << std::endl;
    plyfile << "end_header" << std::endl;

    cv::Point3f point;
    for(int y=0; y<point_cloud.rows; y++)
        for(int x=0; x<point_cloud.cols; x++){
            if(disparity.at<uchar>(y,x)==0){
                continue;
            }
            point = point_cloud.at<cv::Point3f>(y,x);
            plyfile << point.x << " " << point.y << " " << point.z << std::endl;
        }
    plyfile.close();

}

void WriteToMesh::writeDisparityToMeshRaw(cv::Mat disparity, cv::Mat Qmatrix, std::string filename){
    cv::Mat point_cloud;
    cv::reprojectImageTo3D(disparity, point_cloud, Qmatrix);
    writeDisparityToMeshProcessed(point_cloud, disparity, filename); 
}

void WriteToMesh::writeWithColorToMeshRaw(cv::Mat disparity, cv::Mat rgb_img, cv::Mat Qmatrix, std::string filename){
    cv::Mat point_cloud;
    cv::reprojectImageTo3D(disparity, point_cloud, Qmatrix);
    WriteToMesh::writeWithColorToMeshProcessed(point_cloud, disparity, rgb_img, filename);
}

void WriteToMesh::writeWithColorToMeshProcessed(cv::Mat point_cloud, cv::Mat disparity, cv::Mat rgb_img, std::string filename){
    std::ofstream plyfile(filename.c_str());
    if(!plyfile.is_open()){
        throw std::runtime_error("File not found on saveComputedPoints");
    }
    int failed_pixel = 0;
    for(int y=0; y<disparity.rows; y++)
        for(int x=0; x<disparity.cols; x++){
            if(disparity.at<uchar>(y,x)==0){
                failed_pixel++;
            }
        }
    int point_num = (int)point_cloud.total();
    plyfile << "ply" << std::endl;
    plyfile << "format ascii 1.0" << std::endl;
    plyfile << "element vertex " << point_num - failed_pixel << std::endl;
    plyfile << "property float x" << std::endl;
    plyfile << "property float y" << std::endl;
    plyfile << "property float z" << std::endl;
    plyfile << "property uchar red" << std::endl;
    plyfile << "property uchar green" << std::endl;
    plyfile << "property uchar blue" << std::endl;
    plyfile << "end_header" << std::endl;

    cv::Point3f point;
    for(int y=0; y<point_cloud.rows; y++)
        for(int x=0; x<point_cloud.cols; x++){
            if(disparity.at<uchar>(y,x)==0){
                continue;
            }
            point = point_cloud.at<cv::Point3f>(y,x);
            plyfile << point.x << " " << point.y << " " << point.z << " ";
            plyfile << (int)rgb_img.at<cv::Vec3b>(y,x)[2] << " " <<
            (int) rgb_img.at<cv::Vec3b>(y,x)[1] << " " << (int)rgb_img.at<cv::Vec3b>(y,x)[0] << std::endl;

        }
    plyfile.close();


}
