#include <pcl_view.hpp>
#include <pcl_get_planes.hpp>

#include <screen.hpp>

int main(int argc, char **argv){
	if(argc != 7){
		std::cout << "Need to provide the disparity images, Q matrix and the 4 plane coeficients" << std::endl;
		return 0;
	}
	pclView viewtest;
	cv::Mat disparity_img, Q;
	disparity_img = cv::imread(argv[1],CV_LOAD_IMAGE_GRAYSCALE);
	pclGetPlanes computePlanes(1);
	cv::FileStorage fs(argv[2], cv::FileStorage::READ);
	float coeficients1,coeficients2,coeficients3,coeficients4;
	sscanf(argv[3], "%f", &coeficients1);
	sscanf(argv[4], "%f", &coeficients2);
	sscanf(argv[5], "%f", &coeficients3);
	sscanf(argv[6], "%f", &coeficients4);
	fs["Q"] >> Q;
	//If size of Q is not 4x4 exit
	if (Q.cols != 4 || Q.rows != 4)
	{
		std::cerr << "ERROR: Could not read matrix Q (doesn't exist or size is not 4x4)" << std::endl;
		return 1;
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr points;
	pcl::PointCloud<pcl::PointXYZ>::Ptr points_full;
	cv::Mat cut_image;
	points_full = viewtest.convertToPointCloudNoColor(disparity_img, Q);
	
	computePlanes.addLoadedCoeficients(points_full, coeficients1, coeficients2, coeficients3, coeficients4,0);
	
	pcl::PointIndices::Ptr inliners (new pcl::PointIndices);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_points_color(new pcl::PointCloud<pcl::PointXYZRGB>);
	
	inliners = computePlanes.getInlinersForIndex(0);
	pcl::PointXYZRGB point;
	for(int i=0; i<points_full->points.size(); i++){
		point.x = points_full->points[i].x;
		point.y = points_full->points[i].y;
		point.z = points_full->points[i].z;
		uint32_t rgb = 0x555555;
		point.rgb = *reinterpret_cast<float*>(&rgb);
		plane_points_color->points.push_back(point);
	}
	for(int i=0; i<inliners->indices.size(); i++){
		int index = inliners->indices[i];
		point.x = points_full->points[index].x;
		point.y = points_full->points[index].y;
		point.z = points_full->points[index].z;
		uint32_t rgb = 0xFF0000;
		point.rgb = *reinterpret_cast<float*>(&rgb);
		plane_points_color->points.push_back(point);
	}
	viewtest.createVisualizer(plane_points_color);
	viewtest.loop_vizualizer();
	return 1;
}
