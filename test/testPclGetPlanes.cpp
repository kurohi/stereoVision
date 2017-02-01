#include <pcl_view.hpp>
#include <pcl_get_planes.hpp>

#include <screen.hpp>

int main(int argc, char **argv){
	if(argc != 3){
		std::cout << "Need to provide the disparity images and Q matrix" << std::endl;
		return 0;
	}
	pclView viewtest;
	cv::Mat disparity_img, Q;
	disparity_img = cv::imread(argv[1],CV_LOAD_IMAGE_GRAYSCALE);
	pclGetPlanes computePlanes(5);
	cv::FileStorage fs(argv[2], cv::FileStorage::READ);
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
	for(int i=4; i>=0; i--){
		computePlanes.cutImageForPlanes(disparity_img, cut_image, i);
		cv::imshow("cut image", cut_image);
		cv::waitKey(0);
		points = viewtest.convertToPointCloudNoColor(cut_image, Q);
		try{
			if(!computePlanes.computePaneByIndex(points, i)){
				std::cout<<"Num deu...."<<std::endl;
				return 0;
			}
			std::vector<double> coefs;
			coefs = computePlanes.getCoeficientsForIndex(i);
			for(int j=0; j<coefs.size(); j++){
				std::cout<<coefs[j]<<" ";
			}std::cout<<std::endl;
		}catch(cv::Exception &e){
			std::cout << "Deu ruim tio" << std::endl;
		}
	}
	pcl::PointIndices::Ptr inliners (new pcl::PointIndices);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_points_color(new pcl::PointCloud<pcl::PointXYZRGB>);
	
	pcl::PointXYZRGB point;
	for(int i=0; i<points_full->points.size(); i++){
		point.x = points_full->points[i].x;
		point.y = points_full->points[i].y;
		point.z = points_full->points[i].z;
		uint32_t rgb = 0x555555;
		if((computePlanes.isOnPlane(points_full->points[i],3))||
		   (computePlanes.isOnPlane(points_full->points[i],3))||
		   (computePlanes.isOnPlane(points_full->points[i],2))||
		   (computePlanes.isOnPlane(points_full->points[i],1))||
		   (computePlanes.isOnPlane(points_full->points[i],0)))
		{
			rgb = 0xFF0000;
		}
		point.rgb = *reinterpret_cast<float*>(&rgb);
		plane_points_color->points.push_back(point);
	}
	/*
	for(int j=0; j<5; j++){
		inliners = computePlanes.getInlinersForIndex(j);
		for(int i=0; i<inliners->indices.size(); i++){
			int index = inliners->indices[i];
			point.x = points->points[index].x;
			point.y = points->points[index].y;
			point.z = points->points[index].z;
			uint32_t rgb = 0xFF0000;
			point.rgb = *reinterpret_cast<float*>(&rgb);
			plane_points_color->points.push_back(point);
		}
	}
	*/
	viewtest.createVisualizer(plane_points_color);
	viewtest.loop_vizualizer();
	return 1;
}
