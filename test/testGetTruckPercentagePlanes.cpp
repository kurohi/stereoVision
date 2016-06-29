#include<TwinCamera.hpp>
#include<StereoDepth.hpp>
#include<pclView.hpp>
#include<pclGetPlanes.hpp>

int main(int argc, char **argv){
	if(argc<4){
		std::cout<<"Need to specify the calibration, plane, and disparit image files"<<std::endl;
		return 1;
	}
	//reating the disparity map for the empty truck
	cv::Mat disparity = cv::imread(argv[3], cv::IMREAD_GRAYSCALE);
	TwinCamera twin(0,1);

	//creating the computePlanes object with 5 planes to calculate
	pclGetPlanes computePlanes(5);
	pclView viewtest;
	pcl::PointCloud<pcl::PointXYZ>::Ptr points(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr points_full;
	cv::Mat Q;
	cv::FileStorage fs(argv[1], cv::FileStorage::READ);
	fs["Q"] >> Q;
	fs.release();

	cv::FileStorage fs2(argv[2], cv::FileStorage::READ);
	std::vector<double> left_plane;
	std::vector<double> right_plane;
	std::vector<double> top_plane;
	std::vector<double> bottom_plane;
	std::vector<double> back_plane;

	for(int i=0; i<4;i++){
		double val;
		char c = (i+'a');
		fs2[std::string("left_plane_").append(1u,c)] >> val;
		left_plane.push_back(val);
		fs2[std::string("right_plane_").append(1u,c)] >> val;
		right_plane.push_back(val);
		fs2[std::string("top_plane_").append(1u,c)] >> val;
		top_plane.push_back(val);
		fs2[std::string("bottom_plane_").append(1u,c)] >> val;
		bottom_plane.push_back(val);
		fs2[std::string("back_plane_").append(1u,c)] >> val;
		back_plane.push_back(val);
	}
	
	try{
		computePlanes.setCoeficientsForIndex(left_plane, pclGetPlanes::LEFT_CUT);
		computePlanes.setCoeficientsForIndex(right_plane, pclGetPlanes::RIGHT_CUT);
		computePlanes.setCoeficientsForIndex(top_plane, pclGetPlanes::TOP_CUT);
		computePlanes.setCoeficientsForIndex(bottom_plane, pclGetPlanes::BOTTOM_CUT);
		computePlanes.setCoeficientsForIndex(back_plane, pclGetPlanes::BACK_CUT);
	}catch(std::runtime_error &exp){
		std::cerr<<exp.what()<<std::endl;
		return 0;
	}

	points_full = viewtest.convertToPointCloudNoColor(disparity, Q);
	for(int i=0; i<points_full->points.size(); i++){
		if(	(computePlanes.isOnPlane(points_full->points[i], pclGetPlanes::LEFT_CUT)) ||
			(computePlanes.isOnPlane(points_full->points[i], pclGetPlanes::RIGHT_CUT)) ||
			(computePlanes.isOnPlane(points_full->points[i], pclGetPlanes::TOP_CUT)) ||
			(computePlanes.isOnPlane(points_full->points[i], pclGetPlanes::BOTTOM_CUT))) {

			continue;
		}
		points->points.push_back(points_full->points[i]);
	}
	std::cout<<"Valid points: "<<points->points.size()<<std::endl;
	double total_occupied_volume = 0.0;
	pcl::PointXYZ origin;
	origin.x=origin.y=origin.z=0;
	double back_plane_distance = computePlanes.distanceFromPlane(origin, pclGetPlanes::BACK_CUT);
	for(int i=0; i<points->points.size(); i++){
		total_occupied_volume += computePlanes.distanceFromPlane(points->points[i], pclGetPlanes::BACK_CUT)/back_plane_distance;
	}
	std::cout<<"Total occupied volume(voxels): "<<total_occupied_volume <<std::endl;
	std::cout<<"Total occupied volume(cm^3): "<<(430*total_occupied_volume) <<std::endl;
	std::cout<<"Percentage: "<<(total_occupied_volume/(640*480) *100)<<"%"<<std::endl;
}
