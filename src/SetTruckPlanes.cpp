#include <twin_camera.hpp>
#include <stereo_depth.hpp>
#include <pcl_view.hpp>
#include <pcl_get_planes.hpp>

int main(int argc, char **argv){
	if(argc<2){
		std::cout<<"Need to specify the calibration file"<<std::endl;
		return 1;
	}
	//reating the disparity map for the empty truck
	cv::Mat img1,img2, disparity;
	TwinCamera twin(0,1);
	if(argc!=4){
		twin.getDoubleImages(img1,img2);
	}else{
		img1 = cv::imread(argv[2]);
		img2 = cv::imread(argv[3]);
	}
	twin.loadCameraParameters(argv[1], img1, img2);
	StereoDepth stereoDepth;
	twin.rectifyForStereo(img1, img2);
	stereoDepth.setImage1(img1);
	stereoDepth.setImage2(img2);
	if(stereoDepth.doDepth()){
		disparity = stereoDepth.getDisparity();
		cv::equalizeHist(disparity,disparity);
		imwrite("left.png",img1);
		imwrite("right.png",img2);
		
		imwrite("emptyTruck.png", disparity);
	}else{
		std::cout<<"Error when computing the stereo image."<<std::endl;
		return 1;
	}
	
	//creating the computePlanes object with 5 planes to calculate
	pclGetPlanes computePlanes(5);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr points;
	pcl::PointCloud<pcl::PointXYZ>::Ptr points_full;
	cv::Mat Q;
	cv::Mat cut_image;
	cv::FileStorage fs(argv[1], cv::FileStorage::READ);
	fs["Q"] >> Q;
	//If size of Q is not 4x4 exit
	if (Q.cols != 4 || Q.rows != 4) {
		std::cerr << "ERROR: Could not read matrix Q (doesn't exist or size is not 4x4)" << std::endl;
		return 1;
	}

	pclView viewtest;
	points_full = viewtest.convertToPointCloudNoColor(disparity, Q);
	//cutting the left pclGetPlanes
	computePlanes.cutImageForPlanes(disparity, cut_image, pclGetPlanes::LEFT_CUT);
	points = viewtest.convertToPointCloudNoColor(cut_image, Q);
	try{
		if(!computePlanes.computePaneByIndex(points, 0)){
			std::cout<<"Failed to compute the Left plane"<<std::endl;
			return 0;
		}
	}catch(cv::Exception &e){
		std::cout << "Exception when computing the left plane" << std::endl;
	}
	//cutting the right pclGetPlanes
	computePlanes.cutImageForPlanes(disparity, cut_image, pclGetPlanes::RIGHT_CUT);
	points = viewtest.convertToPointCloudNoColor(cut_image, Q);
	try{
		if(!computePlanes.computePaneByIndex(points, 1)){
			std::cout<<"Failed to compute the Left plane"<<std::endl;
			return 0;
		}
	}catch(cv::Exception &e){
		std::cout << "Exception when computing the left plane" << std::endl;
	}
	//cutting the top pclGetPlanes
	computePlanes.cutImageForPlanes(disparity, cut_image, pclGetPlanes::TOP_CUT);
	points = viewtest.convertToPointCloudNoColor(cut_image, Q);
	try{
		if(!computePlanes.computePaneByIndex(points, 2)){
			std::cout<<"Failed to compute the Left plane"<<std::endl;
			return 0;
		}
	}catch(cv::Exception &e){
		std::cout << "Exception when computing the left plane" << std::endl;
	}
	//cutting the bottom pclGetPlanes
	computePlanes.cutImageForPlanes(disparity, cut_image, pclGetPlanes::BOTTOM_CUT);
	points = viewtest.convertToPointCloudNoColor(cut_image, Q);
	try{
		if(!computePlanes.computePaneByIndex(points, 3)){
			std::cout<<"Failed to compute the Left plane"<<std::endl;
			return 0;
		}
	}catch(cv::Exception &e){
		std::cout << "Exception when computing the left plane" << std::endl;
	}
	//cutting the back pclGetPlanes
	computePlanes.cutImageForPlanes(disparity, cut_image, pclGetPlanes::BACK_CUT);
	points = viewtest.convertToPointCloudNoColor(cut_image, Q);
	try{
		if(!computePlanes.computePaneByIndex(points, 4)){
			std::cout<<"Failed to compute the Left plane"<<std::endl;
			return 0;
		}
	}catch(cv::Exception &e){
		std::cout << "Exception when computing the left plane" << std::endl;
	}
	
	//saving the plane coeficients
	cv::FileStorage fs2("truckPlanes.yml", cv::FileStorage::WRITE);
	std::vector<double> coefs;
	coefs = computePlanes.getCoeficientsForIndex(0);
	fs2<<"left_plane_a"<<coefs[0];
	fs2<<"left_plane_b"<<coefs[1];
	fs2<<"left_plane_c"<<coefs[2];
	fs2<<"left_plane_d"<<coefs[3];
	coefs = computePlanes.getCoeficientsForIndex(1);
	fs2<<"right_plane_a"<<coefs[0];
	fs2<<"right_plane_b"<<coefs[1];
	fs2<<"right_plane_c"<<coefs[2];
	fs2<<"right_plane_d"<<coefs[3];
	coefs = computePlanes.getCoeficientsForIndex(2);
	fs2<<"top_plane_a"<<coefs[0];
	fs2<<"top_plane_b"<<coefs[1];
	fs2<<"top_plane_c"<<coefs[2];
	fs2<<"top_plane_d"<<coefs[3];
	coefs = computePlanes.getCoeficientsForIndex(3);
	fs2<<"bottom_plane_a"<<coefs[0];
	fs2<<"bottom_plane_b"<<coefs[1];
	fs2<<"bottom_plane_c"<<coefs[2];
	fs2<<"bottom_plane_d"<<coefs[3];
	coefs = computePlanes.getCoeficientsForIndex(4);
	fs2<<"back_plane_a"<<coefs[0];
	fs2<<"back_plane_b"<<coefs[1];
	fs2<<"back_plane_c"<<coefs[2];
	fs2<<"back_plane_d"<<coefs[3];
	fs2.release();
	
	return 1;
}
