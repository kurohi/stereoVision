#include <pcl_view.hpp>

int main(int argc, char **argv){
	if(argc != 4){
		std::cout << "Need to provide the rgb, disparity images and Q matrix" << std::endl;
		return 0;
	}
	pclView viewtest;
	cv::Mat rgb_img, disparity_img, Q;
	rgb_img = cv::imread(argv[1]);
	disparity_img = cv::imread(argv[2],CV_LOAD_IMAGE_GRAYSCALE);
	cv::FileStorage fs(argv[3], cv::FileStorage::READ);
	fs["Q"] >> Q;
	//If size of Q is not 4x4 exit
	if (Q.cols != 4 || Q.rows != 4)
	{
		std::cerr << "ERROR: Could not read matrix Q (doesn't exist or size is not 4x4)" << std::endl;
		return 1;
	}
	viewtest.visualizeDisparityMap(rgb_img, disparity_img, Q);
}
