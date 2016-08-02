#include <TwinCamera.hpp>
#include <StereoDepth.hpp>
#include <Screen.hpp>
#include <WriteToMesh.hpp>

int main(int argc, char **argv){
	if(argc<3){
		std::cout<<"Need to specify the calibration file and emptyTruck image"<<std::endl;
		return 1;
	}
	Screen screen("Disparity");
	cv::Mat img1,img2, disparity, emptyTruck;
	cv::Mat Q_matrix;
	emptyTruck = cv::imread(argv[2], cv::IMREAD_GRAYSCALE);
	TwinCamera twin(0,1);
	twin.getDoubleImages(img1,img2);
	twin.loadCameraParameters(argv[1], img1, img2);
	Q_matrix = twin.getQMatrix();
	StereoDepth stereoDepth;
	twin.rectifyForStereo(img1, img2);
	stereoDepth.setImage1(img1);
	stereoDepth.setImage2(img2);
	if(stereoDepth.doDepth()){
		disparity = stereoDepth.getDisparity();
		WriteToMesh::writeWithColorToMeshRaw(disparity, img1, Q_matrix, "getPercentageMesh.ply");
		screen.putImage(disparity);
		int i,j;
		double volume = 0;
		double totalVolume = 0;
		for(i=0; i<disparity.rows; i++)
			for(j=0; j<disparity.cols; j++){
				volume += 255 - disparity.at<uchar>(i,j);
				totalVolume += 255 - emptyTruck.at<uchar>(i,j);
			}
		double percentage = volume/(double)totalVolume;
		std::cout<<"Total volume: "<<totalVolume<<std::endl;
		std::cout<<"New volume: "<<volume<<std::endl;
		percentage *= 100.0;
		std::cout<<"Volume percentage is: "<<percentage<<"%"<<std::endl;
	}else{
		std::cout<<"Error when computing the stereo image."<<std::endl;
	}
	cv::waitKey(0);
}
