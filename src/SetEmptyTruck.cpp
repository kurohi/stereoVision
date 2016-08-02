#include <TwinCamera.hpp>
#include <StereoDepth.hpp>
#include <WriteToMesh.hpp>

int main(int argc, char **argv){
	if(argc<2){
		std::cout<<"Need to specify the calibration file"<<std::endl;
		return 1;
	}
	cv::Mat img1,img2, disparity;
	TwinCamera twin(0,1);
	twin.getDoubleImages(img1,img2);
	twin.loadCameraParameters(argv[1], img1, img2);
	cv::Mat Q_matrix = twin.getQMatrix();
	StereoDepth stereoDepth;
	twin.rectifyForStereo(img1, img2);
	stereoDepth.setImage1(img1);
	stereoDepth.setImage2(img2);
	if(stereoDepth.doDepth()){
		disparity = stereoDepth.getDisparity();
		WriteToMesh::writeWithColorToMeshRaw(disparity, img1, Q_matrix, "emptyTruckMesh.ply"); 
		//cv::equalizeHist(disparity,disparity);
		cv::imwrite("img1.jpg",img1);
		cv::imwrite("img2.jpg",img2);
		
		cv::imwrite("emptyTruck.jpg", disparity);
	}else{
		std::cout<<"Error when computing the stereo image."<<std::endl;
	}
}
