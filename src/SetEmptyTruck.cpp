#include <twin_camera.hpp>
#include <stereo_depth.hpp>
#include <write_to_mesh.hpp>

int main(int argc, char **argv){
	if(argc<2){
		std::cout<<"Need to specify the calibration file"<<std::endl;
		return 1;
	}
	bool using_files=false;
	if(argc>=5){
		if(std::string(argv[2]) == "-f"){
			using_files = true;
		}
	}
	cv::Mat img1,img2, disparity;
	TwinCamera twin(0,1);
	if(!using_files){
		twin.getDoubleImages(img1,img2);
	}else{
		img1 = cv::imread(argv[3]);
		img2 = cv::imread(argv[4]);
	}
	twin.loadCameraParameters(argv[1], img1, img2);
	cv::Mat Q_matrix = twin.getQMatrix();
	StereoDepth stereoDepth;
	twin.rectifyForStereo(img1, img2);
	stereoDepth.setImage1(img1);
	stereoDepth.setImage2(img2);
	if(stereoDepth.doDepth()){
		disparity = stereoDepth.getDisparity();
		cv::imshow("emptytruck", disparity);
		cv::waitKey(0);
		WriteToMesh::writeWithColorToMeshRaw(disparity, img1, Q_matrix, "emptyTruckMesh.ply"); 
		//cv::equalizeHist(disparity,disparity);
		cv::imwrite("left.png",img1);
		cv::imwrite("right.png",img2);
		
		cv::imwrite("emptyTruck.png", disparity);
	}else{
		std::cout<<"Error when computing the stereo image."<<std::endl;
	}
}
