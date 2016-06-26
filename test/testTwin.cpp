#include <TwinCamera.hpp>

int main(int argc, char **argv){
	TwinCamera twin(0,1);
	Mat img1, img2;
	twin.getDoubleImages( img1, img2);
	twin.loadCameraParameters(argv[1], img1, img2);
	twin.rectifyForStereo(img1,img2);
	imwrite("image1.jpg", img1);
	imwrite("image2.jpg", img2);
	return 0;
}
