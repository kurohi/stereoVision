#include <Preprocess.hpp>


//remember, the image has to be in grayscale
void Preprocess::smooth(Mat src, Mat &dst){
	medianBlur(src, dst,3);
	if(src.channels() == 1){
		Mat aux = dst;
		equalizeHist(aux, dst);
	}
}
