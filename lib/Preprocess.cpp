#include <Preprocess.hpp>


//remember, the image has to be in grayscale
void Preprocess::smooth(cv::Mat src, cv::Mat &dst){
	medianBlur(src, dst,3);
	if(src.channels() == 1){
		cv::Mat aux = dst;
		cv::equalizeHist(aux, dst);
	}
}
