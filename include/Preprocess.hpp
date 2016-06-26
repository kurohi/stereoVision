#ifndef PREPROCESS_HPP
#define PREPROCESS_HPP

#include <General.hpp>

using namespace cv;

class Preprocess{
public:
	static void smooth(Mat src, Mat &dst);

};

#endif
