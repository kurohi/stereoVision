#ifndef STEREODEPTH_HPP_
#define STEREODEPTH_HPP_

#include <General.hpp>
#include <Preprocess.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/ximgproc/disparity_filter.hpp>

using namespace cuda;
using namespace cv::ximgproc;

class StereoDepth{
public:
	StereoDepth();
	StereoDepth(Mat src1, Mat src2);
	~StereoDepth();
	
	bool isImageOk();
	bool isDisparityOk();
	
	void setImage1(Mat src);
	void setImage2(Mat src);
	Mat getImage1();
	Mat getImage2();
	Mat getDisparity();
	
	bool doDepth();
	
	//variables adjustment methods
	int getMinDisparity();
	void setMinDisparity(int disp);
	int getNumDisparities();
	void setNumDisparities(int n_disp);
	int getBlockSize();
	void setBlockSize(int block);
	int getP1();
	void setP1(int p1);
	int getP2();
	void setP2(int p2);
	int getDisp12MaxDiff();
	void setDisp12MaxDiff(int maxDiff);
	int getPreFilterCap();
	void setPreFilterCap(int preFilter);
	int getUniquenessRatio();
	void setUniquenessRatio(int uniqRadio);
	int getSpeckleWindowSize();
	void setSpeckleWindowSize(int speckleWin);
	int getSpeckleRange();
	void setSpeckleRange(int speckleRan);
	int getMode();
	void setMode(int mod);
	
private:
	void initializeDefaultStereo();
	Rect computeROI(Size2i src_sz, Ptr<StereoMatcher> matcher_instance);
	
	Mat left, right;
	Mat left_for_matcher, right_for_matcher;
    Mat left_disp,right_disp;
    Mat filtered_disp;
    Rect ROI;
	Mat disparity16,disparity8;
	Ptr<StereoSGBM> stereo_left, stereo_right;
	bool is_img_ok, is_disp_ok;
	double sigma, lambda;
	
	//stereodepth config variables
	int minDisparity;
	int numDisparities;
	int blockSize;
	int P1;
	int P2;
	int disp12MaxDiff;
	int preFilterCap;
	int uniquenessRatio;
	int speckleWindowSize;
	int speckleRange;
	int mode;
};

#endif
