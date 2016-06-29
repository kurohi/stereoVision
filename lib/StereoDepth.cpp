#include<StereoDepth.hpp>

StereoDepth::StereoDepth(){
	is_img_ok = is_disp_ok = false;
	initializeDefaultStereo();
}
StereoDepth::StereoDepth(cv::Mat src1, cv::Mat src2){
	left = src1;
	right = src2;
	is_img_ok = true;
	is_disp_ok = false;
	initializeDefaultStereo();
}
StereoDepth::~StereoDepth(){

}

void StereoDepth::initializeDefaultStereo(){
	minDisparity=-64;
	numDisparities=192;
	blockSize=5;
	P1=600;
	P2=2400;
	disp12MaxDiff=10;
	preFilterCap=4;
	uniquenessRatio=1;
	speckleWindowSize=150;
	speckleRange=2;
	//mode=StereoSGBM::MODE_SGBM;
	mode=false;
	sigma=1.0;
	lambda=8000.0;
	stereo_left = cv::StereoSGBM::create(minDisparity, numDisparities, blockSize, P1, P2, disp12MaxDiff, preFilterCap, uniquenessRatio, speckleWindowSize, speckleRange, mode);
	
	stereo_right = cv::StereoSGBM::create(-numDisparities+1, numDisparities, blockSize, P1, P2, disp12MaxDiff, preFilterCap, uniquenessRatio, speckleWindowSize, speckleRange, mode);
}

bool StereoDepth::isImageOk(){
	if((left.cols!=0)&&(right.cols!=0)){
		is_img_ok = true;
	}
	return is_img_ok;
}

bool StereoDepth::isDisparityOk(){
	if(disparity8.cols!=0){
		is_disp_ok = true;
	}
	return is_disp_ok;
}

void StereoDepth::setImage1(cv::Mat src){
	cv::cvtColor(src, left, CV_BGR2GRAY);
	Preprocess::smooth(left, left); 
	isImageOk();
}

void StereoDepth::setImage2(cv::Mat src){
	cv::cvtColor(src, right, CV_BGR2GRAY);
	Preprocess::smooth(right, right); 
	isImageOk();
}

cv::Mat StereoDepth::getImage1(){
	return left;
}

cv::Mat StereoDepth::getImage2(){
	return right;
}

cv::Mat StereoDepth::getDisparity(){
	return disparity8;
}

bool StereoDepth::doDepth(){
	if(is_img_ok){
		left_for_matcher  = left.clone();
		right_for_matcher = right.clone();
		ROI = computeROI(left_for_matcher.size(),stereo_left);
	
		stereo_left->compute(left_for_matcher, right_for_matcher,left_disp);
		stereo_right->compute(right_for_matcher,left_for_matcher, right_disp);
		
		cv::Ptr<DisparityWLSFilter> wls_filter = createDisparityWLSFilter(stereo_left);
		wls_filter->setLambda(lambda);
		wls_filter->setSigmaColor(sigma);
		wls_filter->filter(left_disp,left,disparity16,right_disp,ROI);
		//stereo_left->compute(left,right,disparity16);
		double minVal; double maxVal;
		cv::minMaxLoc( disparity16, &minVal, &maxVal );
		disparity16.convertTo( disparity8, CV_8UC1, 255/(maxVal - minVal));
		is_disp_ok = true;
		return true;
	}else{
		return false;
	}
}


//variable adjustment implementation

int StereoDepth::getMinDisparity(){
	return minDisparity;
}
void StereoDepth::setMinDisparity(int disp){
	//if(disp>=0){
		minDisparity = disp;
		stereo_left->setMinDisparity(minDisparity);
	//}
}
int StereoDepth::getNumDisparities(){
	return numDisparities;
}
void StereoDepth::setNumDisparities(int n_disp){
	if((n_disp>0)&&((n_disp%16)==0)){
		numDisparities = n_disp;
		stereo_left->setNumDisparities(numDisparities);
		stereo_right->setNumDisparities(numDisparities);
		stereo_right->setMinDisparity(-numDisparities+1);
	}
}
int StereoDepth::getBlockSize(){
	return blockSize;
}
void StereoDepth::setBlockSize(int block){
	if((block>=1)&&(block%2==1)){
		blockSize = block;
		stereo_left->setBlockSize(blockSize);
		stereo_right->setBlockSize(blockSize);
	}
}
int StereoDepth::getP1(){
	return P1;
}
void StereoDepth::setP1(int p1){
	if(p1<P2){
		P1 = p1;
		stereo_left->setP1(P1);
		stereo_right->setP1(P1);
	}
}
int StereoDepth::getP2(){
	return P2;
}
void StereoDepth::setP2(int p2){
	if(p2>P1){
		P2 = p2;
		stereo_left->setP2(P2);
		stereo_right->setP2(P2);
	}
}
int StereoDepth::getDisp12MaxDiff(){
	return disp12MaxDiff;
}
void StereoDepth::setDisp12MaxDiff(int maxDiff){
	//a negative value diactivate the check
	disp12MaxDiff = maxDiff;
	stereo_left->setDisp12MaxDiff(disp12MaxDiff);
	stereo_right->setDisp12MaxDiff(disp12MaxDiff);
}
int StereoDepth::getPreFilterCap(){
	return preFilterCap;
}
void StereoDepth::setPreFilterCap(int preFilter){
	preFilterCap = preFilter;
	stereo_left->setPreFilterCap(preFilterCap);
	stereo_right->setPreFilterCap(preFilterCap);
}
int StereoDepth::getUniquenessRatio(){
	return uniquenessRatio;
}
void StereoDepth::setUniquenessRatio(int uniqRadio){
	if(uniqRadio > 0){
		uniquenessRatio = uniqRadio;
		stereo_left->setUniquenessRatio(uniquenessRatio);
		stereo_right->setUniquenessRatio(uniquenessRatio);
	}
}
int StereoDepth::getSpeckleWindowSize(){
	return speckleWindowSize;
}
void StereoDepth::setSpeckleWindowSize(int speckleWin){
	//a zero value disable the check
	if(speckleWin>0){
		speckleWindowSize = speckleWin;
		stereo_left->setSpeckleWindowSize(speckleWindowSize);
		stereo_right->setSpeckleWindowSize(speckleWindowSize);
	}
}
int StereoDepth::getSpeckleRange(){
	return speckleRange;
}
void StereoDepth::setSpeckleRange(int speckleRan){
	if(speckleRan > 0){
		speckleRange = speckleRan;
		stereo_left->setSpeckleRange(speckleRange);
		stereo_right->setSpeckleRange(speckleRange);
	}
}
int StereoDepth::getMode(){
	return mode;
}
void StereoDepth::setMode(int mod){
	if(mod == cv::StereoSGBM::MODE_HH){
		mode = cv::StereoSGBM::MODE_HH;
	}else{
		mode = false;
	}
	stereo_left->setMode(mode);
	stereo_right->setMode(mode);
}

cv::Rect StereoDepth::computeROI(cv::Size2i src_sz, cv::Ptr<cv::StereoMatcher> matcher_instance)
{
	int min_disparity = matcher_instance->getMinDisparity();
	int num_disparities = matcher_instance->getNumDisparities();
	int block_size = matcher_instance->getBlockSize();

	int bs2 = block_size/2;
	int minD = min_disparity, maxD = min_disparity + num_disparities - 1;

	int xmin = maxD + bs2;
	int xmax = src_sz.width + minD - bs2;
	int ymin = bs2;
	int ymax = src_sz.height - bs2;

	cv::Rect r(xmin, ymin, xmax - xmin, ymax - ymin);
	return r;
}
