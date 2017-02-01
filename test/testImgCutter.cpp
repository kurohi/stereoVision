#include "img_cutter.hpp"


int main(int argc, char **argv){
    if(argc<2){
        std::cout<<"Need to provide an image to be cut"<<std::endl;
        std::cout<<"./testImgCutter <image>"<<std::endl;
        return 1;
    }
    cv::Mat orig_src = cv::imread(argv[1]);
    cv::Mat work_copy;
    orig_src.copyTo(work_copy);
    std::vector<cv::Mat> img_slices;
    img_slices = ImgCutter::cut_all_corners(orig_src, cv::Rect(orig_src.cols/3, orig_src.rows/3, orig_src.cols/3, orig_src.rows/3));
    cv::rectangle(orig_src, cv::Point(orig_src.cols/3, orig_src.rows/3), cv::Point(2*orig_src.cols/3, 2*orig_src.rows/3), cv::Scalar(255,0,0), 5);
    cv::imshow("Original img", orig_src);
    std::vector<std::string> titles;
    titles.push_back("Top");
    titles.push_back("Bottom");
    titles.push_back("Left");
    titles.push_back("Right");
    titles.push_back("Center");
    for(size_t i=0; i<titles.size(); i++){
        cv::imshow(titles[i], img_slices[i]);
    }
    cv::waitKey(0);
    return 0;
}
