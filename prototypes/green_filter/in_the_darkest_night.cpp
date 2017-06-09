#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

bool filter(cv::Mat src, cv::Mat &dst, std::vector<uchar> values, std::vector<int> ranges){
    if(values.size() != ranges.size()){
        return false;
    }
    std::vector<cv::Mat> channel(values.size()+1);
    cv::split(src, channel);
    int lower_limit, higher_limit;
    size_t values_size = values.size();
    dst = cv::Mat::zeros(src.size(), src.type());
    for(size_t i=0; i<values_size; i++){
        lower_limit = values[i] - ranges[i];
        higher_limit = values[i] + ranges[i];
        cv::threshold(channel[i], channel[i], lower_limit, 255, cv::THRESH_TOZERO);
        cv::threshold(channel[i], channel[i], higher_limit, 255, cv::THRESH_TOZERO_INV);
    }
    cv::merge(channel, dst);
    cv::cvtColor(dst, dst, cv::COLOR_BGR2GRAY);
    cv::threshold(dst, dst, 128, 255, cv::THRESH_BINARY);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3), cv::Point(1,1));
    cv::dilate(dst, dst, kernel, cv::Point(-1,-1),1);
}

int main(int argc, char **argv){
    if(argc < 2){
        std::cout << "Need to provide the input image" << std::endl;
        std::cout << "./filter <input_img>";
        return 0;
    }
    cv::Mat input_img = cv::imread(argv[1]);
    cv::Mat result_img = input_img.clone();
    std::cout << "Controls:" << std::endl;
    std::cout << "a/z: increase/decrease R thresh value" << std::endl;
    std::cout << "s/x: increase/decrease G thresh value" << std::endl;
    std::cout << "d/c: increase/decrease B thresh value" << std::endl;
    std::cout << "f/v: increase/decrease R thresh range" << std::endl;
    std::cout << "g/b: increase/decrease G thresh range" << std::endl;
    std::cout << "h/n: increase/decrease B thresh range" << std::endl;
    std::cout << "p: print the current values" << std::endl;
    std::vector<uchar> thresh_values(3, 128);
    std::vector<int> thresh_range(3, 50);
    char c = 0;
    while(c!='q'){
        cv::imshow("Original", input_img);
        cv::imshow("Result", result_img);
        c = cv::waitKey(0);
        switch(c){
            case 'a': thresh_values[2]++; break;
            case 'z': thresh_values[2]--; break;
            case 's': thresh_values[1]++; break;
            case 'x': thresh_values[1]--; break;
            case 'd': thresh_values[0]++; break;
            case 'c': thresh_values[0]--; break;
            case 'f': thresh_range[2]++; break;
            case 'v': thresh_range[2]--; break;
            case 'g': thresh_range[1]++; break;
            case 'b': thresh_range[1]--; break;
            case 'h': thresh_range[0]++; break;
            case 'n': thresh_range[0]--; break;
            case 'p':
                std::cout << "Threshold values: ";
                for(int i=0; i<3; i++){
                    std::cout << static_cast<int>(thresh_values[i]) << ", ";
                }
                std::cout << std::endl;
                std::cout << "Threshold range: ";
                for(int i=0; i<3; i++){
                    std::cout << thresh_range[i] << ", ";
                }
                std::cout << std::endl;
                break;
        }
        filter(input_img, result_img, thresh_values, thresh_range);
    }
    return 0;
}
