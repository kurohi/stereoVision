#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

int rect_x, rect_y;

void mouseCallback(int event, int x, int y, int flags, void* userdata){
    if(event == cv::EVENT_LBUTTONDOWN){
        rect_x = x;
        rect_y = y;
    }
}

int main(int argc, char **argv){
    if(argc < 2){
        std::cout<<"Need to specify a left image for selecting the square"<<std::endl;
        std::cout<<"./squareSelection <rect left image>"<<std::endl;
        return 1;
    }
    cv::Mat target_img = cv::imread(argv[1]);
    cv::namedWindow("Square selection");
    cv::setMouseCallback("Square selection", mouseCallback);

    cv::Mat output_img;
    int width = 10;
    int height = 10;
    char c=0;
    while(c!='q'){
        target_img.copyTo(output_img);
        cv::rectangle(output_img, cv::Point(rect_x, rect_y), cv::Point(rect_x+width, rect_y+height), cv::Scalar(255,0,0), 3, 8, 0);
        cv::imshow("Square selection", output_img);
        c = cv::waitKey(10);
        switch(c){  
            case 'a': width++; break;
            case 'z': width--; break;
            case 's': height++; break;
            case 'x': height--; break;
        }
    }
    std::cout<<"Rectangle points: "<<rect_x<<","<<rect_y<<" w:"<<width<<" h:"<<height<<std::endl;


    return 0;
}
