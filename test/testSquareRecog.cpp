#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

int main(int argc, char **argv){
    if(argc<2){
        std::cout<<"Need to specigy an image"<<std::endl;
        std::cout<<"./testSquareRecog <image>"<<std::endl;
        return 1;
    }

    cv::Mat src = cv::imread(argv[1]);
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::Mat contour_img;
    cv::cvtColor(src, contour_img, cv::COLOR_BGR2GRAY);
    cv::findContours(contour_img, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_TC89_L1);

    
    src.copyTo(contour_img);
    cv::drawContours(contour_img, contours, -1, cv::Scalar(255,0,0), 2, 8, hierarchy);

    char c=0;
    cv::Mat edge_img;
    double thresh1, thresh2;
    int appsize = 3;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(1, 1));
    int interactions = 1;
    while(c!=27){
        cv::cvtColor(src, edge_img, cv::COLOR_BGR2GRAY);
        cv::Canny(edge_img, edge_img, thresh1, thresh2, appsize);
        cv::dilate(edge_img, edge_img, kernel, cv::Point(-1,-1), interactions);
        cv::imshow("Edge test", edge_img);
        c = cv::waitKey(0);
        switch(c){
            case 'a': thresh1++;break;
            case 'z': thresh1--;break;
            case 's': thresh2++;break;
            case 'x': thresh2--;break;
            case 'd': appsize+=2;break;
            case 'c': appsize-=2;break;
            case 'f': interactions+=2;break;
            case 'v': interactions-=2;break;
            case 'p':   std::cout<<"Thresh1: "<<thresh1<<std::endl;
                        std::cout<<"Thresh2: "<<thresh2<<std::endl;
                        std::cout<<"Apperture size: "<<appsize<<std::endl;
        }
    }

    cv::imshow("Orig image", src);
    cv::imshow("Contours", contour_img);
    cv::waitKey(0);
    return 0;
}
