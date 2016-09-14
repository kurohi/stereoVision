#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <string>
#include <iomanip>
#include <TwinCamera.hpp>

using namespace cv;
using namespace std;

int main(int argc, char* argv[])
{
    if(argc<4){
        std::cout<<"To run this program you need to specify the number of pictures, with and height of the chessboard and the size of each square"<<std::endl;
	std::cout<<"Example: ./stereocalibrate 20 10 7 12.5 <-f> <folder with pictures>"<<std::endl;
	std::cout<<"if the \"-f\" is defined the program will use a folder with the pictures rather than the cameras"<<std::endl;
    }
    int numBoards = atoi(argv[1]);
    int board_w = atoi(argv[2]);
    int board_h = atoi(argv[3]);
    double square_size = atof(argv[4]);

    cv::Size board_sz = Size(board_w, board_h);
    int board_n = board_w*board_h;

    vector<vector<cv::Point3f> > object_points;
    vector<vector<cv::Point2f> > imagePoints1, imagePoints2;
    vector<cv::Point2f> corners1, corners2;

    vector<cv::Point3f> obj;
    for (int j=0; j<board_h; j++)
    	for(int k=0; k<board_w; k++){
            obj.push_back(cv::Point3f(j*square_size, k*square_size, 0.0f));
    }

    cv::Mat left_img, right_img, left_gray, right_gray;
    std::string folder_root;
    TwinCamera *twin;
    if((argc >= 7)&&(std::string(argv[5]) == "-f")){
	std::cout<<"Using images for calibration"<<std::endl;
	folder_root = std::string(argv[6]);
    }else{
        twin = new TwinCamera(0,1);
    }

    int success = 0, k = 0, img_index =0;
    bool found1 = false, found2 = false;

    while (success < numBoards)
    {
    	if((argc >= 7)&&(std::string(argv[5]) == "-f")){
	    std::ostringstream left_addrs, right_addrs;
	    left_addrs << folder_root << "left_" << std::setw(2) << std::setfill('0') << img_index << ".jpg";
	    std::cout<<left_addrs.str()<<std::endl;
            left_img = cv::imread(left_addrs.str());
	    right_addrs << folder_root << "right_" << std::setw(2) << std::setfill('0') << img_index << ".jpg";
	    std::cout<<right_addrs.str()<<std::endl;
	    right_img = cv::imread(right_addrs.str());
	    if((right_img.empty())||(left_img.empty())){
                break;
	    }
	    }else{
            twin->getDoubleImages(left_img,right_img);
	    }
        cv::cvtColor(left_img, left_gray, CV_BGR2GRAY);
        cv::cvtColor(right_img, right_gray, CV_BGR2GRAY);

        found1 = cv::findChessboardCorners(left_img, board_sz, corners1, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
        found2 = cv::findChessboardCorners(right_img, board_sz, corners2, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

        if (found1)
        {
            cv::cornerSubPix(left_gray, corners1, cv::Size(7, 7), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 100, 1e-5));
            cv::drawChessboardCorners(left_gray, board_sz, corners1, found1);
        }

        if (found2)
        {
            cv::cornerSubPix(right_gray, corners2, cv::Size(7, 7), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 100, 1e-5));
            cv::drawChessboardCorners(right_gray, board_sz, corners2, found2);
        }
        
        cv::imshow("left", left_gray);
        cv::imshow("right", right_gray);

        k = cv::waitKey(0);
        if ((argc >= 7)&&(std::string(argv[5]) == "-f"))
        {
            //k = cv::waitKey(0);
	        k = ' ';
	        img_index++;
        }
        if (k == 27)
        {
            break;
        }
        if (k == ' ' && found1 !=0 && found2 != 0)
        {
            imagePoints1.push_back(corners1);
            imagePoints2.push_back(corners2);
            object_points.push_back(obj);
            printf ("Corners stored\n");
	        std::ostringstream left_addrs, right_addrs;
	        left_addrs << "left_" << std::setw(2) << std::setfill('0') << success << ".jpg";
	        right_addrs << "right_" << std::setw(2) << std::setfill('0') << success << ".jpg";

	        cv::imwrite(left_addrs.str(), left_img);
	        cv::imwrite(right_addrs.str(), right_img);
            success++;
	        std::cout<<success<<std::endl;

            if (success >= numBoards)
            {
                break;
            }
        }
    }
    if((argc < 7)||(std::string(argv[5]) != "-f")){
        delete(twin);
    }else{
        std::ostringstream left_addrs, right_addrs;
        left_addrs << folder_root << "left_00.jpg";
        left_img = cv::imread(left_addrs.str());
        right_addrs << folder_root << "right_00.jpg";
        right_img = cv::imread(right_addrs.str());
    }

    cv::destroyAllWindows();
    printf("Starting Calibration\n");
    cv::Mat CM1 = cv::Mat(3, 3, CV_64FC1);
    cv::Mat CM2 = cv::Mat(3, 3, CV_64FC1);
    cv::Mat D1, D2;
    cv::Mat R, T, E, F;

    float result = cv::stereoCalibrate(object_points, imagePoints1, imagePoints2, 
                    CM1, D1, CM2, D2, left_img.size(), R, T, E, F, 
                    CV_CALIB_SAME_FOCAL_LENGTH | CV_CALIB_ZERO_TANGENT_DIST,
                    cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5));
    
    std::cout<<"Result from calibration: "<<result<<std::endl;

    cv::FileStorage fs1("mystereocalib.yml", cv::FileStorage::WRITE);
    fs1 << "CM1" << CM1;
    fs1 << "CM2" << CM2;
    fs1 << "D1" << D1;
    fs1 << "D2" << D2;
    fs1 << "R" << R;
    fs1 << "T" << T;
    fs1 << "E" << E;
    fs1 << "F" << F;

    printf("Done Calibration\n");

    printf("Starting Rectification\n");

    cv::Mat R1, R2, P1, P2, Q;
    cv::stereoRectify(CM1, D1, CM2, D2, left_img.size(), R, T, R1, R2, P1, P2, Q);
    fs1 << "R1" << R1;
    fs1 << "R2" << R2;
    fs1 << "P1" << P1;
    fs1 << "P2" << P2;
    fs1 << "Q" << Q;

    printf("Done Rectification\n");

    printf("Applying Undistort\n");

    cv::Mat map1x, map1y, map2x, map2y;
    cv::Mat imgU1, imgU2;

    cv::initUndistortRectifyMap(CM1, D1, R1, P1, left_img.size(), CV_32FC1, map1x, map1y);
    cv::initUndistortRectifyMap(CM2, D2, R2, P2, right_img.size(), CV_32FC1, map2x, map2y);

    printf("Undistort complete\n");

    img_index = 0;
    while(1)
    {    
       	if((argc >= 7)&&(std::string(argv[5]) == "-f")){
	    std::ostringstream left_addrs, right_addrs;
	    left_addrs << folder_root << "left_" << std::setw(2) << std::setfill('0') << img_index << ".jpg";
            left_img = cv::imread(left_addrs.str());
	    right_addrs << folder_root << "right_" << std::setw(2) << std::setfill('0') << img_index << ".jpg";
	    right_img = cv::imread(right_addrs.str());
            if((right_img.empty())||(left_img.empty())){
	        break;
	    }

	}else{
            twin->getDoubleImages(left_img,right_img);
	}

        cv::remap(left_img, imgU1, map1x, map1y, INTER_LINEAR, BORDER_CONSTANT, cv::Scalar());
        cv::remap(right_img, imgU2, map2x, map2y, INTER_LINEAR, BORDER_CONSTANT, cv::Scalar());

        cv::imshow("left", imgU1);
        cv::imshow("right", imgU2);

        k = cv::waitKey(5);
	if((argc >= 7)&&(std::string(argv[5]) == "-f")){
	    k = cv::waitKey(0);
	    img_index++;
	}

        if(k==27)
        {
            break;
        }
    }

    

    return(0);
}
