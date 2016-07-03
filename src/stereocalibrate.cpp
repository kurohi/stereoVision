#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <TwinCamera.hpp>

using namespace cv;
using namespace std;

int main(int argc, char* argv[])
{
    int numBoards = atoi(argv[1]);
    int board_w = atoi(argv[2]);
    int board_h = atoi(argv[3]);

    cv::Size board_sz = Size(board_w, board_h);
    int board_n = board_w*board_h;

    vector<vector<cv::Point3f> > object_points;
    vector<vector<cv::Point2f> > imagePoints1, imagePoints2;
    vector<cv::Point2f> corners1, corners2;

    vector<cv::Point3f> obj;
    for (int j=0; j<board_n; j++)
    {
        obj.push_back(cv::Point3f(j/board_w, j%board_w, 0.0f));
    }

    cv::Mat left_img, right_img, left_gray, right_gray;
    std::string folder_root;
    TwinCamera *twin;
    if((argc >= 6)&&(argv[4] == "-f")){
	folder_root = std::string(argv[5]);
    }else{
        twin = new TwinCamera(0,1);
    }

    int success = 0, k = 0 img_index =0;
    bool found1 = false, found2 = false;

    while (success < numBoards)
    {
    	if((argc >= 6)&&(argv[4] == "-f")){
	    std::ostringstream left_adds, right_addrs;
	    left_addrs << folder_root << "left_" << std::setw(3) << std::setfill('0') << img_index;
            left_img = cv::imread(left_addrs.str());
	    right_addrs << folder_root << "right_" << std::setw(3) << std::setfill('0') << img_index;
	    right_img = cv::imread(right_addrs);

	}else{
            twin->getDoubleImages(left_img,right_img);
	}
        cv::cvtColor(left_img, left_gray, CV_BGR2GRAY);
        cv::cvtColor(right_img, right_gray, CV_BGR2GRAY);

        found1 = cv::findChessboardCorners(left_img, board_sz, corners1, cv::CV_CALIB_CB_ADAPTIVE_THRESH | cv::CV_CALIB_CB_FILTER_QUADS);
        found2 = cv::findChessboardCorners(right_img, board_sz, corners2, cv::CV_CALIB_CB_ADAPTIVE_THRESH | cv::CV_CALIB_CB_FILTER_QUADS);

        if (found1)
        {
            cv::cornerSubPix(left_gray, corners1, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::CV_TERMCRIT_EPS | cv::CV_TERMCRIT_ITER, 30, 0.1));
            cv::drawChessboardCorners(left_gray, board_sz, corners1, found1);
        }

        if (found2)
        {
            cv::cornerSubPix(right_gray, corners2, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::CV_TERMCRIT_EPS | cv::CV_TERMCRIT_ITER, 30, 0.1));
            cv::drawChessboardCorners(right_gray, board_sz, corners2, found2);
        }
        
        cv::imshow("image1", left_gray);
        cv::imshow("image2", right_gray);

        k = cv::waitKey(10);
        if (found1 && found2)
        {
            k = cv::waitKey(0);
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
            success++;
	    std::cout<<success<<std::endl;

            if (success >= numBoards)
            {
                break;
            }
        }
    }
    delete(twin);

    cv::destroyAllWindows();
    printf("Starting Calibration\n");
    cv::Mat CM1 = cv::Mat(3, 3, CV_64FC1);
    cv::Mat CM2 = cv::Mat(3, 3, CV_64FC1);
    cv::Mat D1, D2;
    cv::Mat R, T, E, F;

    float result = cv::stereoCalibrate(object_points, imagePoints1, imagePoints2, 
                    CM1, D1, CM2, D2, left_img.size(), R, T, E, F, 
                    cv::CV_CALIB_SAME_FOCAL_LENGTH | cv::CV_CALIB_ZERO_TANGENT_DIST,
                    cv::TermCriteria(cv::CV_TERMCRIT_ITER+cv::CV_TERMCRIT_EPS, 100, 1e-5));
    
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

    cv::initUndistortRectifyMap(CM1, D1, R1, P1, left_img.size(), cv::CV_32FC1, map1x, map1y);
    cv;:initUndistortRectifyMap(CM2, D2, R2, P2, right_img.size(), cv::CV_32FC1, map2x, map2y);

    printf("Undistort complete\n");

    while(1)
    {    
       	if((argc >= 6)&&(argv[4] == "-f")){
	    std::ostringstream left_adds, right_addrs;
	    left_addrs << folder_root << "left_" << std::setw(3) << std::setfill('0') << img_index;
            left_img = cv::imread(left_addrs.str());
	    right_addrs << folder_root << "right_" << std::setw(3) << std::setfill('0') << img_index;
	    right_img = cv::imread(right_addrs);

	}else{
            twin->getDoubleImages(left_img,right_img);
	}

        cv::remap(left_img, imgU1, map1x, map1y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
        cv::remap(right_img, imgU2, map2x, map2y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());

        cv::imshow("image1", imgU1);
        cv::imshow("image2", imgU2);

        k = cv::waitKey(5);

        if(k==27)
        {
            break;
        }
    }

    

    return(0);
}
