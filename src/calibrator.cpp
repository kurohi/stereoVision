/**
* @file calibrator.cpp
* @brief main program for stereo calibration
*/
#include <stdint.h>
#include <iostream>
#include <stdexcept>
#include <stereo_camera_calibration.hpp>

/**
* @brief Unnamed namespace in calibrator.cpp
*/
namespace {
/** @brief path to the configuration YAML file */
const char* kDefaultInputConfigDirectoryPath = "config.yaml";
}  // end of namespace

/**
* @brief main for stereo calibration tool
* @param[in] argc : unused
* @param[in] argv : unused
* @return unused
*/
int main(int argc, const char* argv[]) {
    if (argc < 2){
        std::cout << "usage:calibrator_triple.exe path_to_config.yaml" << std::endl;
        std::cout << "if -v is added at the end a verbose mode is activated showing the results of each found chessboard" << std::endl;
        std::cout << "example" << std::endl;
        std::cout << "calibrator_triple.exe ../config/config.yaml <-v> <-c> <-t>" << std::endl;
        return 0;
    }
    bool with_view = false;
    bool with_circle_grid = false;
    bool with_triple_set = false;
    for(int i=2; i<argc; i++){
        if(std::string(argv[i]) == "-v"){
            with_view = true;
        }else if(std::string(argv[i]) == "-c"){
            std::cout<<"Calibrating with circle grid"<<std::endl;
            with_circle_grid = true;
        }else if(std::string(argv[i]) == "-t"){
            std::cout<<"Triple set mode activated"<<std::endl;
            with_triple_set = true;
        }
    }
    try {
        std::cout << "start stereo calibration" << std::endl;
        //Getting address in case the program is being exevuted from a different folder
        char configFolderPath[100];
        strcpy(configFolderPath, kDefaultInputConfigDirectoryPath);
        if(argc>1){
            strcpy(configFolderPath, argv[1]);
        }
        // generate instance of StereoCameraCalibration class
        stereo_tools::StereoCameraCalibration
        calibrator(configFolderPath, with_triple_set );
        // execute calibration
        if(with_circle_grid){
            calibrator.calibrate_circlegrid(with_view);
        }else{
            calibrator.calibrate(with_view);
        }
        // confirm calibrated result.
        std::cout << "Result confirmation" << std::endl;
        calibrator.rectifyImages();
        // output calibrated result as YAML file.
        calibrator.writeCalibrationFiles();

        std::cout << "stereo calibration terminated normally." << std::endl;

    } catch (const std::runtime_error& e) {
        std::cout << e.what() << std::endl;
        std::cout << "error: failed stereo calibration" << std::endl;
    }

    return 0;
}
