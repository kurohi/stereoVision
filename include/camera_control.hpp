#ifndef CAMERA_CONTROL_HPP
#define CAMERA_CONTROL_HPP


#include <vector>

class CameraControl{
    public:
        CameraControl(std::string camera_addrs);
        CameraControl(int camera_id=0);
        ~CameraControl();

        void applySavedCommand();
        void loadFromCamera();
        void setGain(int newvalue);
        int getGain();
        void setWhiteBalance(int red, int green, int blue);
        std::vector<int> getWhiteBalance();
        void setExposureTime(int exp_time);
        int getExposureTime();
        void reactivateAuto();
        
    private:
        const unsigned char COM_TYPE_COM_STT, COM_TYPE_COM_END, COM_TYPE_REG_WR, COM_TYPE_REG_RD;
        std::string camera_addrs;
        int camera_file;
        bool agc, aec, awc;
        int gain;
        int exposure_time;
        int white_balance_red, white_balance_green, white_balance_blue;
        
        int regWrite(int hcam, unsigned char addr, unsigned char data);
        int regRead(int hcam, unsigned char addr, unsigned char *data);
        int myIrisGetControl(int hcam);
        int myIrisSetControl(int hcam, unsigned char value);
};

#endif
