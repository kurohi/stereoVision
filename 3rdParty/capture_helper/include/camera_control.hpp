#ifndef CAMERA_CONTROL_HPP
#define CAMERA_CONTROL_HPP

#include <vector>

/*
 * @brief This class serves as interface between the camera control parameters and the other modules
 */
class CameraControl{
    public:
        /*
         * @brief Constructor of the class
         * @param[in] Camera address. On linux it usually is /dev/video0
         */
        CameraControl(std::string camera_addrs);
        /*
         * @brief Alternative contructor
         * @param[in] int Id of the camera used to find the /dev/videoX file
         */
        CameraControl(int camera_id=0);
        /*
         * @brief Destructor of the class. Releases the camera file
         */
        ~CameraControl();

        /*
         * @brief Send the values saved into the object to the camera
         */
        void applySavedCommand();
        /*
         * @brief Load current camera parameters into the object
         */
        void loadFromCamera();
        /*
         * @brief Camera's software gain control. Set a negative value to make it automatic
         * @param[in] int The value must be between 0x00 and 0xFF
         */
        void setGain(int newvalue);
        /*
         * @brief Get the object's current gain value
         */
        int getGain();
        /*
         * @brief Set the red, green and blue gains
         * @param[in] int red channel gain (from 0x00 to 0xFF)
         * @param[in] int green channel gain (from 0x00 to 0xFF)
         * @param[in] int blue channel gain (from 0x00 to 0xFF)
         */
        void setWhiteBalance(int red, int green, int blue);
        /*
         * @brief Gets the current white balance values
         * @returns Vector with the gains in this order RGB
         */
        std::vector<int> getWhiteBalance();
        /*
         * @brief Sets the exposure time for the camera
         * @param[in] int new exposure time (from 0x0000 to 0xFFFF);
         */
        void setExposureTime(int exp_time);
        /*
         * @brief Get the current exposure time saved on the object
         * @returns Exposure time saved
         */
        int getExposureTime();
        /*
         * @brief Resets the objects values and reactivate the agc, awc, and aec
         */
        void reactivateAuto();
        
    protected:
        int camera_id;
        const unsigned char COM_TYPE_COM_STT, COM_TYPE_COM_END, COM_TYPE_REG_WR, COM_TYPE_REG_RD;
        std::string camera_addrs;
        int camera_file;
        bool agc, aec, awc;
        int gain;
        int exposure_time;
        int white_balance_red, white_balance_green, white_balance_blue;
        
        /**
         * @brief Write the parameters to the camera registers
         * @param[in] hcam camera handler
         * @param[in] addr register address
         * @param[in] data value to be inserted to the address
         * @return int error code or 0 if succeed
         */
        int regWrite(int hcam, unsigned char addr, unsigned char data);
        /**
         * @brief Read the parameters on the camera registers
         * @param[in] hcam camera handler
         * @param[in] addr register address
         * @param[out] data where the value of the register is writen
         * @return in error code or 0 if succeed
         */
        int regRead(int hcam, unsigned char addr, unsigned char *data);
        /**
         * @brief Sub routine IRIS get control from Shikino High Tech
         * @param[in] hcam camera handler
         * @return int -1 in case of error
         */
        int myIrisGetControl(int hcam);
        /**
         * @brief Sub routine IRIS set control from Shikino High Tech
         * @param[in] hcam camera handler
         * @param[in] value value to write on the camera
         * @return int -1 in case de error
         */
        int myIrisSetControl(int hcam, unsigned char value);
};

#endif
