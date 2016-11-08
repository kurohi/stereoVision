#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/videodev2.h>
#include <string>

#include "camera_control.hpp"


CameraControl::CameraControl(std::string camera_addrs) :    COM_TYPE_COM_STT(0xFF), /* Command ID [START] */
                                                            COM_TYPE_COM_END(0xFE), /* Command ID [END] */
                                                            COM_TYPE_REG_WR(0xFA), /* Command ID [REG WRITE] */
                                                            COM_TYPE_REG_RD(0xF9)  /* Command ID [REG READ ] */
{
    this->camera_addrs = camera_addrs;
    camera_file = open(camera_addrs.c_str(), O_RDWR | O_NONBLOCK);

    gain = exposure_time = -1;
    white_balance_red = white_balance_green = white_balance_blue = -1;
    agc = aec = awc = true;
}

CameraControl::~CameraControl(){
    close(camera_file);
}

void CameraControl::setGain(int newgain){
    if(newgain < 0 ){
        agc = true;
        gain = -1;
    }else{
        agc = false;
        gain = (newgain>255)?255:newgain;
    }
}

int CameraControl::getGain(){
    return gain;
}

void CameraControl::setWhiteBalance(int red, int green, int blue){
    if((red<0)||(green<0)||(blue<0)){
        awc = true;
        red = green = blue = -1;
    }else{
        awc = false;
        white_balance_red =   (red>255)?255:red;
        white_balance_green = (green>255)?255:green;
        white_balance_blue =  (blue>255)?255:blue;
    }
}

std::vector<int> CameraControl::getWhiteBalance(){
    std::vector<int> white_balance;
    white_balance.push_back(white_balance_red);
    white_balance.push_back(white_balance_green);
    white_balance.push_back(white_balance_blue);
    return white_balance;
}

void CameraControl::setExposureTime(int exp_time){
    if(exp_time < 0){
        aec = true;
        exposure_time = -1;
    }else{
        aec = true;
        exposure_time = (exp_time>0xFFFF)?0xFFFF:exp_time;
    }
}

int CameraControl::getExposureTime(){
    return exposure_time;
}

void CameraControl::applySavedCommand(){
    if((!agc)||(!aec)||(!awc)){
        regWrite(camera_file, 0x12, 0x00);

    }else{
        regWrite(camera_file, 0x13, 0x01);
        regWrite(camera_file, 0x00, gain);

        regWrite(camera_file, 0x08, exposure_time>>4);
        regWrite(camera_file, 0x10, exposure_time&0xFF00);

        regWrite(camera_file, 0x01, white_balance_blue);
        regWrite(camera_file, 0x02, white_balance_red);
        regWrite(camera_file, 0x03, white_balance_green);

    }
    
}

int CameraControl::regWrite(int hcam, unsigned char addr, unsigned char data){
    int rtn;

    /* Command Start */
    if  (((rtn = myIrisGetControl(hcam                     )) < 0) ||
        ((rtn = myIrisSetControl(hcam, (COM_TYPE_COM_STT) )) < 0) ||

        /* Regiter Write */
        ((rtn = myIrisGetControl(hcam                     )) < 0) ||
        ((rtn = myIrisSetControl(hcam, (COM_TYPE_REG_WR)  )) < 0) ||
        ((rtn = myIrisSetControl(hcam, (addr)             )) < 0) ||
        ((rtn = myIrisSetControl(hcam, (data)             )) < 0) ||

        /* Command End */
        ((rtn = myIrisGetControl(hcam                     )) < 0) ||
        ((rtn = myIrisSetControl(hcam, (COM_TYPE_COM_END) )) < 0))
    {
        return rtn;   /* Error */
    }else{
        return 0; /* Success */
    }
}

int CameraControl::regRead(int hcam, unsigned char addr, unsigned char *data){
    int rtn;
    int reg_data;

    /* Command Start */
    if  (((rtn = myIrisGetControl(hcam                    )) < 0) ||
        ((rtn = myIrisSetControl(hcam, (COM_TYPE_COM_STT) )) < 0) ||

        /* Regiter Read */
        ((rtn = myIrisGetControl(hcam                     )) < 0) ||
        ((rtn = myIrisSetControl(hcam, (COM_TYPE_REG_RD)  )) < 0) ||
        ((rtn = myIrisSetControl(hcam, (addr)             )) < 0))
    {
        return rtn;
    }

    if ((reg_data = myIrisGetControl(hcam            )) < 0){ 
        return rtn;
    }else{
        *data = (unsigned char)reg_data; /* register value */
    }

    /* Command End */
    if  (((rtn = myIrisGetControl(hcam                     )) < 0) ||
        ((rtn = myIrisSetControl(hcam, (COM_TYPE_COM_END) ))  < 0))
    {
        return rtn;
    }else{
        return 0;   /* Success */
    }
}


/*
 * (Sub Routine) IRIS contorol
 */

int CameraControl::myIrisGetControl(int hcam){
    struct v4l2_control control_s;
    int err;
    control_s.id = V4L2_CID_IRIS_ABSOLUTE;

    if ((err = ioctl( hcam, VIDIOC_G_CTRL, &control_s)) < 0) {
        return -1;
    }

    return control_s.value;
}

int CameraControl::myIrisSetControl(int hcam, unsigned char value){
    struct v4l2_control control_s;
    int err;

    control_s.id = V4L2_CID_IRIS_ABSOLUTE;
    control_s.value = value;

    if ((err = ioctl( hcam, VIDIOC_S_CTRL, &control_s)) < 0) {
        return -1;
    }

    return 0;
}
