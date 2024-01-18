
#ifndef _AUTO_AIM_H_
#define _AUTO_AIM_H_

#include <thread>
#include <opencv2/opencv.hpp>
#include <unistd.h>
#include "hik_camera/HikCam.hpp"
#include "Detector/Detector.hpp"
#include "Tracker/Tracker.hpp"
#include "visionSerial/include/serial.hpp"

Detector Detector_;
Tracker Tracker_(Detector_);
HikCam Hik;
long start_time = 0;

//是否使用海康相机
#define USE_HIK

//图像读取进程
cv::VideoCapture cap("../image/R.avi");
[[noreturn]] void CapThread(){
    while(true){
#ifdef USE_HIK
        Hik.GetMat(Detector_.buff);
#else
        cap.read(Detector_.buff);
        cv::waitKey(15);
#endif
    }
};

//相机初始化
bool CamInit(){
#ifdef USE_HIK
    if(Hik.StartDevice(0) != 0) return false;//开启相机
    Hik.SetResolution(1280, 1024);//设置分辨率
    Hik.SetPixelFormat(17301512);//设置像素格式PixelType_Gvsp_BayerGR8
    Hik.SetExposureTime(5000);//设置曝光时间
    Hik.SetGAIN(5.0);
    Hik.SetFrameRate(120);//设置帧率上限
    Hik.SetStreamOn();//开始取流
    printf("[CamFPS] %.1fhz\n", Hik.GetFrameRate());//输出实际帧率
#endif
    //VideoCapture读取图像线程
    std::thread Cam(CapThread);
    Cam.detach();
    return true;
};
#endif