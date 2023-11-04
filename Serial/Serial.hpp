#ifndef RM_CPP_SERIAL_HPP
#define RM_CPP_SERIAL_HPP

#include <iostream>
#include <libserialport.h>
#include "../Detector/Detector.hpp"
#include "../Tracker/Tracker.hpp"
#include "../Tracker/avgFilter.hpp"

class Serial {
private:
    Detector *serial_Detector;
    Tracker *serial_Tracker;
    bool sp_ret;
    class avgFilter yawFilter;
    class avgFilter pitchFilter;
public:
    std::string msg;
//    char buffer[25]="BY 111.11P  22.22S33.33E";
    char buffer[25];
    struct sp_port *serPort;

    double sp_yaw,sp_pitch;

    Serial(class Detector &Detector, class Tracker &Tracker);

    //开启设备
    bool open();

    //发送数据
    [[noreturn]] void send();

    //接受数据
    [[noreturn]] bool receive();

    ~Serial();


};


#endif //RM_CPP_SERIAL_HPP
