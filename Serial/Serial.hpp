#ifndef RM_CPP_SERIAL_HPP
#define RM_CPP_SERIAL_HPP

#include <iostream>
#include <libserialport.h>
#include "../Detector/Detector.hpp"
#include "../Tracker/Tracker.hpp"

class Serial {
public:
    Detector *serial_Detector;
    Tracker *serial_Tracker;
    bool sp_ret;

    std::string msg;
//    char buffer[25]="BY 111.11P  22.22S33.33E";
    char buffer[25];
    struct sp_port *serPort;

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
