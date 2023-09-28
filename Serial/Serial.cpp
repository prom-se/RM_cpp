#include "Serial.hpp"

Serial::Serial(Detector &Detector, Tracker &Tracker) {
    serial_Detector = &Detector;
    serial_Tracker = &Tracker;
    sp_ret = open();
}

bool Serial::open() {
    sp_return ret = sp_get_port_by_name("/dev/ttyUSB0", &serPort);
    if(ret != SP_OK)sp_get_port_by_name("/dev/ttyUSB1", &serPort);
    ret = sp_open(serPort,SP_MODE_READ_WRITE);
    if(ret != SP_OK) return false;
    sp_set_baudrate(serPort,115200);
    sp_set_bits(serPort, 8);
    sp_set_bits(serPort,SP_PARITY_NONE);
    sp_set_stopbits(serPort, 1);
    return true;
}

[[noreturn]] void Serial::send(){
    while (true){
        if(!sp_ret) {
            msg = "NULL";
            serial_Detector->serMsg = msg;
            sp_ret = open();
        }
        else {
            msg = "A";
            msg += "Y";
            if(serial_Detector->offset_pitch>0)msg += "+";
            else msg += "-";
            msg += cv::format("%05.2f",abs(serial_Detector->yaw));
            msg += "P";
            if(serial_Detector->yaw>0)msg += "+";
            else msg += "-";
            msg += cv::format("%05.2f",abs(serial_Detector->offset_pitch));
            if(serial_Detector->yaw < 5 && serial_Detector->offset_pitch < 5) msg += "F";
            else msg += "N";
            msg += "E";
            sp_nonblocking_write(serPort,msg.c_str(),msg.length());
            serial_Detector->serMsg = msg;
        }
    }
}

[[noreturn]] bool Serial::receive() {//TODO:串口接收
    while(true){//ABY360.00P-180.00E
        sp_nonblocking_read(serPort,buffer,1);
        if(buffer[0]=='A'){
            sp_nonblocking_read(serPort,&(buffer[1]),17);
            if(buffer[17]=='E'){
                if(buffer[1]=='B'){
                    serial_Detector->target_color = "blue";
                }
                else if(buffer[1]=='R'){
                    serial_Detector->target_color = "red";
                }
                else{
                    continue;
                }
                serial_Detector->readMsg = buffer;
            }
            else{
                serial_Detector->readMsg = "NULL";
                continue;
            }
        }
        else{
            serial_Detector->readMsg = "NULL";
            continue;
        }
    }
}

Serial::~Serial() = default;
