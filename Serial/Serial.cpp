#include "Serial.hpp"

Serial::Serial(Detector &Detector, Tracker &Tracker) {
    serial_Detector = &Detector;
    serial_Tracker = &Tracker;
    sp_ret = open();
}

bool Serial::open() {
    sp_return ret = sp_get_port_by_name("/dev/ttyACM0", &serPort);
    if(ret != SP_OK)sp_get_port_by_name("/dev/ttyUSB0", &serPort);
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
            msg = "A";msg += "Y";
            if(serial_Detector->offset_pitch>0)msg += "+";
            else msg += "-";
            msg += cv::format("%06.2f",abs(serial_Detector->yaw));
            msg += "P";
            if(serial_Detector->yaw>0)msg += "+";
            else msg += "-";
            msg += cv::format("%06.2f",abs(serial_Detector->offset_pitch));
            if(serial_Detector->yaw < 5 && serial_Detector->offset_pitch < 5) msg += "F";
            else msg += "N";
            msg += "E";
            sp_nonblocking_write(serPort,msg.c_str(),msg.length());
            serial_Detector->serMsg = msg;
        }
    }
}

[[noreturn]] bool Serial::receive() {
    while(true){
        char sign;
        sp_nonblocking_read(serPort,&sign,1);
        if(sign=='A'){
            sp_nonblocking_read(serPort,&buffer,24);
            if(buffer[23]=='E' && buffer[1]=='Y' && buffer[9]=='P' && buffer[17]=='S'){
                serial_Detector->target_color = buffer[0]=='B' ? "blue":"red";
                float selfYaw,selfPitch,shootSpeed;
                std::string strbuffer = buffer;
                selfYaw=std::stof(strbuffer.substr(2,7));
                selfPitch=std::stof(strbuffer.substr(10,7));
                shootSpeed=std::stof(strbuffer.substr(18,5));
                serial_Tracker->selfYaw=selfYaw;serial_Tracker->selfPitch=selfPitch;
                serial_Tracker->speed=shootSpeed;
                serial_Detector->readMsg = 'A'+strbuffer;
            }
        }
        else{
            serial_Detector->readMsg = "NULL";
        }
    }
}

Serial::~Serial() = default;
