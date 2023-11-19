#include "Serial.hpp"

Serial::Serial(Detector &Detector, Tracker &Tracker) {
    yawFilter.Size=50;
    pitchFilter.Size=50;
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
    sp_set_parity(serPort,SP_PARITY_NONE);
    sp_set_stopbits(serPort, 1);
    return true;
}

[[noreturn]] void Serial::send(){
    while (true){
        bool vofa=false;
        if(!sp_ret) {
            msg = "NULL";
            serial_Detector->serMsg = msg;
            sp_ret = open();
        }
        else if(sp_ret && vofa && !serial_Detector->Target_rvec.empty()){
            float data[6];
            uint8_t tail[4]={0x00,0x00,0x80,0x7f};
            data[0]=serial_Tracker->CarTracker.pos(0);
            data[1]=serial_Tracker->CarTracker.predict(0);
            data[2]=serial_Detector->Target_dis;
            data[3]=serial_Detector->Target_rvec.at<float>(0,0);
            data[4]=serial_Detector->Target_rvec.at<float>(1,0);
            data[5]=serial_Detector->Target_rvec.at<float>(2,0);
            sp_blocking_write(serPort,data,4*6,0); 
            sp_blocking_write(serPort,tail,4,0);
        }
        else if(serial_Detector->found){
            double yaw,pitch,fYaw,fPitch;
            yawFilter.update(serial_Tracker->pre_yaw);
            pitchFilter.update(serial_Tracker->pre_pitch);
            yawFilter.get_avg(fYaw);pitchFilter.get_avg(fPitch);
    //                yaw=serial_Detector->yaw;
    //                pitch=serial_Detector->offset_pitch;
            yaw=serial_Tracker->pre_yaw;
            pitch=serial_Tracker->pre_pitch;
            if(abs(yaw)>50)yaw=0;if(abs(pitch)>50)pitch=0;
            msg = "A";msg += "Y";
            if(yaw>0)msg += "+";
            else msg += "-";
            msg += cv::format("%06.2f",abs(yaw));
            msg += "P";
            if(pitch>0)msg += "+";
            else msg += "-";
            msg += cv::format("%06.2f",abs(pitch));
            if(abs(fYaw) < 5 && abs(fPitch) < 5 && yaw!=0 && pitch!=0) msg += "F";
            else msg += "N";
            msg += "E";
            sp_blocking_write(serPort,msg.c_str(),19,0);
            serial_Detector->serMsg = msg;
        }
    }
}

[[noreturn]] bool Serial::receive() {
    while(true){
        char sign;
//        sign='A';//DEBUG
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
                serial_Tracker->speed=shootSpeed!=0?shootSpeed:25.00;
                serial_Detector->readMsg = 'A'+strbuffer;
            }
        }
        else{
            serial_Detector->readMsg = "NULL";
        }
    }
}

Serial::~Serial() = default;
