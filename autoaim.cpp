#include "autoaim.hpp"

int main(){
    //相机初始化
    if(!CamInit()) return 1;
    //打开串口
    visionSerial Serial_("/dev/ttyACM0",115200);
    //log线程
    // std::thread write(&Detector::log,std::ref(Detector_));
    // write.detach();
    //主循环
    while(start_time != -1){
        //检测装甲板
        Serial_.robotUpdate(*Tracker_.rMsg);
        Detector_.target_color=!Tracker_.rMsg->foeColor?"blue":"red";
        Detector_.isRune=Tracker_.rMsg->mode!=0?true:false;
        Tracker_.BuffTracker.isSmall=Tracker_.rMsg->mode==1?true:false;
        Detector_.detect();
        //解算，跟踪目标
        Tracker_.track();
        if(!Detector_.found&&Detector_.rune.targets.pix_position.size()<2){Tracker_.vMsg->aimPitch=0;Tracker_.vMsg->aimYaw=0;}
        Tracker_.vMsg->fire = Tracker_.vMsg->aimPitch<3 && Tracker_.vMsg->aimYaw<3?1:0;
        Serial_.visionUpdate(*Tracker_.vMsg);
        if(Serial_.isOk){
            Detector_.serMsg=cv::format("Y:%05.2f/P:%05.2f",Tracker_.vMsg->aimYaw,Tracker_.vMsg->aimPitch);
            Detector_.readMsg=cv::format("SelfY:%05.2f/SelfP:%05.2f",Tracker_.rMsg->robotYaw,Tracker_.rMsg->robotPitch);
        }
        else{
            Detector_.serMsg="ERROR";
            Detector_.readMsg="ERROR";
        }
        //DeBug绘图并GUI显示
        Detector_.debug(start_time, Detector_.roi_bin,true);
    };
    return 0;
}
