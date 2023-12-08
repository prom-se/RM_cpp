#include "autoaim.hpp"

int main(){
    //相机初始化
    if(!CamInit()) return 1;
    //串口发送线程
    Serial_.sendThread();
    //串口接收线程
    Serial_.reciveThread();
    //log线程
    // std::thread write(&Detector::log,std::ref(Detector_));
    // write.detach();
    //主循环
    do{
        //检测装甲板
        Detector_.target_color=!Tracker_.rMsg.foeColor?"blue":"red";
        Detector_.detect();
        //解算，跟踪目标
        Tracker_.track();
        Tracker_.vMsg.head=0xa5;
        Serial_.visionUpdate(&Tracker_.vMsg);
        Serial_.robotUpdate(&Tracker_.rMsg);
        if(!Detector_.found){Tracker_.vMsg.aimPitch=0;Tracker_.vMsg.aimYaw=0;}
        Detector_.serMsg=cv::format("Y:%05.2f/P:%05.2f",Tracker_.vMsg.aimYaw,Tracker_.vMsg.aimPitch);
        Detector_.readMsg=cv::format("SelfY:%05.2f/SelfP:%05.2f",Tracker_.rMsg.robotYaw,Tracker_.rMsg.robotPitch);
        //DeBug绘图并GUI显示
        Detector_.debug(start_time, Detector_.roi_bin,true);
    }while(start_time != -1);
    return 0;
}
