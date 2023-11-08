#include "autoaim.hpp"

int main(){
    //相机初始化
    if(!CamInit()) return 1;
    //串口发送线程
    std::thread send(&Serial::send,std::ref(Serial_));
    send.detach();
    //串口接收线程
    std::thread read(&Serial::receive,std::ref(Serial_));
    read.detach();
    //log线程
    std::thread write(&Detector::log,std::ref(Detector_));
    write.detach();
    //主循环
    do{
        //检测装甲板
        Detector_.detect();
        //解算，跟踪目标
        Tracker_.track();
        //DeBug绘图并GUI显示
        Detector_.debug(start_time, Detector_.roi_bin,false);
    }while(start_time != -1);
    return 0;
}
