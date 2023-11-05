#include "Detector.hpp"
#include <opencv2/opencv.hpp>
#include <vector>
#include <chrono>


//延长灯条四点，提取数字时使用
bool light_ex(const std::vector<cv::Point2f>& src, cv::Point2f dst[]){
    dst[0].x = 2*src[0].x-src[1].x;dst[0].y = 2*src[0].y-src[1].y;
    dst[1].x = 2*src[1].x-src[0].x;dst[1].y = 2*src[1].y-src[0].y;
    dst[2].x = 2*src[2].x-src[3].x;dst[2].y = 2*src[2].y-src[3].y;
    dst[3].x = 2*src[3].x-src[2].x;dst[3].y = 2*src[3].y-src[2].y;
    return true;
};

//点序修正
bool points_sort(std::vector<cv::Point2f> &points){
    const std::vector<cv::Point2f> raw_points = points;
    if (points[0].y < points[2].y){
        points[0] = raw_points[0];
        points[1] = raw_points[3];
        points[2] = raw_points[1];
        points[3] = raw_points[2];
    }
    else if(points[0].y > points[2].y){
        points[0] = raw_points[1]; //排序后点序为 0：左上 1：左下 2：右上 3：右下
        points[1] = raw_points[0];
        points[2] = raw_points[2];
        points[3] = raw_points[3];
    }
    return true;
};

//数字识别
bool Detector::number_classify(){
    cv::Point2f armor_points[4];
    light_ex(Armor.pix_position[Armor.nums-1], armor_points);
    cv::Point2f roi_points[4];
    roi_points[0].x = 0;roi_points[0].y = 0;
    roi_points[1].x = 0;roi_points[1].y = 640;
    roi_points[2].x = 800;roi_points[2].y = 0;
    roi_points[3].x = 800;roi_points[3].y = 640;
    cv::Mat w_mat = cv::getPerspectiveTransform(armor_points,roi_points);
    cv::warpPerspective(src, roi, w_mat, cv::Size(800,640));
    cv::cvtColor(roi, roi,cv::COLOR_BGR2GRAY);
    roi(cv::Rect(160, 100, 480, 440)).copyTo(roi);

    roi.convertTo(roi,CV_32F,1.f/255.f);
    double min,max;
    cv::minMaxLoc(roi.reshape(1,1), &min, &max, nullptr, nullptr);
    roi=roi-min;max=max-min;
    roi=roi/max;
    cv::pow(roi,0.5f,roi);
    roi.convertTo(roi_bin,CV_8U,255.f);

//    cv::threshold(roi_bin, roi_bin,150,255, 0);
    cv::threshold(roi_bin, roi_bin,0,255,cv::THRESH_BINARY | cv::THRESH_OTSU);
    cv::resize(roi_bin,roi_bin,cv::Size(20,28));

    cv::Mat blob;
    cv::dnn::blobFromImage(roi_bin, blob,1.0,cv::Size(20,28));
    net.setInput(blob);
    cv::Mat outputs = net.forward();

    cv::Point class_id_point;
    double confidence;
    cv::minMaxLoc(outputs.reshape(1,1), nullptr, &confidence, nullptr, &class_id_point);
    int label_id = class_id_point.x;
//    label_id=0;//DEBUG
    Armor.number.emplace_back(num_classes[label_id]);

    return true;
}

//寻找灯条
bool Detector::findLightBar() {
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    std::vector<cv::Point2f> lightPoint;
    cv::RotatedRect rect;
    cv::findContours(dst, contours, hierarchy,cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) return false;
    for(const auto & contour : contours){
        bool passed;
        cv::Size2d size; double theta;
        rect = cv::minAreaRect(contour);
        rect.points(lightPoint);
        theta = rect.angle;
        if (45<=theta && theta<=90) theta = theta - 90;
        passed = points_sort(lightPoint);
        size.height = sqrt(pow(lightPoint[1].x - lightPoint[0].x,2)+pow(lightPoint[1].y - lightPoint[0].y,2));
        size.width = sqrt(pow(lightPoint[0].x - lightPoint[2].x,2)+pow(lightPoint[0].y - lightPoint[2].y,2));
        //cv::putText(src, cv::format("theta:%f", theta), lightPoint[0],cv::FONT_HERSHEY_SIMPLEX, 1,cv::Scalar(0,0,255), 2);
        if (size.height/(size.width+0.01)<1.5 || size.height/(size.width+0.01) >8) passed = false;
        if ((double)size.area()<3) passed = false;
        if (abs(theta)>45) passed = false;
        cv::Scalar bgr;
        bgr = cv::mean(src(cv::boundingRect(contour)));
        std::string lightColor;
        lightColor = bgr.val[0]>bgr.val[1] ? "blue":"red";
        if (lightColor!=target_color) passed = false;
        if (passed){
            susBar.nums++;
            susBar.size.emplace_back(size);
            susBar.theta.emplace_back(theta);
            susBar.position.emplace_back(lightPoint);
            susBar.matched.emplace_back(false);
        }
    }
    return true;
}

//检测两疑似灯条之间是否有灯条
bool Detector::containTest(int i, int j) {
    for(int k=0;k<susBar.nums;k++){
        if(k==i || k == j) continue;
        float x = (susBar.position[k][0].x+susBar.position[k][2].x)/2;
        float yt = susBar.position[k][0].y;
        float yb = susBar.position[k][1].y;
        float test_xl = susBar.position[i][0].x<susBar.position[j][0].x ? (susBar.position[i][0].x+susBar.position[i][2].x)/2:(susBar.position[j][0].x+susBar.position[j][2].x)/2;
        float test_xr = susBar.position[i][0].x>susBar.position[j][0].x ? (susBar.position[i][0].x+susBar.position[i][2].x)/2:(susBar.position[j][0].x+susBar.position[j][2].x)/2;
        float test_yt = susBar.position[i][0].y<susBar.position[j][0].y ? (susBar.position[i][0].y+susBar.position[i][2].y)/2:(susBar.position[j][0].y+susBar.position[j][2].y)/2;
        float test_yb = susBar.position[i][0].y>susBar.position[j][0].y ? (susBar.position[i][0].y+susBar.position[i][2].y)/2:(susBar.position[j][0].y+susBar.position[j][2].y)/2;
        if(test_xl<x && x<test_xr and test_yt<yt && yt<test_yb || test_yt<yb && yb<test_yb) return false;
    }
    return true;
}

//匹配灯条，获得装甲板
bool Detector::matchLightBar(){
    if(susBar.nums){
        for(int i=0;i<susBar.nums;i++){
            for(int j=i+1;j<susBar.nums;j++){
                if(!susBar.matched[i]
                        and !susBar.matched[j]
                        and (susBar.size[i].width/susBar.size[j].width)<1.5
                        and (susBar.size[i].height/susBar.size[j].height)<1.5
                        and abs(susBar.theta[i]-susBar.theta[j])<20
                        and containTest(i,j)){
                    susBar.matched[i] = true;
                    susBar.matched[j] = true;
                    Armor.leftBar.nums++;
                    Armor.rightBar.nums++;
                    Armor.nums++;
                    if (susBar.position[i][0].x < susBar.position[j][0].x){
                        Armor.leftBar.size.emplace_back(susBar.size[i]);
                        Armor.leftBar.theta.emplace_back(susBar.theta[i]);
                        Armor.leftBar.position.emplace_back(susBar.position[i]);
                        Armor.rightBar.size.emplace_back(susBar.size[j]);
                        Armor.rightBar.theta.emplace_back(susBar.theta[j]);
                        Armor.rightBar.position.emplace_back(susBar.position[j]);
                    }
                    else{
                        Armor.leftBar.size.emplace_back(susBar.size[j]);
                        Armor.leftBar.theta.emplace_back(susBar.theta[j]);
                        Armor.leftBar.position.emplace_back(susBar.position[j]);
                        Armor.rightBar.size.emplace_back(susBar.size[i]);
                        Armor.rightBar.theta.emplace_back(susBar.theta[i]);
                        Armor.rightBar.position.emplace_back(susBar.position[i]);
                    }
                    Armor.pix_position.emplace_back(susBar.position[0]);
                    Armor.pix_position[Armor.nums - 1][0].x = (Armor.leftBar.position[Armor.nums - 1][0].x +
                                                               Armor.leftBar.position[Armor.nums - 1][2].x) / 2;
                    Armor.pix_position[Armor.nums - 1][1].x = (Armor.leftBar.position[Armor.nums - 1][1].x +
                                                               Armor.leftBar.position[Armor.nums - 1][3].x) / 2;
                    Armor.pix_position[Armor.nums - 1][2].x = (Armor.rightBar.position[Armor.nums - 1][0].x +
                                                               Armor.rightBar.position[Armor.nums - 1][2].x) / 2;
                    Armor.pix_position[Armor.nums - 1][3].x = (Armor.rightBar.position[Armor.nums - 1][1].x +
                                                               Armor.rightBar.position[Armor.nums - 1][3].x) / 2;
                    Armor.pix_position[Armor.nums - 1][0].y = (Armor.leftBar.position[Armor.nums - 1][0].y +
                                                               Armor.leftBar.position[Armor.nums - 1][2].y) / 2;
                    Armor.pix_position[Armor.nums - 1][1].y = (Armor.leftBar.position[Armor.nums - 1][1].y +
                                                               Armor.leftBar.position[Armor.nums - 1][3].y) / 2;
                    Armor.pix_position[Armor.nums - 1][2].y = (Armor.rightBar.position[Armor.nums - 1][0].y +
                                                               Armor.rightBar.position[Armor.nums - 1][2].y) / 2;
                    Armor.pix_position[Armor.nums - 1][3].y = (Armor.rightBar.position[Armor.nums - 1][1].y +
                                                               Armor.rightBar.position[Armor.nums - 1][3].y) / 2;

                    Armor.theta.emplace_back(abs(
                            atan2l(Armor.pix_position[Armor.nums-1][2].y-Armor.pix_position[Armor.nums-1][0].y,
                                   Armor.pix_position[Armor.nums-1][2].x-Armor.pix_position[Armor.nums-1][0].x))*180/CV_PI);

                    Armor.center.emplace_back(Armor.pix_position[0][0]);
                    Armor.center[Armor.nums - 1].x =
                            (Armor.pix_position[Armor.nums - 1][0].x + Armor.pix_position[Armor.nums - 1][1].x +
                             Armor.pix_position[Armor.nums - 1][2].x + Armor.pix_position[Armor.nums - 1][3].x) / 4;
                    Armor.center[Armor.nums - 1].y =
                            (Armor.pix_position[Armor.nums - 1][0].y + Armor.pix_position[Armor.nums - 1][1].y +
                             Armor.pix_position[Armor.nums - 1][2].y + Armor.pix_position[Armor.nums - 1][3].y) / 4;

                    Armor.pix_distance.emplace_back(
                            sqrt(pow(Armor.center[i].x - 640, 2) + pow(Armor.center[i].y - 512, 2)));

                    Detector::number_classify();

                    Armor.size.emplace_back(
                            Armor.pix_position[Armor.nums - 1][2].x - Armor.pix_position[Armor.nums - 1][0].x,
                            Armor.pix_position[Armor.nums - 1][1].y - Armor.pix_position[Armor.nums - 1][0].y);

                    if (Armor.number[Armor.nums - 1] == "1" || Armor.size[Armor.nums - 1].width / Armor.size[Armor.nums - 1].height > 3.75) {Armor.type.emplace_back('b');}
                    else if(Armor.number[Armor.nums - 1] == "Outpost" || Armor.number[Armor.nums - 1] == "2") {Armor.type.emplace_back('s');}
                    else Armor.type.emplace_back('s');
                    if (Armor.number[Armor.nums - 1] != "Negative" && Armor.pix_distance[Armor.nums - 1] <= Armor.pix_distance[Armor.best_index]) Armor.best_index = Armor.nums - 1;
                    if (Armor.number[Armor.nums - 1] == "Negative"
                        or Armor.theta[Armor.nums - 1] > 45){
                        susBar.matched[i] = false;
                        susBar.matched[j] = false;
                        Armor.leftBar.nums--;
                        Armor.rightBar.nums--;
                        Armor.nums--;
                        Armor.leftBar.position.pop_back();
                        Armor.leftBar.theta.pop_back();
                        Armor.leftBar.size.pop_back();
                        Armor.rightBar.position.pop_back();
                        Armor.rightBar.theta.pop_back();
                        Armor.rightBar.size.pop_back();
                        Armor.pix_position.pop_back();
                        Armor.theta.pop_back();
                        Armor.center.pop_back();
                        Armor.pix_distance.pop_back();
                        Armor.size.pop_back();
                        Armor.number.pop_back();
                        Armor.type.pop_back();
                    }
                }
            }
        }
    }
    else return false;
    return true;
}

//执行检测任务
bool Detector::detect(){
    memset(&Armor, 0, sizeof(Armor));
    memset(&susBar, 0, sizeof(susBar));
    memset(&rune.boxes, 0, sizeof(rune.boxes));
    memset(&rune.indexes, 0, sizeof(rune.indexes));
    memset(&rune.classIds, 0, sizeof(rune.classIds));
    memset(&rune.confidences, 0, sizeof(rune.confidences));
    memset(&rune.targets, 0, sizeof(rune.targets));

    src.copyTo(show);
    frames++;
    bool ret;
    ret = !src.empty();
    if(!ret) return false;
    if(!isRune){
        cv::cvtColor(src, dst,cv::COLOR_BGR2GRAY);
        dst.copyTo(gray);
        cv::threshold(dst, dst, ThresholdValue, 255, 0);
        ret = findLightBar();
        if(!ret) return false;
        ret = matchLightBar();
        if(!ret) return false;
    }
    else{
        rune.targets.color = target_color;

        rune.prepare(src);
//        rune.infer_v5();
        rune.infer_v8();
        rune.findTarget();
    }
    return true;
}

//绘图，debug时调用
void Detector::draw(){
    if(!Target_tvec.empty()){
        cv::putText(show, cv::format("x:%.2fcm", Target_tvec.at<double>(0,0)),
                    cv::Point2i(50,50),cv::FONT_HERSHEY_SIMPLEX,
                    1,cv::Scalar(0,255,255),2);
        cv::putText(show, cv::format("y:%.2fcm", Target_tvec.at<double>(0,1)),
                    cv::Point2i(250,50),cv::FONT_HERSHEY_SIMPLEX,
                    1,cv::Scalar(0,255,255),2);
        cv::putText(show, cv::format("z:%.2fcm", Target_tvec.at<double>(0,2)),
                    cv::Point2i(450,50),cv::FONT_HERSHEY_SIMPLEX,
                    1,cv::Scalar(0,255,255),2);
    }
    if(!isnan(Target_dis)){
        cv::putText(show, cv::format("distance:%.2fcm", Detector::Target_dis),
                    cv::Point2i(50,100),cv::FONT_HERSHEY_SIMPLEX,
                    1,cv::Scalar(0,255,255),2);
        cv::putText(show, cv::format("yaw:%.4f pitch:%.4f offset:%.4f",yaw,pitch,offset_pitch),
                    cv::Point2i(50,150),cv::FONT_HERSHEY_SIMPLEX,
                    1,cv::Scalar(0,255,255),2);
    }
    for(const std::vector<cv::Point2f>& point:susBar.position){
        cv::line(show, point[0], point[1], cv::Scalar(0,255,0), 1);
        cv::line(show, point[0], point[2], cv::Scalar(0,255,0), 1);
        cv::line(show, point[3], point[1], cv::Scalar(0,255,0), 1);
        cv::line(show, point[3], point[2], cv::Scalar(0,255,0), 1);
        /*
        cv::putText(show, "0", point[0],cv::FONT_HERSHEY_SIMPLEX, 1,cv::Scalar(0,0,255), 2);
        cv::putText(show, "1", point[1],cv::FONT_HERSHEY_SIMPLEX, 1,cv::Scalar(0,0,255), 2);
        cv::putText(show, "2", point[2],cv::FONT_HERSHEY_SIMPLEX, 1,cv::Scalar(0,0,255), 2);
        cv::putText(show, "3", point[3],cv::FONT_HERSHEY_SIMPLEX, 1,cv::Scalar(0,0,255), 2);
        */
        
    }
    for(const std::vector<cv::Point2f>& point:Armor.pix_position) {
        cv::line(show, point[0], point[1], cv::Scalar(255, 0, 255), 2);
        cv::line(show, point[0], point[2], cv::Scalar(255, 0, 255), 2);
        cv::line(show, point[3], point[1], cv::Scalar(255, 0, 255), 2);
        cv::line(show, point[3], point[2], cv::Scalar(255, 0, 255), 2);
    }
    for(const cv::Point2f& point:Armor.center){
        cv::circle(show, point, 4, cv::Scalar(0,0,255), 1);
    }
    for(int i=0;i<Armor.nums;i++){
        cv::putText(show,Armor.number[i],Armor.pix_position[i][0], cv::FONT_HERSHEY_SIMPLEX,
                    1,cv::Scalar(0,255,255),2);
        cv::putText(show,cv::format("%c",Armor.type[i]),Armor.pix_position[i][2], cv::FONT_HERSHEY_SIMPLEX,
                    1,cv::Scalar(0,255,255),2);
    }
    for(int i=0;i<rune.targets.pix_position.size();i++) {
        std::vector<cv::Point2f> point = rune.targets.pix_position[i];
        if(i==rune.targets.index_R) cv::putText(show(cv::Rect(show.cols/2-640,show.rows/2-512,1280,1024)), "R", point[0],cv::FONT_HERSHEY_SIMPLEX, 1,cv::Scalar(0,0,255), 2);
        else if(i==rune.targets.index_Target) cv::putText(show(cv::Rect(show.cols/2-640,show.rows/2-512,1280,1024)), "Target", point[0],cv::FONT_HERSHEY_SIMPLEX, 1,cv::Scalar(0,0,255), 2);
        else if(std::find(rune.targets.index_Activatied.begin(),rune.targets.index_Activatied.end(),i)!=rune.targets.index_Activatied.end())cv::putText(show(cv::Rect(show.cols/2-640,show.rows/2-512,1280,1024)), "Activated", point[0],cv::FONT_HERSHEY_SIMPLEX, 1,cv::Scalar(0,0,255), 2);
        cv::line(show(cv::Rect(show.cols/2-640,show.rows/2-512,1280,1024)), point[0], point[1], cv::Scalar(255, 0, 255), 2);
        cv::line(show(cv::Rect(show.cols/2-640,show.rows/2-512,1280,1024)), point[0], point[2], cv::Scalar(255, 0, 255), 2);
        cv::line(show(cv::Rect(show.cols/2-640,show.rows/2-512,1280,1024)), point[3], point[1], cv::Scalar(255, 0, 255), 2);
        cv::line(show(cv::Rect(show.cols/2-640,show.rows/2-512,1280,1024)), point[3], point[2], cv::Scalar(255, 0, 255), 2);

    }
    show.copyTo(drawed);
}

//Debug函数，控制台输出检测信息，显示GUI
void Detector::debug(long &start_time, cv::Mat &show_mat, bool show_flag=false){
    Detector::draw();
    now_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    delta_time = delta_time + (double)(now_time - start_time)/1000.0;
    fps = frames*1000.0 / (delta_time);
    if(frames==50) {
        if(isRune)printf("Rune Mode\n");
        else printf("Armor Mode\n");
        if(Armor.nums && !isRune){
            printf("Armor FOUND !!\n");
            printf("Number:%s\n", Armor.number[Armor.best_index].c_str());
            printf("Distance:%.2fcm\n", Target_dis);
        }
        else if(!rune.boxes.empty() && isRune){
            printf("Rune FOUND !!\n");
            printf("Distance:%.2fcm\n", Target_dis);
        }
        else{
            printf("NOT FOUND!\n");
            printf("Distance:NULL\n");
        }
        printf("Send:%s\n", serMsg.c_str());
        printf("Read:%s\n", readMsg.c_str());
        printf("Delay:%.1fms\n", delta_time / frames);
        printf("FPS:%.1fhz\n", fps);
        time_t now = time(nullptr);
        printf("Time:%s", ctime(&now));
        printf("-------------------------------\n");
        delta_time = 0;
        frames = 0;
    }
    if (show_flag){
        cv::namedWindow("show",cv::WINDOW_NORMAL);
        cv::namedWindow("show_mat",cv::WINDOW_NORMAL);
        cv::imshow("show_mat", show_mat);
        cv::imshow("show", show);
        if((cv::waitKey(1) & 0xFF) == 0x71 || (cv::waitKey(1) & 0xFF) == 0x51) start_time = -1;
        else start_time = now_time;
    }
    else start_time = now_time;
}

void Detector::log(){
    time_t now = time(nullptr);
    writer.open(cv::format("../log/%s.avi",ctime(&now)),cv::VideoWriter::fourcc('D','I','V','X'), 100, cv::Size(1280, 1024));
    while(!drawed.empty()){
        writer.write(drawed);
    }
}
