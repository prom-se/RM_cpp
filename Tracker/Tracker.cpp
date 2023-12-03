
#include "Tracker.hpp"

bool Tracker::pnpSolve(){
    cv::Mat tvec;
    cv::Mat rvec;
    if(!track_Detector->isRune){
        if(!track_Detector->found) {
            pre_yaw=0;pre_pitch=0;
            return false;
        }
        const std::vector<cv::Point3d> *Armor_Point;
        Armor_Point = track_Detector->Armor.type[track_Detector->Armor.best_index] == 'b' ? &large_Armor:&small_Armor;
        cv::solvePnP(*Armor_Point, track_Detector->Armor.pix_position[track_Detector->Armor.best_index],
                     cameraMatrix, distCoeffs, rvec, tvec, false,cv::SOLVEPNP_IPPE);

        tvec.at<double>(0,1) = -tvec.at<double>(0,1);
        tvec.at<double>(0,0) += xyzFix.at(0);
        tvec.at<double>(0,1) += xyzFix.at(1);
        tvec.at<double>(0,2) += xyzFix.at(2);

        track_Detector->Target_rvec = rvec;track_Detector->Target_tvec = tvec/10;
        track_Detector->Target_dis = sqrt(pow(tvec.at<double>(0,0),2)+pow(tvec.at<double>(0,1),2)+pow(tvec.at<double>(0,2),2))/10;
        track_Detector->yaw = atan2(tvec.at<double>(0,0),tvec.at<double>(0,2)) * 180 / CV_PI;
        track_Detector->pitch = atan2(tvec.at<double>(0,1),tvec.at<double>(0,2)) * 180 / CV_PI;
    }
    else{
        if(track_Detector->rune.targets.index_Target == -1) return false;
//        cv::solvePnP(rune_Point, track_Detector->rune.targets.pix_position[track_Detector->rune.targets.index_Target],
//                     cameraMatrix, distCoeffs, rvec, tvec, false,cv::SOLVEPNP_IPPE_SQUARE);
        track_Detector->Target_dis = sqrt(633.8*633.8+pow(140*cos(BuffTracker.targetTheta/180*CV_PI),2));
    }
    disFilter.update(track_Detector->Target_dis);
    disFilter.get_avg(track_Detector->Target_dis);
    return true;
}


bool Tracker::offset(){
    if(!track_Detector->Armor.nums){
        return false;
    }
    int iteration = 20;
    double aim_y, real_y, theta, time;
    double x_distance, y_distance;
    if(!track_Detector->isRune){
        x_distance = sqrt(pow(track_Detector->Target_tvec.at<double>(0,0),2)+pow(track_Detector->Target_tvec.at<double>(0,2),2))/100;
        y_distance = track_Detector->Target_tvec.at<double>(0,1)/100;
    }
    else if (track_Detector->isRune){
        x_distance = track_Detector->Target_dis/100;
        y_distance = 1.4 * sin(BuffTracker.targetTheta/180*CV_PI);
    }
    aim_y = y_distance;
    speed=rMsg.muzzleSpeed!=0?rMsg.muzzleSpeed:25.00;
    for(int i=0;i<iteration;i++){
        theta = atan2(aim_y,x_distance);
        time = (exp(air_k*x_distance)-1)/(air_k*speed*cos(theta));
        real_y = speed*sin(theta)*time - Gravity*(time*time)/2;
        aim_y = aim_y+(y_distance-real_y);
        if (abs(real_y-y_distance)<0.001){
            offset_time=time;
            track_Detector->offset_y = aim_y*100;
            break;
        }
    }
    track_Detector->offset_pitch = atan2(track_Detector->offset_y/100,x_distance)*180/CV_PI;
    return true;
}

bool Tracker::track() {
    bool ret;
    ret = pnpSolve();
    if(!ret) return false;
    offset();
    track_Detector->offset_pitch += PitchFix.at(0);
    trackTarget();
    draw();
    return true;
}

void Tracker::trackTarget() {
    if(track_Detector->isRune){
        buff_init();
    }
    else{
        car_init();
        double k=4;pre_k=k*2;
        ekf_filter.dt(k);
        ekf_filter.predict();
        ekf_filter.update(CarTracker.pos);
        ekf_filter.pre_time=pre_k*offset_time;
        CarTracker.predict=ekf_filter.pre_position;
        car_reFind();
    }
}

void Tracker::buff_init(){
    BuffTracker.color = track_Detector->target_color;
    if(track_Detector->rune.targets.index_R != -1) BuffTracker.R_position = track_Detector->rune.targets.center[track_Detector->rune.targets.index_R];
    double r = 0;
    for(int i=0;i<track_Detector->rune.targets.center.size();i++){
        if(i != track_Detector->rune.targets.index_R){
            r = r + sqrt(pow(track_Detector->rune.targets.center[i].x - BuffTracker.R_position.x,2)+
                        pow(track_Detector->rune.targets.center[i].y - BuffTracker.R_position.y,2));
        }
    }
    if(track_Detector->rune.targets.index_R != -1) BuffTracker.radius = r / ((double)track_Detector->rune.targets.center.size()-1);
    else BuffTracker.radius = r / (double)track_Detector->rune.targets.center.size();
    if(BuffTracker.isSmall) {
        BuffTracker.spd = 60;  //小能量机关转速固定10RPM
        double k = (track_Detector->rune.targets.center[track_Detector->rune.targets.index_Target].x-BuffTracker.R_position.x)/BuffTracker.radius;
        if(k <-1) k = -1;
        if(k > 1) k =  1;
        BuffTracker.targetTheta = acos(k)* 180 / CV_PI;
        BuffTracker.targetTheta = track_Detector->rune.targets.center[track_Detector->rune.targets.index_Target].y<BuffTracker.R_position.y ? BuffTracker.targetTheta : 360-BuffTracker.targetTheta;
        double predic = BuffTracker.color=="blue"?BuffTracker.spd*offset_time*3/2:-BuffTracker.spd*offset_time*3/2;
        BuffTracker.preTheta=BuffTracker.targetTheta+predic;
        BuffTracker.Target_position.x = BuffTracker.radius*cos( BuffTracker.preTheta*CV_PI/180)+BuffTracker.R_position.x;
        BuffTracker.Target_position.y = -BuffTracker.radius*sin( BuffTracker.preTheta*CV_PI/180)+BuffTracker.R_position.y;
    }
    else if(!BuffTracker.isSmall){
        double k = (track_Detector->rune.targets.center[track_Detector->rune.targets.index_Target].x-BuffTracker.R_position.x)/BuffTracker.radius;
        if(k <-1) k = -1;
        if(k > 1) k =  1;
        BuffTracker.targetTheta = acos(k)* 180 / CV_PI;
        BuffTracker.targetTheta = track_Detector->rune.targets.center[track_Detector->rune.targets.index_Target].y<BuffTracker.R_position.y ? BuffTracker.targetTheta : 360-BuffTracker.targetTheta;
        Big_buff_track();
        BuffTracker.Target_position.x = BuffTracker.radius*cos( BuffTracker.preTheta*CV_PI/180)+BuffTracker.R_position.x;
        BuffTracker.Target_position.y = -BuffTracker.radius*sin( BuffTracker.preTheta*CV_PI/180)+BuffTracker.R_position.y;
    }
    double fx = cameraMatrix.at<double>(0, 0);
    double fy = cameraMatrix.at<double>(1, 1);
    double cx = cameraMatrix.at<double>(0, 2);
    double cy = cameraMatrix.at<double>(1, 2);
    track_Detector->yaw = atan((BuffTracker.Target_position.x - cx) / fx) * 180/CV_PI;
    track_Detector->offset_pitch = atan((cy - BuffTracker.Target_position.y) / fy) * 180/CV_PI +track_Detector->offset_pitch-track_Detector->pitch;
}

void Tracker::Big_buff_track() {
    //TODO:大能量机关预测（此处还有bug）
    double now_time = (double)std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count()/1000000.0-start_time;
    double delta_time;
    if(last_time != -1){
        bool ret = true;
        delta_time = now_time - last_time;
        last_time = now_time;
        double delta_r = abs(BuffTracker.targetTheta - BuffTracker.lastTheta);
        if (350<delta_r && delta_r<360) {
            delta_r = (360-delta_r)/180*CV_PI;
        }
        else if(delta_r<72){
            delta_r = delta_r/180*CV_PI;
        }
        else{
            ret = false;
        }
        if(ret){
            t.emplace_back(now_time);
            dt.emplace_back(delta_time);
            dr.emplace_back(delta_r);
//            std::cout << t[t.size()-1] << " " << dr[t.size()-1] << std::endl;
       }
    }
    else{
        start_time = (double)std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count()/1000000.0;
        last_time = 0;
        BuffTracker.lastTheta = BuffTracker.targetTheta;
    }
    if(t.size() == raw_points && !t.empty()){
        for (int i = 0; i < raw_points; ++i) {
            problem.AddResidualBlock(
                    new ceres::AutoDiffCostFunction<TrigResidual, 1, 1, 1, 1, 1>(new TrigResidual(t[t.size()-1-i], dr[t.size()-1-i], dt[t.size()-1-i])),
                     nullptr,
                        &a, &b ,&c, &d);
        }
        ceres::Solve(options, &problem, &summary);
    }
    if(c!=0){
        now_time = (double)std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count()/1000000.0-start_time;
        double predic = BuffTracker.color=="red"? -(a*ceres::sin(offset_time*3/2)*ceres::sin(b*now_time+c-offset_time*3/2/2)+ d * offset_time*3/2)
                :a*ceres::sin(offset_time*3/2)*ceres::sin(b*now_time+c-offset_time*3/2/2)+ d * offset_time*3/2;
        predic = predic * 180 / CV_PI;
        BuffTracker.preTheta=BuffTracker.targetTheta+predic;
    }
}

void Tracker::car_init(){
#ifndef USE_MSG
    selfYaw=0.1;selfPitch=0.1;
#endif
    selfYaw=rMsg.robotYaw;selfPitch=rMsg.robotPitch;
    CarTracker.t_yaw = selfYaw-track_Detector->yaw;
    CarTracker.t_pitch = selfPitch<0?selfPitch+track_Detector->pitch+180:selfPitch+track_Detector->pitch-180;
    if(CarTracker.t_yaw>360) CarTracker.t_yaw-=360;
    if(CarTracker.t_yaw<0) CarTracker.t_yaw+=360;
    CarTracker.dis = track_Detector->Target_dis*cos(CarTracker.t_pitch/180*CV_PI);
    CarTracker.pos(0)=CarTracker.dis*cos(CarTracker.t_yaw/180*CV_PI);
    CarTracker.pos(1)=CarTracker.dis*sin(CarTracker.t_yaw/180*CV_PI);

    double delta_x,delta_y;
    delta_x = abs(CarTracker.pos(0)-CarTracker.last_x);
    delta_y = abs(CarTracker.pos(1)-CarTracker.last_y);
    if(delta_x > 10 || delta_y > 10){
        CarTracker.switched=true;
        ekf_filter.x(0)=CarTracker.pos(0);
        ekf_filter.x(1)=CarTracker.pos(1);
    }
    else{
        CarTracker.switched=false;
    }
    CarTracker.last_x = CarTracker.pos(0);
    CarTracker.last_y = CarTracker.pos(1);
}

void Tracker::car_reFind() {
    double dis = sqrt(pow(CarTracker.predict(0),2)+pow(CarTracker.predict(1),2));
    if((CarTracker.predict(1)/CarTracker.dis)>=1){CarTracker.pre_yaw=90;}
    else{CarTracker.pre_yaw=asin(CarTracker.predict(1)/CarTracker.dis)*180/CV_PI;}
    if(CarTracker.predict(0)<0) CarTracker.pre_yaw=180-CarTracker.pre_yaw;
    if(CarTracker.predict(0)>0 && CarTracker.predict(1)<0) CarTracker.pre_yaw=CarTracker.pre_yaw+360;
    CarTracker.pre_yaw = -(CarTracker.pre_yaw-selfYaw);

    if(CarTracker.pre_yaw> 360) CarTracker.pre_yaw-=360;
    if(CarTracker.pre_yaw<-360) CarTracker.pre_yaw+=360;

    if(CarTracker.pre_yaw> 180) CarTracker.pre_yaw-=360;
    if(CarTracker.pre_yaw<-180) CarTracker.pre_yaw+=360;
    CarTracker.pre_yaw=CarTracker.pre_yaw>90?180-CarTracker.pre_yaw:CarTracker.pre_yaw;
    CarTracker.pre_yaw=CarTracker.pre_yaw<-90?-180-CarTracker.pre_yaw:CarTracker.pre_yaw;
    CarTracker.pre_pitch = track_Detector->offset_pitch;
    pre_yaw=CarTracker.pre_yaw;pre_pitch=CarTracker.pre_pitch;
    vMsg.aimYaw=pre_yaw;vMsg.aimPitch=pre_pitch;
    double fx = cameraMatrix.at<double>(0, 0);
    double fy = cameraMatrix.at<double>(1, 1);
    double cx = cameraMatrix.at<double>(0, 2);
    double cy = cameraMatrix.at<double>(1, 2);
    double X = dis * sin(CarTracker.pre_yaw/180*CV_PI);
    double Y = dis * sin(CarTracker.pre_pitch/180*CV_PI);
    double org_X = track_Detector->Target_dis * sin(track_Detector->yaw/180*CV_PI);
    double org_Y = track_Detector->Target_dis * sin(track_Detector->offset_pitch/180*CV_PI);
    // 使用相机内参将相对坐标转换为像素坐标
    double pixelX = (fx * X / dis) + cx;
    double pixelY = (fy * -Y / dis) + cy;
    double pixel_orgX = (fx * org_X / track_Detector->Target_dis) + cx;
    double pixel_orgY = (fy * -org_Y / track_Detector->Target_dis) + cy;
    target.x=(float)pixelX;target.y=(float)pixelY;
    org.x=(float)pixel_orgX;org.y=(float)pixel_orgY;
}

void Tracker::draw(){
    if(BuffTracker.radius>0){
        cv::circle(track_Detector->show(cv::Rect(track_Detector->show.cols/2-640,track_Detector->show.rows/2-512,1280,1024)), BuffTracker.R_position, (int)BuffTracker.radius,cv::Scalar(255, 255 ,255), 1);
        cv::circle(track_Detector->show(cv::Rect(track_Detector->show.cols/2-640,track_Detector->show.rows/2-512,1280,1024)), BuffTracker.Target_position, 10,cv::Scalar(0, 255 ,0), -1);
        cv::putText(track_Detector->show,cv::format("predicTime:%2f",offset_time),cv::Point2i(50, 250),cv::FONT_HERSHEY_SIMPLEX,
                    1,cv::Scalar(0,255,255),2);
        cv::putText(track_Detector->show,cv::format("Theta:%2f",BuffTracker.targetTheta),cv::Point2i(50, 300),cv::FONT_HERSHEY_SIMPLEX,
                    1,cv::Scalar(0,255,255),2);
        cv::putText(track_Detector->show,cv::format("preTheta:%2f",BuffTracker.preTheta),cv::Point2i(50, 350),cv::FONT_HERSHEY_SIMPLEX,
                    1,cv::Scalar(0,255,255),2);
        cv::putText(track_Detector->show,cv::format("a:%2f b:%2f c:%2f d:%2f",a,b,c,d),cv::Point2i(50, 400),cv::FONT_HERSHEY_SIMPLEX,
                    1,cv::Scalar(0,255,255),2);
        cv::arrowedLine(track_Detector->show(cv::Rect(track_Detector->show.cols/2-640,track_Detector->show.rows/2-512,1280,1024)),BuffTracker.R_position,BuffTracker.Target_position,cv::Scalar(0,255,0));
    }
    if(CarTracker.dis != 0){
        cv::circle(track_Detector->show, target, 10,cv::Scalar(255, 0,255), 3);
        cv::circle(track_Detector->show, org, 5,cv::Scalar(100, 0,255), 3);
        cv::putText(track_Detector->show,cv::format("t_yaw:%2f t_pitch:%2f",CarTracker.t_yaw,CarTracker.t_pitch),cv::Point2i(50, 250),cv::FONT_HERSHEY_SIMPLEX,
                    1,cv::Scalar(0,255,255),2);
        cv::putText(track_Detector->show,cv::format("pre_yaw:%2f pre_pitch:%2f",CarTracker.pre_yaw,CarTracker.pre_pitch),cv::Point2i(50, 500),cv::FONT_HERSHEY_SIMPLEX,
                    1,cv::Scalar(0,255,255),2);
        cv::putText(track_Detector->show,cv::format("pre_x:%2f pre_y:%2f",CarTracker.predict(0),CarTracker.predict(1)),cv::Point2i(50, 450),cv::FONT_HERSHEY_SIMPLEX,
                    1,cv::Scalar(0,255,255),2);
        cv::putText(track_Detector->show,cv::format("delta_x:%2f delta_z:%2f",CarTracker.predict(0)-track_Detector->Target_tvec.at<double>(0,0),CarTracker.predict(1)-track_Detector->Target_tvec.at<double>(0,2)),cv::Point2i(50, 400),cv::FONT_HERSHEY_SIMPLEX,
                    1,cv::Scalar(0,255,255),2);
        cv::putText(track_Detector->show,cv::format("v_x:%2f v_y:%2f",ekf_filter.x(2),ekf_filter.x(3)),cv::Point2i(50, 300),cv::FONT_HERSHEY_SIMPLEX,
                    1,cv::Scalar(0,255,255),2);
    }
    if(CarTracker.switched){
        cv::putText(track_Detector->show,"Switched",cv::Point2i(50, 350),cv::FONT_HERSHEY_SIMPLEX,
                    1,cv::Scalar(0,255,255),2);
    }
}


Tracker::Tracker(class Detector &Detector){
    Tracker::track_Detector = &Detector;
    disFilter.Size=3;
    cv::Mat(3, 3, CV_64FC1, const_cast<double *>(cameraMatrix_2.data())).copyTo(cameraMatrix);
    cv::Mat(1, 5, CV_64FC1, const_cast<double *>(distCoeffs_2.data())).copyTo(distCoeffs);

    problem.AddParameterBlock(&a,1);
    problem.AddParameterBlock(&b,1);
    problem.AddParameterBlock(&d,1);

    problem.SetParameterLowerBound(&a, 0, 0.5);
    problem.SetParameterUpperBound(&a, 0, 1.5);

    problem.SetParameterLowerBound(&b, 0, 1.5);
    problem.SetParameterUpperBound(&b, 0, 2.5);

    problem.SetParameterLowerBound(&d, 0, 0.7);
    problem.SetParameterUpperBound(&d, 0, 1.5);

    options.logging_type = ceres::SILENT;
}


Tracker::~Tracker() = default;