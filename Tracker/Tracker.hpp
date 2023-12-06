
#ifndef RM_CPP_TRACKER_HPP
#define RM_CPP_TRACKER_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include "../Detector/Detector.hpp"
#include "EKF.hpp"
#include "avgFilter.hpp"

#define USE_MSG //使用串口信息

const std::array<double,3> xyzFix = {0,0,0}; // mm
const std::array<double,1> PitchFix = {0};

const std::vector<cv::Point3d> small_Armor = {cv::Point3f(-67.5, 27.5, 0),
                                              cv::Point3f(-67.5, -27.5, 0),
                                              cv::Point3f(67.5, 27.5, 0),
                                              cv::Point3f(67.5, -27.5, 0)};

const std::vector<cv::Point3d> large_Armor = {cv::Point3f(-112.5, 27.5, 0),
                                              cv::Point3f(-112.5, -27.5, 0),
                                              cv::Point3f(112.5, 27.5, 0),
                                              cv::Point3f(112.5, -27.5, 0)};

const std::vector<cv::Point3d> rune_Point = {cv::Point3f(-150, 150, 0),
                                              cv::Point3f(-150, -150, 0),
                                              cv::Point3f(150, 150, 0),
                                              cv::Point3f(150, -150, 0)};

//const std::vector<cv::Point3d> rune_Point = {cv::Point3f(-26, 26, 0),
//                                             cv::Point3f(-26, -26, 0),
//                                             cv::Point3f(26, 26, 0),
//                                             cv::Point3f(26, -26, 0)};


const std::array<double,9> cameraMatrix_old{1016.808471, 0,          645.1676831,
                                          0,           1016.554536, 508.7050056,
                                          0,           0,          1};
const std::array<double,5> distCoeffs_old{-0.392676910576646,0.203662234,0,0,-0.0395421280802953};


const std::array<double,9> cameraMatrix_1{1869.074197, 0,          664.6998242,
                                          0,           1867.898354, 518.0525069,
                                          0,           0,          1};
const std::array<double,5> distCoeffs_1{-0.163116073466183,0.255155351,0,0,0};



const std::array<double,9> cameraMatrix_2{1870.83826, 0,          654.97005,
                                        0,           1869.7689, 528.98147,
                                        0,           0,          1};

const std::array<double,5> distCoeffs_2{-0.15885,0.205852,0,0,0};

struct carTracker{
    double t_yaw = 0;
    double t_pitch = 0;
    double dis = 0;
    Eigen::Vector2d pos;
    Eigen::Vector2d predict;
    double pre_yaw,pre_pitch;
    bool switched=false;
    double last_x,last_y;
};

class Tracker {
private:
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    Detector *track_Detector;
    class avgFilter disFilter;
public:
    double pre_yaw,pre_pitch;
    double Gravity = 9.78;
    double air_k = 0.0282;
    double speed = 20;
    double pre_k=10;
    double offset_time{};
    explicit Tracker(class Detector &Detector);    
    carTracker CarTracker;
    cv::Point2f target,org;
    EKF ekf_filter;
    //追踪
    bool track();

    //PnP测距
    bool pnpSolve();

    //枪口抬高
    bool offset();
    ~Tracker();

    void trackTarget();

    void car_init();
    float selfYaw{},selfPitch{};
    void car_reFind();

    void draw();

};


#endif //RM_CPP_TRACKER_HPP
