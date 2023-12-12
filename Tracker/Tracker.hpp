
#ifndef RM_CPP_TRACKER_HPP
#define RM_CPP_TRACKER_HPP

#include <opencv2/opencv.hpp>
#include "vector"
#include "../Detector/Detector.hpp"
#include "EKF.hpp"
#include "avgFilter.hpp"
#include <ceres/ceres.h>
#include "../visionSerial/include/serial.hpp"
#define USE_MSG //使用串口信息

const std::array<double,3> xyzFix = {0,-40,0}; // mm
const std::array<double,1> PitchFix = {1};

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


const std::array<double,9> cameraMatrix_old{1016.808471,    0,            645.1676831,
                                            0,              1016.554536,  508.7050056,
                                            0,              0,            1};
const std::array<double,5> distCoeffs_old{-0.392676910576646,0.203662234,0,0,-0.0395421280802953};


const std::array<double,9> cameraMatrix_1{1869.074197, 0,          664.6998242,
                                          0,           1867.898354, 518.0525069,
                                          0,           0,          1};

const std::array<double,5> distCoeffs_1{-0.163116073466183,0.255155351,0,0,0};

const std::array<double,9> cameraMatrix_2{1870.83826, 0,          654.97005,
                                        0,           1869.7689, 528.98147,
                                        0,           0,          1};

const std::array<double,5> distCoeffs_2{-0.15885,0.205852,0,0,0};

struct buffTracker{
    std::string color;
    bool isSmall = false;
    double radius = 0;//半径
    double spd = 0;//角速度
    double targetTheta = 0;//目标角度
    double lastTheta = 0;//上一帧角度
    double preTheta;
    cv::Point2f R_position;
    cv::Point2d Target_position;

};

struct carTracker{
    double t_yaw = 0;
    double t_pitch = 0;
    double dis = 0;
    Eigen::Vector2d pos;
    Eigen::Vector2d predict;
    double pre_yaw,pre_pitch;
    bool switched=false;
    double last_x=0,last_y=0;
};

struct TrigResidual {
    TrigResidual(double x, double y, double z)
            : t_(x), dr_(y), dt_(z){}
    template <typename T>
    bool operator()(const T* const a, const T* const b, const T* const c, const T* const d, T* residual) const {
        residual[0] = a[0] * ceres::sin(T(dt_)/2.0) * ceres::sin(b[0] * T(t_) + c[0] - T(dt_)/2.0) + d[0]*T(dt_) - T(dr_);
        return true;
    }

private:
    const double t_;
    const double dr_;
    const double dt_;
};


class Tracker {
private:
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    Detector *track_Detector;
    class avgFilter disFilter;
public:
    double pre_yaw,pre_pitch;
    double Gravity = 9.8;
    double air_k = 0.033;
    double speed = 20;
    double pre_k=10;
    double offset_time{};
    explicit Tracker(class Detector &Detector);
    buffTracker BuffTracker;
    carTracker CarTracker;
    cv::Point2f target,org;
    EKF ekf_filter;
    visionMsg* vMsg=new visionMsg;
    robotMsg* rMsg=new robotMsg;
    //追踪
    bool track();

    //PnP测距
    bool pnpSolve();

    //枪口抬高
    bool offset();
    ~Tracker();

    void trackTarget();

    void buff_init();
    void Big_buff_track();

    double last_time  =-1;
    double start_time = 0;
    // 离散数据点
    int raw_points = 25;
    std::vector<double> t;
    std::vector<double> dt;
    std::vector<double> dr;

    // 初始化参数值
    double a = 0.9125;
    double b = 1.942;
    double c = 0;
    double d = 1.1775;

    // 创建Ceres问题
    ceres::Problem problem;
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;

    void car_init();
    float selfYaw{},selfPitch{};
    void car_reFind();

    void draw();

};


#endif //RM_CPP_TRACKER_HPP
