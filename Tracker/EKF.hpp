
#ifndef RM_CPP_EKF_HPP
#define RM_CPP_EKF_HPP

#include <Eigen/Dense>
#include <thread>

using namespace Eigen;
class EKF {
    // 定义系统动态模型和测量模型
    MatrixXd A;// 状态转移矩阵
    MatrixXd H;// 测量矩阵
    MatrixXd Q;// 过程噪声协方差矩阵
    MatrixXd R;// 测量噪声协方差矩阵
    MatrixXd P;// 协方差矩阵

    double delta_s={};//更新时间差
    long last_time={};
    long now_time={};
public:
    Vector4d x;// 初始状态估计
    bool flag=true;
    Vector2d measurements={0,0};
    Vector2d pre_position={0,0};
    double pre_time;
    void init();
    void predict();
    void update(const Vector2d& measurement);
    void dt(double k);
    [[noreturn]] void track_thread();

    EKF();
    ~EKF() = default;
};


#endif //RM_CPP_EKF_HPP