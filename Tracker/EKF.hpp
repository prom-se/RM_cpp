
#ifndef RM_CPP_EKF_HPP
#define RM_CPP_EKF_HPP

#include <Eigen/Dense>
using namespace Eigen;
class EKF {
    // 定义系统动态模型和测量模型
    MatrixXd A;// 状态转移矩阵
    MatrixXd H;// 测量矩阵
    MatrixXd Q;// 过程噪声协方差矩阵
    MatrixXd R;// 测量噪声协方差矩阵
    MatrixXd P;// 协方差矩阵
public:
    Vector4d x;// 初始状态估计
    void init();
    void predict();
    void update(const Vector2d &measurement);
    EKF();
};


#endif //RM_CPP_EKF_HPP
