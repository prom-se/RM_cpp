#include "EKF.hpp"
using namespace Eigen;

void EKF::init() {
    P.resize(4,4);H.resize(2,4);
    Q.resize(4,4);R.resize(2,2);
    A.resize(4,4);
    // 初始化状态、协方差和模型参数
    x << 0, 0, 1, 1;  // 初始状态 [x位置, y位置, x速度, y速度]
    P = Matrix4d::Identity() * 0.1;  // 初始协方差矩阵
    A << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;
    H << 1, 0, 0, 0,
            0, 1, 0, 0;
    Q = Matrix4d::Identity() * 0.05;  // 过程噪声
    R = Matrix2d::Identity() * 0.1;   // 测量噪声
}

EKF::EKF() {
    EKF::init();
}

void EKF::predict() {
    x = A * x;
    P = A * P * A.transpose() + Q;
}

void EKF::update(const Vector2d &measurement) {
    Vector2d y = measurement - H * x;
    Matrix2d S = H * P * H.transpose() + R;
    Matrix<double, 4, 2> K = P * H.transpose() * S.inverse();

    x = x + K * y;
    P = (MatrixXd::Identity(4, 4) - K * H) * P;
}

