#include "NhcCostFunction.h"
#include <cmath>

template <typename T>
bool NhcCostFunction::operator()(const T* const coeffs, T* residual) const {
    // 计算位置、速度、加速度和Snap
    T t2 = t_ * t_;
    T t3 = t2 * t_;
    T t4 = t3 * t_;
    T t5 = t4 * t_;

    T x = coeffs[0] + coeffs[1] * t_ + coeffs[2] * t2 + coeffs[3] * t3 + coeffs[4] * t4 + coeffs[5] * t5;
    T vx = coeffs[1] + 2 * coeffs[2] * t_ + 3 * coeffs[3] * t2 + 4 * coeffs[4] * t3 + 5 * coeffs[5] * t4;
    T ax = 2 * coeffs[2] + 6 * coeffs[3] * t_ + 12 * coeffs[4] * t2 + 20 * coeffs[5] * t3;
    T jx = 6 * coeffs[3] + 24 * coeffs[4] * t_ + 60 * coeffs[5] * t2;
    T sx = 24 * coeffs[4] + 120 * coeffs[5] * t_;

    // 计算残差
    residual[0] = x - T(pos_.x());
    residual[1] = vx - T(vel_.x());
    residual[2] = ax - T(acc_.x());
    residual[3] = sx; // Snap

    // 非完整约束：假设车辆的速度和转向角度之间的关系
    T theta = atan2(coeffs[1], coeffs[0]); // 假设车辆的方向角
    T v = sqrt(coeffs[1] * coeffs[1] + coeffs[0] * coeffs[0]); // 车辆速度
    T omega = v / T(1.0); // 假设车辆的转向角速度
    residual[4] = omega - T(1.0); // 假设转向角速度为1 rad/s

    return true;
}
