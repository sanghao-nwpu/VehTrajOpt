#ifndef VTO_CERES_H
#define VTO_CERES_H

#include <iostream>
#include <vector>
#include <ceres/ceres.h>

#include "types.h"

// 单帧位置观测残差函数
struct PointResidual {
    PointResidual(ObservedVehicleState2D observed_state, double weight) 
        : observed_state_(observed_state), weight_(weight) {}

    //括号重载函数参数包含优化变量和残差
    template <typename T>
    bool operator()(const T* const p, T* residual) const {
        //opti_param[0]为x坐标，opti_param[1]为y坐标
        residual[0] = weight_ * (p[0] - observed_state_.x);
        residual[1] = weight_ * (p[1] - observed_state_.y);
        return true;
    }
private:
    ObservedVehicleState2D observed_state_;
    double weight_;
};

// 平滑性残差函数
struct SmoothnessResidual {
    SmoothnessResidual(double weight) : weight_(weight) {}

    template <typename T>
    bool operator()(const T* const p1, const T* const p2, const T* const p3, T* residual) const {
        // 计算二阶差异
        residual[0] = weight_ * (p3[0] - 2.0 * p2[0] + p1[0]); // x方向的平滑性
        residual[1] = weight_ * (p3[1] - 2.0 * p2[1] + p1[1]); // y方向的平滑性
        return true;
    }

    double weight_;
};

// 动力学残差函数
struct DynamicResidual {
    DynamicResidual(ObservedVehicleState2D observed_state, double weight) 
        : observed_state_(observed_state), weight_(weight) {}

    // 括号重载函数参数包含两个帧的优化变量和残差
    template <typename T>
    bool operator()(const T *p1,
                    const T *p2,
                    T *residual) const
    {
        T angle, vx, vy;

        angle = R2D(atan2(p2[1] - p1[1], p2[0] - p1[0]));
        vx = (p2[0] - p1[0]) / observed_state_.delta_t;
        vy = (p2[1] - p1[1]) / observed_state_.delta_t;
        // 计算残差
        residual[0] = weight_ * (observed_state_.vx - vx);
        residual[1] = weight_ * (observed_state_.vy - vy);
        residual[2] = weight_ * (observed_state_.yaw - angle);
        if (residual[2] > 180 || residual[2] < -180)
        {
            residual[2] = residual[2] - 360.0 * floor(residual[2] / 360.0 + 0.5);
        }

        return true;
    }

private:
    ObservedVehicleState2D observed_state_;
    double weight_;
};

#endif // VTO_CERES_H