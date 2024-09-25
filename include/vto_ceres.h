#ifndef VTO_CERES_H
#define VTO_CERES_H

#include <iostream>
#include <vector>
#include <ceres/ceres.h>

#include "types.h"

// 单帧观测残差函数
struct PointResidual {
    PointResidual(ObservedVehicleState2D observed_state) : observed_state_(observed_state) {}

    //括号重载函数参数包含优化变量和残差
    template <typename T>
    bool operator()(const T* const opti_param, T* residual) const {
        //opti_param[0]为x坐标，opti_param[1]为y坐标
        residual[0] = opti_param[0] - observed_state_.x;
        residual[1] = opti_param[1] - observed_state_.y;
        return true;
    }
private:
    ObservedVehicleState2D observed_state_;
};

// 动力学残差函数
struct DynamicResidual {
    DynamicResidual(ObservedVehicleState2D observed_state) : observed_state_(observed_state) {}

    // 括号重载函数参数包含两个帧的优化变量和残差
    template <typename T>
    bool operator()(const T* opti_param_01, 
                    const T* opti_param_02,
                    const T* opti_param_03, 
                    T* residual) const {
        T angle;
        T delta_x12, delta_y12;
        T delta_x23, delta_y23;
        T delta_x13, delta_y13;

        delta_x12 = (opti_param_02[0] - opti_param_01[0]);
        delta_y12 = (opti_param_02[1] - opti_param_01[1]);
        delta_x23 = (opti_param_03[0] - opti_param_02[0]);
        delta_y23 = (opti_param_03[1] - opti_param_02[1]);
        delta_x13 = (opti_param_03[0] - opti_param_01[0]);
        delta_y13 = (opti_param_03[1] - opti_param_01[1]);
        angle = R2D(atan2(delta_y13, delta_x13));
        // 计算残差
        residual[0] = 100.0 * ((opti_param_03[0] - opti_param_02[0]) - (opti_param_02[0] - opti_param_01[0]));
        residual[1] = 100.0 * ((opti_param_03[1] - opti_param_02[1]) - (opti_param_02[1] - opti_param_01[1])); 
        residual[2] = (observed_state_.yaw - 
                      R2D(atan2(opti_param_03[1] - opti_param_01[1], 
                                opti_param_03[0] - opti_param_01[0])));
        if (residual[2] > 180 || residual[2] < -180)
        {
            residual[2] = residual[2] - 360.0 * floor(residual[2] / 360.0 + 0.5);
        }
        // residual[2] = 10.0 * residual[2];
        
        printf("observed_state: %.3lf, %.3lf, %.3lf, angle: %.3lf, residual: %.3lf\n",
               observed_state_.x,
               observed_state_.y,
               observed_state_.yaw,
               angle, 
               residual[2]);
        return true;
    }
private:
    ObservedVehicleState2D observed_state_;
};

#endif // VTO_CERES_H