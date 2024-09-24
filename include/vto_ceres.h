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
    DynamicResidual() {}

    // 括号重载函数参数包含两个帧的优化变量和残差
    template <typename T>
    bool operator()(const T* opti_param_01, 
                    const T* opti_param_02,
                    const T* opti_param_03, 
                    T* residual) const {
        // 计算残差
        residual[0] = (opti_param_03[0] - opti_param_02[0]) - (opti_param_02[0] - opti_param_01[0]);
        residual[1] = (opti_param_03[1] - opti_param_02[1]) - (opti_param_02[1] - opti_param_01[1]); 
        return true;
    }
};

#endif // VTO_CERES_H