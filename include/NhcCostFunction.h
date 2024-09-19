#ifndef NHCCOSTFUNCTION_H
#define NHCCOSTFUNCTION_H

#include <Eigen/Dense>
#include <ceres/ceres.h>

using namespace Eigen;

struct NhcCostFunction {
    NhcCostFunction(double t, const Vector3d& pos, const Vector3d& vel, const Vector3d& acc)
        : t_(t), pos_(pos), vel_(vel), acc_(acc) {}

    template <typename T>
    bool operator()(const T* const coeffs, T* residual) const;

private:
    double t_;
    Vector3d pos_, vel_, acc_;
};


//第一个参数（5）：表示残差的数量。在这个例子中，残差的数量是 5，通常对应于不同的误差项（例如，位置误差、速度误差、加速度误差等）。
//第二个参数（6）：表示参数的数量。在这个例子中，参数的数量是 6，通常对应于多项式的系数。
class NhcCostFunctionAnalytic : public ceres::SizedCostFunction<5, 6> {
public:
    NhcCostFunctionAnalytic(double t, const Vector3d& observed_point, const Vector3d& observed_velocity, const Vector3d& observed_acceleration)
        : t_(t), observed_point_(observed_point), observed_velocity_(observed_velocity), observed_acceleration_(observed_acceleration) {}

    virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {
        const double* coeffs = parameters[0];

        // 计算多项式值
        double x = coeffs[0] + coeffs[1] * t_ + coeffs[2] * t_ * t_ + coeffs[3] * t_ * t_ * t_ + coeffs[4] * t_ * t_ * t_ * t_ + coeffs[5] * t_ * t_ * t_ * t_ * t_;
        double y = coeffs[0] + coeffs[1] * t_ + coeffs[2] * t_ * t_ + coeffs[3] * t_ * t_ * t_ + coeffs[4] * t_ * t_ * t_ * t_ + coeffs[5] * t_ * t_ * t_ * t_ * t_;
        double z = coeffs[0] + coeffs[1] * t_ + coeffs[2] * t_ * t_ + coeffs[3] * t_ * t_ * t_ + coeffs[4] * t_ * t_ * t_ * t_ + coeffs[5] * t_ * t_ * t_ * t_ * t_;

        // 计算残差
        residuals[0] = x - observed_point_.x();
        residuals[1] = y - observed_point_.y();
        residuals[2] = z - observed_point_.z();
        residuals[3] = 0.0; // 速度残差
        residuals[4] = 0.0; // 加速度残差

        // 如果需要计算雅可比矩阵
        if (jacobians != nullptr && jacobians[0] != nullptr) {
            double* jacobian = jacobians[0];
            // 计算雅可比矩阵
            jacobian[0] = 1.0;
            jacobian[1] = t_;
            jacobian[2] = t_ * t_;
            jacobian[3] = t_ * t_ * t_;
            jacobian[4] = t_ * t_ * t_ * t_;
            jacobian[5] = t_ * t_ * t_ * t_ * t_;
        }

        return true;
    }

private:
    double t_;
    Vector3d observed_point_;
    Vector3d observed_velocity_;
    Vector3d observed_acceleration_;
};


#endif // NHCCOSTFUNCTION_H
