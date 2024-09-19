/**
 * @file main.cpp
 * @brief 轨迹优化示例，使用 Ceres Solver 和 Eigen 库。
 *
 * 该项目通过 Ceres Solver 优化轨迹参数，满足特定约束条件。
 *
 * @author [sanghao]
 * @date [2024.09.12]
 * @version 1.0
 */

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <ceres/ceres.h>

using namespace Eigen;
using namespace std;

// 定义一个简单的代价函数，用于优化轨迹
struct NhcCostFunction {
    NhcCostFunction(double t, const Vector3d& pos, const Vector3d& vel)
        : t_(t), pos_(pos), vel_(vel) {}

    template <typename T>
    bool operator()(const T* const coeffs, T* residuals) const {
        // 计算残差
        residuals[0] = coeffs[0] + 
                       coeffs[1] * t_ + 
                       coeffs[2] * t_ * t_ + 
                       coeffs[3] * t_ * t_ * t_ - T(pos_.x());
        residuals[1] = coeffs[1] + 
                       coeffs[2] * t_ * 2.0 + 
                       coeffs[3] * t_ * t_ * 3.0 - T(vel_.x());

        return true;
    }

private:
    double t_;
    Vector3d pos_, vel_;
};

// 轨迹优化函数
vector<Vector3d> optimizeTrajectory(const vector<Vector3d>& rawTrajectory) {
    vector<Vector3d> optimizedTrajectory = rawTrajectory;

    ceres::Problem problem;

    // 假设我们使用三次多项式来表示轨迹
    int num_coeffs = 4;
    vector<double> coeffs(num_coeffs * optimizedTrajectory.size(), 0.0);

    printf("Add parameter block for coefficients\n");
    printf("  num_coeffs: %d\n", num_coeffs);
    printf("  num_points: %lu\n", optimizedTrajectory.size());
    for (size_t i = 1; i < optimizedTrajectory.size(); ++i) {
        double t = i * 1.0; // 假设时间间隔为1秒
        ceres::CostFunction* cost_function =
            new ceres::AutoDiffCostFunction<NhcCostFunction, 2, 4>(
                new NhcCostFunction(t, optimizedTrajectory[i], Vector3d::Zero()));
        printf("Add residual block for point %lu\n", i);
        printf("  pos: (%f, %f, %f)\n", 
                optimizedTrajectory[i].x(), 
                optimizedTrajectory[i].y(), 
                optimizedTrajectory[i].z());
        problem.AddResidualBlock(cost_function, nullptr, coeffs.data() + i * num_coeffs);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    cout << summary.BriefReport() << endl;

    // 使用优化后的系数重新生成轨迹
    for (size_t i = 0; i < optimizedTrajectory.size(); ++i) {
        double t = i * 1.0;
        double t2 = t * t;
        double t3 = t2 * t;

        optimizedTrajectory[i].x() = coeffs[i * num_coeffs + 0] + 
                                     coeffs[i * num_coeffs + 1] * t + 
                                     coeffs[i * num_coeffs + 2] * t2 + 
                                     coeffs[i * num_coeffs + 3] * t3;
        optimizedTrajectory[i].y() = coeffs[i * num_coeffs + 0] + 
                                     coeffs[i * num_coeffs + 1] * t + 
                                     coeffs[i * num_coeffs + 2] * t2 + 
                                     coeffs[i * num_coeffs + 3] * t3;
        optimizedTrajectory[i].z() = coeffs[i * num_coeffs + 0] + 
                                     coeffs[i * num_coeffs + 1] * t + 
                                     coeffs[i * num_coeffs + 2] * t2 + 
                                     coeffs[i * num_coeffs + 3] * t3;
    }

    return optimizedTrajectory;
}

int main() {
    vector<Vector3d> rawTrajectory = {
        Vector3d(0, 0, 0),
        Vector3d(1, 1, 0),
        Vector3d(2, 1, 0),
        Vector3d(3, 0, 0),
        Vector3d(4, 0, 0),
        Vector3d(5, 0, 0),
        Vector3d(6, 0, 0),
        Vector3d(7, -1, 0),
        Vector3d(8, -1, 0),
        Vector3d(9, 1, 0),
        Vector3d(10, 1, 0)
    };

    vector<Vector3d> optimizedTrajectory = optimizeTrajectory(rawTrajectory);

    for (const auto& point : optimizedTrajectory) {
        cout << "(" << point.x() << ", " << point.y() << ", " << point.z() << ")" << endl;
    }

    return 0;
}
