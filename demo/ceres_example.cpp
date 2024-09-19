#include <iostream>
#include <ceres/ceres.h>

//创建四个残差块，也就是四个仿函数
struct CostFunctor {
  template <typename T>
  bool operator()(const T* const x, T* residual) const {
    residual[0] = x[0] + 10.0 * x[1];
    residual[1] = sqrt(5.0) * (x[2] - x[3]);
    residual[2] = (x[1] - 2.0 * x[2]) * (x[1] - 2.0 * x[2]);
    residual[3] = sqrt(10.0) * (x[0] - x[3]) * (x[0] - x[3]);
    return true;
  }
};

// 定义解析求导的CostFunction
class AnalyticCostFunction : public ceres::SizedCostFunction<4, 4> {
 public:
  virtual ~AnalyticCostFunction() {}

    /** 
        @brief 计算残差和雅可比矩阵
        @param parameters 待优化参数 1 * 4
        @param residuals  计算出的残差 一维数组，长度为4
        @param jacobians  计算出的雅可比矩阵 1* (4 * 4)
    */
    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const {
    const double* x = parameters[0];

    // 计算残差
    residuals[0] = x[0] + 10 * x[1];  // 这里假设第一个残差为0，根据实际问题修改
    residuals[1] = sqrt(5.0) * (x[2] - x[3]);  // 这里假设第二个残差为0，根据实际问题修改
    residuals[2] = (x[1] - 2.0 * x[2]) * (x[1] - 2.0 * x[2]);
    residuals[3] = sqrt(10.0) * (x[0] - x[3]) * (x[0] - x[3]);

    // 计算雅可比矩阵[i, r * param_size[i] + c]
    // 这里只有一个参数块[4 * 1]，所以雅可比矩阵的行数为1
    if (jacobians != NULL && jacobians[0] != NULL) {
      jacobians[0][0 * 4 + 0] = 1.0;
      jacobians[0][0 * 4 + 1] = 10.0;
      jacobians[0][0 * 4 + 2] = 0.0;
      jacobians[0][0 * 4 + 3] = 0.0;
      jacobians[0][1 * 4 + 0] = 0.0;
      jacobians[0][1 * 4 + 1] = 0.0;
      jacobians[0][1 * 4 + 2] = sqrt(5.0);
      jacobians[0][1 * 4 + 3] = -sqrt(5.0);
      jacobians[0][2 * 4 + 0] = 0.0;
      jacobians[0][2 * 4 + 1] = 2.0 * (x[1] - 2.0 * x[2]);
      jacobians[0][2 * 4 + 2] = -4.0 * (x[1] - 2.0 * x[2]);
      jacobians[0][2 * 4 + 3] = 0.0;
      jacobians[0][3 * 4 + 0] = 2.0 * sqrt(10.0) * (x[0] - x[3]);
      jacobians[0][3 * 4 + 1] = 0.0;
      jacobians[0][3 * 4 + 2] = 0.0;
      jacobians[0][3 * 4 + 3] = -2.0 * sqrt(10.0) * (x[0] - x[3]);
    }

    return true;
  }
};

int main(int argc, char** argv) {
  // 设置初始值
  double x[4] = {3.0, -1.0, 0.0, 1.0};

  // 加入problem
  ceres::Problem problem;
  // 解析求导
  problem.AddResidualBlock(new AnalyticCostFunction(), NULL, x);

  // // 自动求导
  // problem.AddResidualBlock(
  //   new ceres::AutoDiffCostFunction<CostFunctor, 4, 4>(new CostFunctor),
  //     NULL, x);

  // 设置options
  ceres::Solver::Options options;
  options.max_num_iterations = 100;  // 迭代次数
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";
  std::cout << "x = " << x[0] << " " << x[1] << " " << x[2] << " " << x[3] << "\n";
  std::cout << summary.message << "\n";

  return 0;
}
