#include <iostream>
#include <vector>
#include <ceres/ceres.h>
#include <fstream>
#include <sstream>
#include <iomanip> // 添加此行以包含 setprecision

struct SplineResidual {
    SplineResidual(double t, double x, double y) : t_(t), x_(x), y_(y) {}

    template <typename T>
    bool operator()(const T* const coeffs_x, const T* const coeffs_y, T* residual) const {
        // 计算 x 和 y 的预测值
        T x_pred = coeffs_x[0] + coeffs_x[1] * T(t_) + coeffs_x[2] * T(t_) * T(t_) + coeffs_x[3] * T(t_) * T(t_) * T(t_);
        T y_pred = coeffs_y[0] + coeffs_y[1] * T(t_) + coeffs_y[2] * T(t_) * T(t_) + coeffs_y[3] * T(t_) * T(t_) * T(t_);
        
        // 残差
        residual[0] = T(x_) - x_pred; // x方向残差
        residual[1] = T(y_) - y_pred; // y方向残差
        return true;
    }

private:
    const double t_; // 参数 t
    const double x_; // 真实的 x 值
    const double y_; // 真实的 y 值
};

// 读取数据函数
bool ReadData(const std::string& filename, 
              std::vector<double>& t_data, 
              std::vector<double>& x_data, 
              std::vector<double>& y_data) 
{
    std::ifstream infile(filename);
    if (!infile.is_open()) {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return false;
    }

    std::string line;
    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        double t, x, y;
        if (iss >> t >> x >> y) {
            t_data.push_back(t);
            x_data.push_back(x);
            y_data.push_back(y);
        }
    }

    infile.close();
    return true;
}

// 写入结果函数
// 写入结果函数
void WriteResults(const std::string& results_filename, 
                  const std::string& points_filename, 
                  const double* coeffs_x, 
                  const double* coeffs_y, 
                  const std::vector<double>& t_data) {
    // 写入拟合系数
    std::ofstream results_file(results_filename);
    if (!results_file.is_open()) {
        std::cerr << "无法打开文件: " << results_filename << std::endl;
        return;
    }

    results_file << "拟合得到的 x(t) 三次多项式系数: " << std::endl;
    results_file << "a_x: " << coeffs_x[0] << ", b_x: " << coeffs_x[1] 
                 << ", c_x: " << coeffs_x[2] << ", d_x: " << coeffs_x[3] << std::endl;

    results_file << "拟合得到的 y(t) 三次多项式系数: " << std::endl;
    results_file << "a_y: " << coeffs_y[0] << ", b_y: " << coeffs_y[1] 
                 << ", c_y: " << coeffs_y[2] << ", d_y: " << coeffs_y[3] << std::endl;

    results_file.close();

    // 写入拟合后的曲线点坐标
    std::ofstream points_file(points_filename);
    if (!points_file.is_open()) {
        std::cerr << "无法打开文件: " << points_filename << std::endl;
        return;
    }

    // 计算并写入拟合后的坐标点
    for (const auto& t : t_data) {
        // 计算 x(t) 和 y(t)
        double x_pred = coeffs_x[0] + coeffs_x[1] * t + coeffs_x[2] * t * t + coeffs_x[3] * t * t * t;
        double y_pred = coeffs_y[0] + coeffs_y[1] * t + coeffs_y[2] * t * t + coeffs_y[3] * t * t * t;
        
        // 写入到文件，保留6位小数
        points_file << std::fixed << std::setprecision(6) << t << " " 
                    << x_pred << " " << y_pred << std::endl;
    }

    points_file.close();
}


int main() {
    // 输入文件名
    std::string input_filename = "../data/example/input.txt";
    // 输出文件名
    std::string output_filename = "../data/example/log.txt";
    std::string curve_points_filename = "../data/example/output.txt";

    // 数据容器
    std::vector<double> t_data, x_data, y_data;

    // 读取数据
    if (!ReadData(input_filename, t_data, x_data, y_data)) {
        return 1; // 如果读取失败，则退出
    }

    // 存储多项式系数
    double coeffs_x[4] = {0.0, 0.0, 0.0, 0.0}; // x(t) 的系数
    double coeffs_y[4] = {0.0, 0.0, 0.0, 0.0}; // y(t) 的系数

    // 创建优化问题
    ceres::Problem problem;

    // 为每一个数据点添加残差
    for (size_t i = 0; i < t_data.size(); ++i) {
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<SplineResidual, 2, 4, 4>(
                new SplineResidual(t_data[i], x_data[i], y_data[i])),
            nullptr,
            coeffs_x,
            coeffs_y);
    }

    // 设置求解器选项
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    // 求解
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // 写入结果
    WriteResults(output_filename, curve_points_filename, coeffs_x, coeffs_y, t_data);


    return 0;
}
