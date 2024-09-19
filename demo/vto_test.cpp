#include <iostream>
#include <vector>
#include <ceres/ceres.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <sstream>
#include <iomanip> // 添加此行以包含 setprecision


struct Config {
    std::string input_filename;
    std::string output_filename;
    std::string curve_points_filename;
    bool ceres_auto_diff_flag;
};

bool ReadConfigYaml(const std::string& filename, Config& config) {
    YAML::Node config_node = YAML::LoadFile(filename);
    
    config.input_filename = config_node["input_filename"].as<std::string>();
    config.output_filename = config_node["output_filename"].as<std::string>();
    config.curve_points_filename = config_node["curve_points_filename"].as<std::string>();
    config.ceres_auto_diff_flag = config_node["ceres_auto_diff_flag"].as<bool>();
    return true;
}

bool ReadConfigTxt(const std::string& filename, Config& config) {
    std::ifstream infile(filename);
    if (!infile.is_open()) {
        std::cerr << "无法打开配置文件: " << filename << std::endl;
        return false;
    }

    std::string line;
    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        std::string key;
        if (std::getline(iss, key, '=')) {
            std::string value;
            if (std::getline(iss, value)) {
                if (key == "input_filename") {
                    config.input_filename = value;
                } else if (key == "output_filename") {
                    config.output_filename = value;
                } else if (key == "curve_points_filename") {
                    config.curve_points_filename = value;
                }
            }
        }
    }

    infile.close();
    return true;
}


// 定义解析求导的代价函数
class SplineAnalyticResidual : public ceres::SizedCostFunction<2, 4, 4> {
public:
    SplineAnalyticResidual(double t, double x, double y)
        : t_(t), x_(x), y_(y) {}

    virtual ~SplineAnalyticResidual() {}

    // 计算残差和雅可比
    virtual bool Evaluate(double const* const* parameters, 
                          double* residuals, 
                          double** jacobians) const 
    {
        // 提取参数
        const double* coeffs_x = parameters[0];
        const double* coeffs_y = parameters[1];

        // 计算残差
        double spline_x = CalculateSpline(coeffs_x, t_);
        double spline_y = CalculateSpline(coeffs_y, t_);
        
        residuals[0] = x_ - spline_x;
        residuals[1] = y_ - spline_y;

        // 计算雅可比
        if (jacobians != nullptr) {
            if (jacobians[0] != nullptr) {
                // 计算对 coeffs_x 的偏导数
                ComputeJacobianX(coeffs_x, t_, jacobians[0]);
            }
            if (jacobians[1] != nullptr) {
                // 计算对 coeffs_y 的偏导数
                ComputeJacobianY(coeffs_y, t_, jacobians[1]);
            }
        }

        return true;
    }

private:
    double t_;
    double x_;
    double y_;

    double CalculateSpline(const double* coeffs, double t) const {
        // 根据 spline 公式计算值
        // ... 实现您的样条计算逻辑
        double spline = 0.0;
        spline = coeffs[0] + coeffs[1] * t + coeffs[2] * t * t + coeffs[3] * t * t * t;
        return spline;
    }

    void ComputeJacobianX(const double* coeffs, double t, double* jacobian) const {
        // 计算对 coeffs_x 的雅可比
        int num_coeffs = 4;
        int num_residuals = 2;
        // jacobian = new double[num_coeffs * num_residuals];
        jacobian[0 * num_coeffs + 0] = -1.0; // 第一个残差对第一个参数的偏导数
        jacobian[0 * num_coeffs + 1] = -t; // 第一个残差对第二个参数的偏导数
        jacobian[0 * num_coeffs + 2] = -t * t; // 第一个残差对第三个参数的偏导数
        jacobian[0 * num_coeffs + 3] = -t * t * t; // 第一个残差对第四个参数的偏导数
        jacobian[1 * num_coeffs + 0] = 0.0; // 第二个残差对第一个参数的偏导数
        jacobian[1 * num_coeffs + 1] = 0.0; // 第二个残差对第二个参数的偏导数
        jacobian[1 * num_coeffs + 2] = 0.0; // 第二个残差对第三个参数的偏导数
        jacobian[1 * num_coeffs + 3] = 0.0; // 第二个残差对第四个参数的偏导数    
    }

    void ComputeJacobianY(const double* coeffs, double t, double* jacobian) const {
        // 计算对 coeffs_y 的雅可比
        // ... 实现您的雅可比计算逻辑
        int num_coeffs = 4;
        int num_residuals = 2;
        // jacobian = new double[num_coeffs * num_residuals];
        jacobian[0 * num_coeffs + 0] = 0.0; // 第一个残差对第一个参数的偏导数
        jacobian[0 * num_coeffs + 1] = 0.0; // 第一个残差对第二个参数的偏导数
        jacobian[0 * num_coeffs + 2] = 0.0; // 第一个残差对第三个参数的偏导数
        jacobian[0 * num_coeffs + 3] = 0.0; // 第一个残差对第四个参数的偏导数
        jacobian[1 * num_coeffs + 0] = -1.0; // 第二个残差对第一个参数的偏导数
        jacobian[1 * num_coeffs + 1] = -t; // 第二个残差对第二个参数的偏导数
        jacobian[1 * num_coeffs + 2] = -t * t; // 第二个残差对第三个参数的偏导数
        jacobian[1 * num_coeffs + 3] = -t * t * t; // 第二个残差对第四个参数的偏导数
    }
};

// 定义自动求导的代价函数
struct SplineAutoDiffResidual {
    SplineAutoDiffResidual(double t, double x, double y) : t_(t), x_(x), y_(y) {}

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
    const double x_; // 观测的 x 值
    const double y_; // 观测的 y 值
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
    std::vector<double> data;
    double value;
    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        while (iss >> value)
        {
            data.push_back(value);
        }
        if (data.size() >= 3)
        {
            t_data.push_back(data[0]);
            x_data.push_back(data[2]);
            y_data.push_back(data[3]);
        }
        data.clear();
    }

    infile.close();
    return true;
}

// 写入结果函数
void WriteResults(const std::string& results_filename, 
                  const std::string& points_filename, 
                  const double* coeffs_x, 
                  const double* coeffs_y, 
                  const ceres::Solver::Summary& summary,
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
    results_file << summary.FullReport() << std::endl;
    printf("%s\n", summary.FullReport().c_str());
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
    // 配置参数
    Config config;

    // 读取配置文件
    std::string config_yaml_filename = "../data/config.yaml"; // 配置文件路径
    if (!ReadConfigYaml(config_yaml_filename, config)) {
        return 1; // 如果读取失败，则退出
    }

    // 数据容器
    std::vector<double> t_data, x_data, y_data;

    // 读取数据
    if (!ReadData(config.input_filename, t_data, x_data, y_data)) {
        return 1; // 如果读取失败，则退出
    }

    // 存储多项式系数
    double coeffs_x[4] = {0.0, 0.0, 0.0, 0.0}; // x(t) 的系数
    double coeffs_y[4] = {0.0, 0.0, 0.0, 0.0}; // y(t) 的系数

    // 创建优化问题
    ceres::Problem problem;

    // 为每一个数据点添加残差
    if (true == config.ceres_auto_diff_flag)
    {
        for (size_t i = 0; i < t_data.size(); ++i) {
            problem.AddResidualBlock(
                new ceres::AutoDiffCostFunction<SplineAutoDiffResidual, 2, 4, 4>(
                    new SplineAutoDiffResidual(t_data[i], x_data[i], y_data[i])),
                nullptr,
                coeffs_x,
                coeffs_y);
        }
    }
    else
    {
        for (size_t i = 0; i < t_data.size(); ++i) {
            problem.AddResidualBlock(new SplineAnalyticResidual(t_data[i], x_data[i], y_data[i]),
                                    nullptr,
                                    coeffs_x,
                                    coeffs_y);
        }
    }

    // 设置求解器选项
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 100;
    options.minimizer_progress_to_stdout = true;

    // 求解
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // 写入结果
    WriteResults(config.output_filename, 
                 config.curve_points_filename, 
                 coeffs_x, 
                 coeffs_y, 
                 summary, 
                 t_data);


    return 0;
}
