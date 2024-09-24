#include <iostream>
#include <vector>
#include <ceres/ceres.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <sstream>
#include <iomanip> // 添加此行以包含 setprecision

#include "types.h"
#include "vto_ceres.h"

/** @brief 结构体定义 */

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
                  const std::vector<double> x_points, 
                  const std::vector<double> y_points,
                  const ceres::Solver::Summary& summary,
                  const std::vector<double>& t_data) {
    // 写入拟合系数
    std::ofstream results_file(results_filename);
    if (!results_file.is_open()) {
        std::cerr << "无法打开文件: " << results_filename << std::endl;
        return;
    }

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
    for (size_t i = 0; i < t_data.size(); ++i)
    {
        // 写入到文件，保留6位小数
        points_file << std::fixed << std::setprecision(6) 
                    << t_data[i] << " " 
                    << x_points[i] << " " 
                    << y_points[i] << std::endl;
    }

    points_file.close();
}


int main() {
    // 配置参数
    Config config;
    size_t num_observations = 0; // 观测点个数
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
    num_observations = t_data.size();
    // 存储多项式系数
    double coeffs_x[4] = {0.0, 0.0, 0.0, 0.0}; // x(t) 的系数
    double coeffs_y[4] = {0.0, 0.0, 0.0, 0.0}; // y(t) 的系数
    std::vector<double> x_points, y_points; // 存储拟合后的曲线点坐标
    x_points = x_data;  //初值设置为观测数据
    y_points = y_data;
    // 创建优化问题
    ceres::Problem problem;

    ObservedVehicleState2D obs_vehicle_state;
    double* parameters = new double[t_data.size() * 2]; // 优化参数
    // 为每一个数据点添加残差
    for (size_t i = 0; i < t_data.size(); ++i) 
    {
        // 设置优化参数初值
        parameters[i * 2] = x_data[i];
        parameters[i * 2 + 1] = y_data[i];

        // 添加单帧观测残差
        obs_vehicle_state.x = x_data[i];
        obs_vehicle_state.y = y_data[i];
        obs_vehicle_state.yaw = 0.0;

        // 添加单帧观测残差
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<PointResidual, 2, 2>(
                new PointResidual(obs_vehicle_state)),
            nullptr,
            &parameters[i * 2]);

        // 添加帧间观测残差
        if (i > 0 && i < t_data.size() - 1)
        {
            problem.AddResidualBlock(
                new ceres::AutoDiffCostFunction<DynamicResidual, 2, 2, 2, 2>(
                    new DynamicResidual()),
                nullptr,
                &parameters[(i - 1) * 2], 
                &parameters[i * 2], 
                &parameters[(i + 1) * 2]);
        }
    }

    // 设置求解器选项
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 200;
    options.minimizer_progress_to_stdout = true;

    // 求解
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // 写入结果
    for (size_t i = 0; i < t_data.size(); i++)
    {
        x_points[i] = parameters[i * 2];
        y_points[i] = parameters[i * 2 + 1];
    }
    delete [] parameters;

    WriteResults(config.output_filename, 
                 config.curve_points_filename, 
                 x_points, 
                 y_points, 
                 summary, 
                 t_data);

    return 0;
}
