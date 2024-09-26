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

typedef struct TrackVehicleInfo
{
    double t; // 时间
    int id; // 车辆ID
    double x; // x坐标
    double y; // y坐标
    double yaw; // 航向角
    double vx; // 速度
    double vy; // 速度
    double ax; // 加速度
    double ay; // 加速度
} TrackVehicleInfo;

typedef struct OptimizedVehicleInfo
{
    double t; // 时间
    double x; // x坐标
    double y; // y坐标
    double vx; // 速度
    double vy; // 速度
    double yaw; // 航向角
} OptimizedVehicleInfo;

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
              std::vector<TrackVehicleInfo>& track_vehicle_info_list) 
{
    TrackVehicleInfo track_vehicle_info;
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
            track_vehicle_info.t = data[0] * 1e-3;
            track_vehicle_info.x = data[1];
            track_vehicle_info.y = data[2];
            // track_vehicle_info.z = data[3];
            track_vehicle_info.vx = data[4];
            track_vehicle_info.vy = data[5];
            // track_vehicle_info.vz = data[6];
            track_vehicle_info.yaw = data[7];
            track_vehicle_info_list.push_back(track_vehicle_info);
        }
        data.clear();
    }

    infile.close();
    return true;
}

// 写入结果函数
void WriteResults(const std::string& results_filename, 
                  const std::string& points_filename, 
                  const ceres::Solver::Summary& summary,
                  const std::vector<OptimizedVehicleInfo> &optimized_vehicle_info_list) {
    // 写入拟合系数
    std::ofstream results_file(results_filename);
    if (!results_file.is_open()) {
        std::cerr << "无法打开文件: " << results_filename << std::endl;
        return;
    }

    results_file << summary.FullReport() << std::endl;
    // printf("%s\n", summary.FullReport().c_str());
    results_file.close();

    // 写入拟合后的曲线点坐标
    std::ofstream points_file(points_filename);
    if (!points_file.is_open()) {
        std::cerr << "无法打开文件: " << points_filename << std::endl;
        return;
    }

    // 计算并写入拟合后的坐标点
    for (size_t i = 0; i < optimized_vehicle_info_list.size(); ++i)
    {
        // 写入到文件，保留6位小数
        points_file << std::fixed << std::setprecision(6) 
                    << optimized_vehicle_info_list[i].t << " " 
                    << optimized_vehicle_info_list[i].x << " " 
                    << optimized_vehicle_info_list[i].y << std::endl;
    }

    points_file.close();
}


int main() {
    // 配置参数
    Config config;
    size_t num_observations = 0; // 观测点个数
    OptimizedVehicleInfo optimized_vehicle_info; // 优化结果
    ObservedVehicleState2D obs_vehicle_state;

    // 数据容器
    std::vector<TrackVehicleInfo> track_vehicle_info_list; // 存储观测数据
    std::vector<OptimizedVehicleInfo> optimized_vehicle_info_list; // 存储优化结果

    // 读取配置文件
    std::string config_yaml_filename = "../data/config.yaml"; // 配置文件路径
    if (!ReadConfigYaml(config_yaml_filename, config)) {
        return 1; // 如果读取失败，则退出
    }


    // 读取数据
    if (!ReadData(config.input_filename, track_vehicle_info_list)) {
        return 1; // 如果读取失败，则退出
    }
    // 打印读取数据
    std::cout << "读取数据成功，共有" << track_vehicle_info_list.size() << "个观测点" << std::endl;

    num_observations = track_vehicle_info_list.size();
    
    // 创建优化问题
    ceres::Problem problem;

    // 优化参数
    double* parameters = new double[num_observations * 2]; // 优化参数,  x, y

    // 为每一个数据点添加残差
    for (size_t i = 0; i < num_observations; ++i) 
    {
        // 设置优化参数初值
        parameters[i * 2] = track_vehicle_info_list[i].x;
        parameters[i * 2 + 1] = track_vehicle_info_list[i].y;

        // 添加单帧观测残差
        obs_vehicle_state.t = track_vehicle_info_list[i].t;
        obs_vehicle_state.x = track_vehicle_info_list[i].x;
        obs_vehicle_state.y = track_vehicle_info_list[i].y;
        obs_vehicle_state.vx = track_vehicle_info_list[i].vx;
        obs_vehicle_state.vy = track_vehicle_info_list[i].vy;
        obs_vehicle_state.yaw = track_vehicle_info_list[i].yaw;

        // 添加单帧观测残差
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<PointResidual, 2, 2>(
                new PointResidual(obs_vehicle_state, 1.0)),
            nullptr,
            &parameters[i * 2]);

        // 添加帧间观测残差
        if (i > 0)
        {
            obs_vehicle_state.delta_t = track_vehicle_info_list[i].t - track_vehicle_info_list[i - 1].t;
            problem.AddResidualBlock(
                new ceres::AutoDiffCostFunction<DynamicResidual, 3, 2, 2>(
                    new DynamicResidual(obs_vehicle_state, 1.0)),
                nullptr,
                &parameters[(i - 1) * 2], 
                &parameters[i * 2]);
        }

        // 添加平滑性约束
        if (i > 0 && i < num_observations - 1)
        {
            problem.AddResidualBlock(
                new ceres::AutoDiffCostFunction<SmoothnessResidual, 2, 2, 2, 2>(
                    new SmoothnessResidual(10.0)),
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

    std::cout << summary.FullReport() << std::endl;

    // 写入结果
    for (size_t i = 0; i < num_observations; i++)
    {
        optimized_vehicle_info.t = track_vehicle_info_list[i].t;
        optimized_vehicle_info.x = parameters[i * 2];
        optimized_vehicle_info.y = parameters[i * 2 + 1];
        optimized_vehicle_info_list.push_back(optimized_vehicle_info);
    }
    delete [] parameters;
    std::cout << "deleting parameters" << std::endl;

    std::cout << "start writing results" << std::endl;
    WriteResults(config.output_filename, 
                 config.curve_points_filename, 
                 summary,
                 optimized_vehicle_info_list);
    std::cout << "end writing results" << std::endl;
    return 0;
}
