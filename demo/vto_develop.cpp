#include <iostream>
#include <vector>
#include <ceres/ceres.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <sstream>
#include <iomanip> // 添加此行以包含 setprecision

/** @brief 结构体定义 */

struct Config {
    std::string input_filename;
    std::string output_filename;
    std::string curve_points_filename;
    bool ceres_auto_diff_flag;
};

// 轨迹观测点结构
struct VTO_Observation {
    double x; // 观测点的x坐标
    double y; // 观测点的y坐标
    double angle; // 观测点的角度
    double velo_x; // 观测点的速度
    double velo_y; // 观测点的速度
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
        // 处理注释，忽略以#开头的行
        size_t comment_pos = line.find('#');
        if (comment_pos != std::string::npos) {
            line = line.substr(0, comment_pos);
        }

        // 去除前后空白字符
        line.erase(line.begin(), std::find_if(line.begin(), line.end(), [](unsigned char ch) {
            return !std::isspace(ch);
        }));
        line.erase(std::find_if(line.rbegin(), line.rend(), [](unsigned char ch) {
            return !std::isspace(ch);
        }).base(), line.end());

        // 处理键值对
        std::istringstream iss(line);
        std::string key;
        if (std::getline(iss, key, '=')) {
            std::string value;
            if (std::getline(iss, value)) {
                // 去除值的前后空白字符
                value.erase(value.begin(), std::find_if(value.begin(), value.end(), [](unsigned char ch) {
                    return !std::isspace(ch);
                }));
                value.erase(std::find_if(value.rbegin(), value.rend(), [](unsigned char ch) {
                    return !std::isspace(ch);
                }).base(), value.end());

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

// 单帧观测残差函数
struct TrajectoryResidual {
    TrajectoryResidual(double observed_x, double observed_y)
        : observed_x_(observed_x), observed_y_(observed_y) {}

    //括号重载函数参数包含优化变量和残差
    template <typename T>
    bool operator()(const T *x, const T *y, T* residual) const {
        residual[0] = *x - T(observed_x_);
        residual[1] = *y - T(observed_y_);
        return true;
    }

    double observed_x_;
    double observed_y_;
};

// 帧间观测残差函数
struct InterFrameResidual {
    InterFrameResidual() {}

    // 括号重载函数参数包含两个帧的优化变量和残差
    template <typename T>
    bool operator()(const T* x1, const T* y1, 
                    const T* x2, const T* y2, 
                    const T* x3, const T* y3, 
                    T* residual) const {
        // 计算残差
        residual[0] = (*x3 - *x2) - (*x2 - *x1); // 示例：两个帧的x坐标的平均值
        residual[1] = (*y3 - *y2) - (*y2 - *y1); // 示例：两个帧的y坐标的平均值
        return true;
    }
};


// 定义多项式自动求导的代价函数
struct SplineAutoDiffResidual
{
    SplineAutoDiffResidual(double t, double x, double y) : t_(t), x_(x), y_(y) {}

    template <typename T>
    bool operator()(const T *const coeffs_x, const T *const coeffs_y, T *residual) const
    {
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

    // 为每一个数据点添加残差
    for (size_t i = 0; i < t_data.size(); ++i) 
    {
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<TrajectoryResidual, 2, 1, 1>(
                new TrajectoryResidual(x_data[i], y_data[i])),
            nullptr,
            &x_points[i],
            &y_points[i]);
    }
    for (size_t i = 1; i < t_data.size() - 1; ++i)
    {
        problem.AddResidualBlock(new ceres::AutoDiffCostFunction<InterFrameResidual, 2, 1, 1, 1, 1, 1, 1>(
            new InterFrameResidual()),
            nullptr,
            &x_points[i-1],
            &y_points[i-1],
            &x_points[i],
            &y_points[i],
            &x_points[i+1],
            &y_points[i+1]);
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
    WriteResults(config.output_filename, 
                 config.curve_points_filename, 
                 x_points, 
                 y_points, 
                 summary, 
                 t_data);


    return 0;
}
