import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
from scipy.interpolate import UnivariateSpline

# 设置随机种子以便复现结果
np.random.seed(42)

# 生成样本数据
t = np.linspace(0, 10, 101)  # t 范围从 0 到 10，共 10 个点
# coeff_x = np.random.rand(4)  # 随机生成一个三次多项式的系数
# coeff_y = np.random.rand(4)  # 随机生成一个三次多项式的系数

coeff_x = [0.01, 0.0, -1.0, 0.0]  # 你可以根据需要修改这些值
coeff_y = [0.0, 0.5, -0.5, 1.0]  # 你可以根据需要修改这些值
# x = np.polyval(coeff_x, t) + np.random.normal(0, 0.5, len(t))  # 添加噪声的 x 数据
x = np.polyval(coeff_x, t)
y = np.polyval(coeff_y, t) + np.random.normal(0, 0.5, len(t))  # 添加噪声的 y 数据

# 拟合三次多项式
coeff_fit_x = np.polyfit(t, x, 3)
coeff_fit_y = np.polyfit(t, y, 3)
print(f"拟合后的系数为: {coeff_fit_x}, {coeff_fit_y}")

# 计算拟合后的曲线
fit_t = np.linspace(0, 10, 1000)
fit_x = np.polyval(coeff_fit_x, fit_t)
fit_y = np.polyval(coeff_fit_y, fit_t)

# 创建三次样条拟合
spline_x = UnivariateSpline(t, x, s=1)  # s 是平滑因子
spline_y = UnivariateSpline(t, y, s=1)

# 生成更细的 t 点用于绘图和记录
spl_t = np.linspace(0, 10, 1000)
spl_x = spline_x(fit_t)
spl_y = spline_y(fit_t)

# 将带噪声的原始数据和拟合后的曲线都存储到文件
output_filename = "curve_with_noise.txt"

with open(output_filename, 'w') as f:
    for ti, xi, yi in zip(t, x, y):
        f.write(f"{ti:.6f} {xi:.6f} {yi:.6f}\n")

print(f"数据已保存到文件: {output_filename}")

# 可视化结果
fignum = 0
fignum += 1
plt.figure(fignum, figsize=(12, 8))
plt.scatter(t, x, color='red', label='Noisy Data (x)', zorder=5)
plt.scatter(t, y, color='blue', label='Noisy Data (y)', zorder=5)
plt.plot(fit_t, fit_x, label='Cubic Spline (x)', color='orange')
plt.plot(fit_t, fit_y, label='Cubic Spline (y)', color='green')
plt.title('Cubic Spline with Noise')
plt.xlabel('t')
plt.ylabel('Value')
plt.legend()
plt.grid()
plt.show()
print("可视化结果已显示 1/2")

fignum += 1
plt.figure(fignum)
plt.scatter(x, y, color='red', label='Noisy Data', zorder=5)
plt.plot(fit_x, fit_y, label='polyfit', color='orange')
plt.plot(spl_x, spl_y, label='Cubic Spline', color='green')
plt.title('Cubic Spline with Noise')
plt.xlabel('x')
plt.ylabel('y')
plt.legend()
plt.grid()
plt.axis('equal')
plt.show()
print("可视化结果已显示 2/2")
