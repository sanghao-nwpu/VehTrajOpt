import numpy as np
import pandas as pd
import math
import matplotlib.pyplot as plt

plt.close("all")

# 生成车辆轨迹的函数
def generate_vehicle_trajectory(total_time, dt, initial_params=None):
    if initial_params is None:
        initial_params = {
            'initial_x': 0,
            'initial_y': 0,
            'initial_angle': 0,
            'initial_v': 1,
            'initial_w': 0
        }    # 初始化参数
    t = np.arange(0, total_time, dt)
    x = np.zeros_like(t)
    y = np.zeros_like(t)
    angle = np.zeros_like(t)
    v = np.zeros_like(t)
    w = np.zeros_like(t)

    # 设置初始值
    x[0] = initial_params['initial_x']
    y[0] = initial_params['initial_y']
    angle[0] = initial_params['initial_angle']
    v[0] = initial_params['initial_v']
    w[0] = initial_params['initial_w']

    for i in range(1, len(t)):
        # 更新角度，根据角速度
        angle[i] = angle[i - 1] + w[i - 1] * dt
        
        # 计算新的位置，根据速度和角度
        x[i] = x[i - 1] + v[i - 1] * np.cos(angle[i - 1]) * dt
        y[i] = y[i - 1] + v[i - 1] * np.sin(angle[i - 1]) * dt
        v[i] = v[i - 1] + np.random.normal(0, 0.2)  # 速度加上高斯噪声
        w[i] = w[i - 1] + np.random.normal(0, np.deg2rad(0.5))
        # 这里可以根据实际情况调整速度和角速度
        # 例如：v[i] = v[0]  # 始终保持初始速度
        # 或者随时间线性变化：v[i] = initial_v + 0.1 * (i * dt)
        # w[i] = initial_w  # 始终保持初始角速度

    # 创建 DataFrame
    trajectory = pd.DataFrame({
        't': t,
        'x': x,
        'y': y,
        'angle': angle,
        'v': v,
        'w': w
    })
    
    return trajectory

# 参数设置
total_time = 10.0  # 总时间
dt = 0.1           # 时间步长

root_dir = "/home/sh/VTO_PRD/DevelopFolder"  # 请将文件名替换成你的文件名
vehicle_traj_path = root_dir + "/data/example/vehicle_traj_truth.txt"  # 请将文件名替换成你的文件名

# 生成轨迹
initial_params = {
    'initial_x': 0,
    'initial_y': 0,
    'initial_angle': 0,  # 45度
    'initial_v': 25 / 3.6,  # 25km/h
    'initial_w': 10 * np.pi / 180  # 10度/s
}
trajectory = generate_vehicle_trajectory(total_time, dt, initial_params)

# 保存到 TXT 文件
with open(vehicle_traj_path, 'w') as f:
    f.write('t x y angle v w\n')
    for index, row in trajectory.iterrows():
        f.write(
            f"{row['t']:.6f} {row['x']:.6f} {row['y']:.6f} "
            f"{row['angle']:.6f} {row['v']:.6f} {row['w']:.6f}\n"
        )
print("生成的车辆轨迹已保存为:", vehicle_traj_path)


plt.figure(figsize=(10, 6))
plt.plot(trajectory['x'], trajectory['y'], 
         marker='o', markersize=3, linestyle='-',
         color='b', alpha=0.7, label='Trajectory')
plt.title('Vehicle Trajectory')
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.grid()
plt.axis('equal')
plt.legend()
plt.show()

plt.figure(figsize=(10, 6))
plt.plot(trajectory['t'], trajectory['v'], 
         marker='o', markersize=3, linestyle='-',
         color='b', alpha=0.7, label='Velocity')
plt.title('Vehicle Velocity Over Time')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.grid()
plt.legend()
plt.show()

plt.figure(figsize=(10, 6))
plt.plot(trajectory['t'], np.degrees(trajectory['angle']), 
         marker='o', markersize=3, linestyle='-',
         color='b', alpha=0.7, label='Angle')
plt.title('Vehicle Angle Over Time')
plt.xlabel('Time (s)')
plt.ylabel('Angle (radians)')
plt.grid()
plt.legend()
plt.show()

plt.figure(figsize=(10, 6))
plt.plot(trajectory['t'], np.degrees(trajectory['w']), 
         marker='o', markersize=3, linestyle='-',
         color='b', alpha=0.7, label='Angular Velocity')
plt.title('Vehicle Angular Velocity Over Time')
plt.xlabel('Time (s)')
plt.ylabel('Angular Velocity (rad/s)')
plt.grid()
plt.legend()
plt.show()
