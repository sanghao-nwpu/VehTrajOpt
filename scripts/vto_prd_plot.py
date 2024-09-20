import numpy as np
import matplotlib.pyplot as plt
# import matplotlib.font_manager as fm

plt.close('all')  # 关闭所有窗口

# # 获取当前字体列表
# fonts = fm.findSystemFonts(fontpaths=None, fontext='ttf')
# # print(fonts)

# # 设置字体
# font = fm.FontProperties(fname=r'/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc', size=14)

# 读取第一个文件内容
root_dir = "/home/sh/VTO_PRD/DevelopFolder/VehTrajOpt/data/20240919_traj_1_1"  # 请将文件名替换成你的文件名
input_data_path = root_dir + "/track_vehicle_info.txt"  # 请将文件名替换成你的文件名
data_input = np.loadtxt(input_data_path)
output_data_path =  root_dir + "/track_vehicle_info_optimized.txt"  # 请将文件名替换成你的文件名
data_output = np.loadtxt(output_data_path)


# 设置图形大小
plt.figure(figsize=(12, 8))
plt.plot(data_input[:, 2], data_input[:, 3],
         marker='o', markersize=3, linestyle='-',
         color='b', alpha=0.7, label='input')
plt.plot(data_output[:, 1], data_output[:, 2],
         marker='o', markersize=3, linestyle='-',
         color='r', alpha=0.7, label='output')
# 设置标题和标签
plt.title('x-y 轨迹比较', fontsize=16)
plt.xlabel('X [m]', fontsize=14)
plt.ylabel('Y [m]', fontsize=14)

# 设置网格
plt.grid(True, linestyle='--', alpha=0.7)

# 添加图例
plt.legend(fontsize=12)

plt.axis('equal')  # 保持x，y轴比例一致

# 显示图形
plt.tight_layout()  # 自动调整布局
plt.show()
