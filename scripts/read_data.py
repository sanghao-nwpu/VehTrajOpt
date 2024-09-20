import os
import numpy as np
import pandas as pd

# 指定要读取的文件夹路径
folder_path = '你的文件夹路径'

# 存储所有数据的列表
data_list = []

# 遍历文件夹中的所有文件
for filename in os.listdir(folder_path):
    if 'mark' in filename and filename.endswith('.txt'):  # 根据需要修改文件类型
        file_path = os.path.join(folder_path, filename)
        # 读取文件数据
        try:
            data = np.loadtxt(file_path, delimiter=',')  # 假设数据是以逗号分隔的
            data_list.append(data)
        except Exception as e:
            print(f"读取文件 {filename} 时出现错误: {e}")

# 将所有数据合并为一个 NumPy 数组
all_data = np.vstack(data_list)  # 假设所有数据列数一致

# 将数据转换为 Pandas DataFrame
df = pd.DataFrame(all_data)

# 将数据保存为 NumPy 数组和 Pandas 系列
numpy_array = all_data
pandas_series = pd.Series(df.values.flatten())  # 如果需要将 DataFrame 转换为系列

# 输出结果
print("NumPy 数组：")
print(numpy_array)
print("\nPandas 系列：")
print(pandas_series)
