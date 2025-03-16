import numpy as np

# 读取 .npy 文件
file_path = "/home/dai/workspace/datasets_build/src/event_converter/output.npy"  # 替换为你的 .npy 文件路径
data = np.load(file_path, allow_pickle=True)

# 打印数据信息
print("Data shape:", data.shape)  # 打印数据的形状
print("Data type:", data.dtype)   # 打印数据的类型
print("Data content:")
print(data)  # 打印数据内容

# 访问结构化数组的字段
if data.dtype.fields is not None:  # 检查是否是结构化数组
    print("\nField names:", data.dtype.names)  # 打印字段名
    for field in data.dtype.names:
        print(f"Field '{field}':", data[field])  # 打印每个字段的值