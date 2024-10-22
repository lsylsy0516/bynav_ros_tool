import matplotlib.pyplot as plt

# 初始化数据存储
latitudes = []
longitudes = []
heights = []

# 打开文件并读取内容
filename = 'gnss3.dat'  # 替换为您的文件路径
with open(filename, 'r') as file:
    for line in file:
        # 查找#BESTPOSA消息
        if '#BESTPOSA' in line:
            # 按逗号分割字符串并提取数据
            data = line.strip().split(',')
            
            try:
                # 获取纬度、经度、高度信息（第12、13、14字段）
                latitude = float(data[11])
                longitude = float(data[12])
                height = float(data[13])

                # 存储数据
                latitudes.append(latitude)
                longitudes.append(longitude)
                heights.append(height)
            except (ValueError, IndexError) as e:
                print(f"Invalid data: {line.strip()}")  # 输出无效数据行，便于调试

# 创建轨迹图
plt.figure()
plt.plot(longitudes, latitudes, 'o-', linewidth=1.5)
plt.title('BESTPOSA GNSS Trajectory')
plt.xlabel('Longitude')
plt.ylabel('Latitude')
plt.grid(True)
plt.show()

# 可视化高度变化
plt.figure()
plt.plot(range(len(heights)), heights, 'o-', linewidth=1.5)
plt.title('Height Variation from BESTPOSA')
plt.xlabel('Data Point Index')
plt.ylabel('Height (meters)')
plt.grid(True)
plt.show()
