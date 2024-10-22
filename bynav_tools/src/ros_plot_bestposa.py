import rosbag
import matplotlib.pyplot as plt
from pyproj import Proj, Transformer

# 初始化数据存储
latitudes = []
longitudes = []
heights = []

# 打开 ROS bag 文件并读取 /bynav/bestgnsspos 话题的数据
bag_file = '1018car.bag'  # 替换为您的 bag 文件路径
topic_name = '/bynav/bestpos'

with rosbag.Bag(bag_file, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        latitudes.append(msg.lat)
        longitudes.append(msg.lon)
        heights.append(msg.hgt)

# 使用 pyproj 定义转换器，将 WGS-84 坐标转换为 UTM 坐标
# 自动根据经纬度选择合适的 UTM 区域
transformer = Transformer.from_crs("EPSG:4326", "EPSG:32633", always_xy=True)

# 转换所有经纬度数据为以米为单位的平面坐标
x_coords, y_coords = transformer.transform(longitudes, latitudes)

# 绘制轨迹图（以米为单位）
plt.figure()
plt.plot(x_coords, y_coords, 'o-', linewidth=1.5)
plt.title('GNSS Trajectory in Meters (UTM)')
plt.xlabel('X (meters)')
plt.ylabel('Y (meters)')
plt.grid(True)
plt.show()

# 可视化高度变化
plt.figure()
plt.plot(range(len(heights)), heights, 'o-', linewidth=1.5)
plt.title('Height Variation from GNSS Data')
plt.xlabel('Data Point Index')
plt.ylabel('Height (meters)')
plt.grid(True)
plt.show()
