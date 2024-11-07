from pathlib import Path

import requests
from OpenSSL import SSL
import numpy as np
import folium
from folium import plugins
import subprocess
from PIL import Image
import io
from html2image import Html2Image
import os

# ##########################################全局变量#############################################################
# 替换为你的高德API密钥,在高德开放平台里注册-》应用管理-》创建新应用-》服务平##台那里选择web服务-》记住生成的key，这里没有密钥的。后续填写到amap_key中
amap_key = '82ed988a4b5e5da017c5f4b038604822'
list_latlon=[]
Lon = []
Lat = []

# 仓库经纬度（格式：经度,纬度）
warehouse_location = '113.967847,22.592495'

# 农田经纬度（格式：经度,纬度）,换成自己的
farm_location = '113.98053,22.585877'  #113.98053,22.585877 113.981088,22.586437'


strategy = 11
alternative_route = 3
# #######################################################################################################
colors = ['#3388ff', '#66cc33', '#ff3333', '#ff9900', '#cc66ff', '#33ff33', '#0033ff']
# 绘制地图
san_map = folium.Map(
        location=[22.589186, 113.974188],
        zoom_start=17,
        control_scale=True,
        tiles='http://www.google.cn/maps/vt?lyrs=s@189&gl=cn&x={x}&y={y}&z={z}', # google卫星图
        attr='default')
def PlotLineOnMap(Lat, Lon, index):
    tri = np.array(list(zip(Lat, Lon)))
    folium.PolyLine(tri, color=colors[index % len(colors)]).add_to(san_map)
    # marker_cluster = plugins.MarkerCluster().add_to(san_map)
    # for lat, lon in zip(Lat, Lon):
    #     folium.Marker([lat, lon], color='red').add_to(marker_cluster)



# 获取高德地图的路径规划点
def get_route(start, end, mode, amap_key):
    # 这里的url中选择是步行，公交还是驾车路径，本文中driving?表示驾车，具体介绍见：https://lbs.amap.com/api/webservice/guide/api/direction
    url = f'https://restapi.amap.com/v5/direction/driving?origin={start}&destination={end}&strategy={mode}&show_fields=polyline&key={amap_key}'   # jiache路径规划
    # url = f'https://restapi.amap.com/v5/direction/walking?isindoor=0&origin={start}&destination={end}&alternative_route={alternative_route}&show_fields=polyline&key={amap_key}'    # 步行路径规划
    # url = f'https://restapi.amap.com/v5/direction/electrobike?&origin={start}&destination={end}&alternative_route={alternative_route}&show_fields=polyline&key={amap_key}'    # ebike路径规划

    response = requests.get(url)
    data = response.json()

    if data['status'] == '1':
        routes = data['route']['paths']
        return routes
    else:
        print('请求失败，请检查输入参数。')
        return None

def get_Lat_Lon(route):
    for i, step in enumerate(route):
        list_latlon.append(step["polyline"])
        # print(f'步骤 {i + 1}: {step["instruction"]}:{step["polyline"]}')
    for item in list_latlon:
        points = item.split(';')
        for point in points:
            coords = point.split(',')
            Lon.append(float(coords[0]))
            Lat.append(float(coords[1]))
    return Lat, Lon

# 主函数

def main():
    routes = get_route(warehouse_location, farm_location, strategy, amap_key)
    for index, route in enumerate(routes):
        route = route['steps']
        # print(route)
        if route:
            Lat, Lon = get_Lat_Lon(route)
        else:
            print('无法获取路线规划。')
        # 绘制路径规划线路
        PlotLineOnMap(Lat, Lon, index)
        list_latlon.clear()
        Lon.clear()
        Lat.clear()

        break
    # san_map.fit_bounds([[113.967847,22.592495],
    #                     [113.98053, 22.592495],
    #                     [113.967847, 22.585877],
    #                [113.98053,22.585877]])
    san_map.save('showpoint11.html')

    # img_data = san_map._to_png(5)
    # img = Image.open(io.BytesIO(img_data))
    # img.save('image.png')
    # try:
    #     subprocess.run(['wkhtmltoimage', 'showpoint10.html', 'showpoint10.png'])
    #     print("HTML文件已成功转换为图片。")
    # except Exception as e:
    #     print(f"转换失败：{e}")
    root = san_map.get_root()
    html = root.render()  # 这个拿到的就是一个html的内容
    # mo.save('text.html')
    # 2.使用Html2Image将地图html文件转成png

    base = Path(__file__).resolve().parent
    # 以下为Html2Image参数的2中写法，custom_flags参数是网页生成后延迟10秒生成图片（地图加载慢，眼部就会出现空白方块，output_path 是文件生成后存储的文件夹，screenshot为生成图片的方法）
    hti = Html2Image(custom_flags=['--virtual-time-budget=10000', '--hide-scrollbars'])
    hti.output_path = base
    hti.screenshot(html_str=str(html), save_as='test.png')

if __name__ == '__main__':
    main()



