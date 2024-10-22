import math

# 常量定义
PI = 3.14159265358979324
a = 6378245.0  # 长半轴
ee = 0.00669342162296594323  # 偏心率平方

def transform_lat(x, y):
    ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * math.sqrt(abs(x))
    ret += ((20.0 * math.sin(6.0 * x * PI) + 20.0 * math.sin(2.0 * x * PI)) * 2.0) / 3.0
    ret += ((20.0 * math.sin(y * PI) + 40.0 * math.sin((y / 3.0) * PI)) * 2.0) / 3.0
    ret += ((160.0 * math.sin((y / 12.0) * PI) + 320.0 * math.sin((y * PI) / 30.0)) * 2.0) / 3.0
    return ret

def transform_lon(x, y):
    ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * math.sqrt(abs(x))
    ret += ((20.0 * math.sin(6.0 * x * PI) + 20.0 * math.sin(2.0 * x * PI)) * 2.0) / 3.0
    ret += ((20.0 * math.sin(x * PI) + 40.0 * math.sin((x / 3.0) * PI)) * 2.0) / 3.0
    ret += ((150.0 * math.sin((x / 12.0) * PI) + 300.0 * math.sin((x / 30.0) * PI)) * 2.0) / 3.0
    return ret

def delta(lon, lat):
    d = [0.0, 0.0]
    dLon = transform_lon(lon - 105.0, lat - 35.0)
    dLat = transform_lat(lon - 105.0, lat - 35.0)

    radLat = (lat / 180.0) * PI
    magic = math.sin(radLat)
    magic = 1.0 - ee * magic * magic
    sqrtMagic = math.sqrt(magic)

    d[0] = (dLon * 180.0) / (a / sqrtMagic * math.cos(radLat) * PI)
    d[1] = (dLat * 180.0) / ((a * (1.0 - ee)) / (magic * sqrtMagic) * PI)
    return d

def WGS84ToGCJ02(lon_WGS, lat_WGS):
    d = delta(lon_WGS, lat_WGS)
    GCJ = [lon_WGS + d[0], lat_WGS + d[1]]
    return GCJ

def GCJ02ToWGS84(lon_GCJ, lat_GCJ):
    WGS = [lon_GCJ, lat_GCJ]
    temp = WGS84ToGCJ02(WGS[0], WGS[1])
    dx = temp[0] - lon_GCJ
    dy = temp[1] - lat_GCJ
    while abs(dx) > 1e-6 or abs(dy) > 1e-6:
        WGS[0] -= dx
        WGS[1] -= dy
        temp = WGS84ToGCJ02(WGS[0], WGS[1])
        dx = temp[0] - lon_GCJ
        dy = temp[1] - lat_GCJ
    return WGS