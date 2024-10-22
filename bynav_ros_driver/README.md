# 安装依赖
运行脚本文件 install_dependencies.sh
1. 在线安装
```shell
./install_dependencies.sh online
```
2. 离线安装
```shell
./install_dependencies.sh offline
```

# 编译
运行脚本文件 build.sh

# 加载环境
```shell
. devel/setup.bash
```

# 运行
支持串口和网口两种通信方式
1. 串口连接方式
```shell
roslaunch bynav_ros_driver connect_tty.launch
```
2. 网口连接方式
```shell
roslaunch bynav_ros_driver connect_net.launch
```

在 launch 文件中可以修改串口和网口连接信息

在config文件夹下的ntrip_parameters.yaml文件中可以启用ntrip和配置ntrip相关参数。

**注意：**
1. 如果需要使用IMU的相关数据，那么使用前需要根据具体的IMU型号在 std_driver_config.xml 文件中配置 IMU 频率以及 gyro 和 acc 的scale
2. 如果脚本文件无法执行，可以通过下面的指令增加可执行权限
```shell
chmod +x build.sh
chmod +x install_dependencies.sh
```