# HIKROBOT MVS ROS 2 相机驱动

基于海康 MVS SDK 的 ROS 2 Humble C++ 相机驱动，支持 USB 3.0 和 GigE 接口的海康工业相机。

功能包括：
- 自动发现并连接相机
- 支持按序列号或 IP 地址选择相机
- 发布标准 sensor_msgs/msg/Image 图像话题
- 支持运行时动态修改曝光、增益、帧率、像素格式
- 支持断线重连
- 支持通过 launch 文件启动

---

## 一、系统要求

- 操作系统：Ubuntu 22.04 (Jammy)
- ROS 版本：ROS 2 Humble
- 相机：海康机器人工业相机（USB 3.0 或 GigE）
- SDK：海康 MVS SDK for Linux (x86_64)

---

## 二、安装 ROS 2 Humble

如果尚未安装：

sudo apt update
sudo apt install ros-humble-desktop ros-dev-tools python3-colcon-common-extensions -y

配置环境变量：

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

---

## 三、安装海康 MVS SDK

1. 前往海康机器人官网下载 MVS for Linux (x86_64)：
   https://www.hikrobotics.com/en/machinevision/service/download

2. 解压并安装：

cd ~/下载
unzip MVS-*.zip
sudo dpkg -i MVS-*.deb
sudo apt --fix-broken install -y

3. 验证安装：

ls /opt/MVS
ls /opt/MVS/include/MvCameraControl.h
ls /opt/MVS/lib/64/libMvCameraControl.so

4. 配置动态库路径：

export LD_LIBRARY_PATH=/opt/MVS/lib/64:$LD_LIBRARY_PATH

---

## 四、编译本功能包

1. 将本功能包放入工作空间：

mkdir -p ~/ros2_ws/src
cp -r hikrobot_camera ~/ros2_ws/src/

2. 安装依赖并编译：

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select hikrobot_camera --cmake-args -DMVS_SDK_ROOT=/opt/MVS

3. 配置环境：

source ~/ros2_ws/install/setup.bash

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
echo "export LD_LIBRARY_PATH=/opt/MVS/lib/64:\$LD_LIBRARY_PATH" >> ~/.bashrc
source ~/.bashrc

---

## 五、配置 USB 权限（仅 USB 相机）

否则会报错 Could not open camera: 0x80000203

echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="2bdf", MODE="0666"' | sudo tee /etc/udev/rules.d/99-hikrobot.rules
sudo udevadm control --reload-rules
sudo udevadm trigger

执行后必须拔插一次相机 USB 线。

注意：USB 3.0 相机必须插在电脑原生蓝色 USB 3.0 接口上。

---

## 六、确认相机序列号

1. 打开 MVS 客户端。
2. 记录相机序列号（例如 DB0178696）。
3. 彻底关闭 MVS 客户端。

命令行查询：

lsusb | grep -i hik

---

## 七、启动相机节点

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export LD_LIBRARY_PATH=/opt/MVS/lib/64:$LD_LIBRARY_PATH

ros2 launch hikrobot_camera camera.launch.py serial_number:=DB0178696

---

## 八、查看图像

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run rqt_image_view rqt_image_view

在窗口左上角下拉菜单中选择 /image_raw

---

## 九、动态调节参数

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 param set /hikrobot_camera exposure_time 10000.0
ros2 param set /hikrobot_camera gain 10.0
ros2 param set /hikrobot_camera frame_rate 15.0
ros2 param set /hikrobot_camera pixel_format BGR8

ros2 param list /hikrobot_camera
ros2 param get /hikrobot_camera exposure_time

---

## 十、参数说明

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| serial_number | '' | 相机序列号选择器 |
| ip_address | '' | GigE 相机 IPv4 地址 |
| image_topic | /image_raw | 图像话题名 |
| frame_id | camera_optical_frame | 图像坐标系 |
| exposure_time | 5000.0 | 曝光时间（微秒） |
| gain | 0.0 | 增益（dB） |
| frame_rate | 30.0 | 采集帧率（Hz） |
| pixel_format | BGR8 | BGR8 / RGB8 / Mono8 |
| reconnect_interval_ms | 2000 | 重连间隔（毫秒） |
| grab_timeout_ms | 1000 | 取帧超时（毫秒） |

---

## 十一、常见问题

| 现象 | 原因 | 解决 |
| --- | --- | --- |
| 0x80000203 | 权限不足或被占用 | 配置 udev、关闭 MVS、拔插 USB |
| Package not found | 未 source 环境 | source install/setup.bash |
| 编译报 destRed 错误 | 源码笔误 | 改为 static_cast<MvGvspPixelType>(*desired) |
| 画面全黑 | 曝光太短或光圈太小 | 调大曝光和增益，拧大光圈 |
| 画面卡顿 | 分辨率或帧率过高 | 降低分辨率或帧率 |
| MVS SDK was not found | SDK 路径不对 | 指定 -DMVS_SDK_ROOT=/opt/MVS |

---

## 十二、目录结构

hikrobot_camera/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── camera.yaml
├── launch/
│   └── camera.launch.py
└── src/
    └── hikrobot_camera_node.cpp

---

## 十三、快速开始

sudo apt install ros-humble-desktop ros-dev-tools python3-colcon-common-extensions -y
cd ~/下载 && sudo dpkg -i MVS-*.deb && sudo apt --fix-broken install -y

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select hikrobot_camera --cmake-args -DMVS_SDK_ROOT=/opt/MVS
source install/setup.bash
export LD_LIBRARY_PATH=/opt/MVS/lib/64:$LD_LIBRARY_PATH

echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="2bdf", MODE="0666"' | sudo tee /etc/udev/rules.d/99-hikrobot.rules
sudo udevadm control --reload-rules && sudo udevadm trigger

ros2 launch hikrobot_camera camera.launch.py serial_number:=你的序列号

ros2 run rqt_image_view rqt_image_view
