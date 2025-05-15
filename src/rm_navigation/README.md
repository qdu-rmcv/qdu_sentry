# 青岛大学哨兵项目

青岛大学未来战队 25 赛季哨兵

## 项目简介

本项目为青岛大学RoboMaster未来战队2025赛季哨兵机器人导航与感知系统，基于 ROS 2 开发。系统集成了点云定位、路径规划与避障功能，实现了哨兵机器人在比赛场地中的自主导航与目标跟踪。

## 系统架构

```
qdu-sentry/
├── src/
│   ├── livox_ros_driver2/  # Livox激光雷达驱动
│   ├── plugins/           # 插件


|   |     ├── fake_cel_transform  #速度发布转换器，用来避免扫描模式时的追踪问题
|   |     └──pcd2pgm      # 地图转换插件，将 liosam 保存的 pcd 文件转换为 pgm 文件以供 navigation 使用
│   ├── rm_description/    # 机器人URDF描述文件
│   ├── rm_navigation/     # 导航启动节点
│   └── rm_perception/     # 感知相关功能包
│       ├── fast_lio/       # 快速激光雷达惯性里程计，用于实时定位和建图
│       ├── icp_registration/ # ICP点云配准，用于地图匹配和定位修正
│       ├── imu_complementary_filter/ # IMU互补滤波器，提高姿态估计精度(TODO)
│       ├── loam_interface/ # point_lio 等里程计算法接口
│       ├── point_lio/      # 基于点的激光雷达惯性里程计，适用于全向移动小车
│       ├── pointcloud_to_laserscan/ # 3D点云转2D激光扫描，用于2D导航
│       ├── sensor_scan_generation/  # 点云相关坐标变换，缝合 pointlio
│       └── terrain_analysis/ # 地形分析模块，评估地面可通行性
├── build/                 # 编译目录
└── install/               # 安装目录
```

## TODO

- [X] 改为Point-LIO以适应新车
- [ ] bringup 启动，参数切换 pointlio 和 fastlio？
- [ ] 修改ICP的全局重定位
- [ ] laserscan转换为voxellayer
- [ ] 确保扫描模式下导航系统稳定运行

尝试更多的感知算法，例如 fastlivo ...

## 安装说明

### 依赖项安装

1. 安装ROS 2依赖：

```bash
rosdepc install --from-paths src --ignore-src -r -y
```

### 编译项目

```bash
# 在工作空间根目录下
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --continue-on-error
source install/setup.bash
```

## 使用方法

### 建图

手动启动 livox_driver,fastlio/pointlio，建图完成后 ctrl+c 退出终端，此时在对应目录下出现 pcd 文件。
运行 pcd2pgm，将 pcd 转换为二维地图，保存并导入到对应位置。
```
ros2 run nav2_map_server map_saver_cli -f map
```

#### 启动导航系统

使用nav.sh脚本可以启动完整的导航功能，包括机器人描述、Livox雷达驱动、Point-LIO定位、点云处理和导航规划等所有必要节点：

```bash
./nav.sh
```

此脚本会自动启动以下组件：

- 机器人URDF描述 (robot_description)
- Livox Mid360雷达驱动
- Point-LIO点云定位算法
- 点云坐标系转换
- 激光雷达数据处理
- 地形分析
- 点云转激光扫描
- ICP配准
- 导航功能包

## 注意事项

- 海康MVS相机会影响LIO功能的正常使用，详见[GitHub Issues](https://github.com/orgs/qdu-rmcv/discussions/2)
- 建图完成后，地图文件(.pgm和.yaml)将保存在启动命令的当前目录下

## 鸣谢

深圳北理莫斯科大学哨兵开源项目 [https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav]
中南大学哨兵开源项目 [https://github.com/baiyeweiguang/CSU-RM-Sentry/tree/main]
华南农业大学哨兵开源项目
以及学术界的各位大佬们的开源
