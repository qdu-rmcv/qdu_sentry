# 编译注意

使用本项目前请执行以下操作：

```bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
```

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

```bash
ls /dev/tty*
sudo chmod 777 /dev/tty*  # 给串口权限
```

## 运行时

运行每个节点前，都需要新建终端并先执行：
```bash
source install/setup.bash
```

### 如果没有硬件
```bash
source install/setup.bash
source /opt/ros/humble/setup.bash
ros2 launch rm_vision_bringup vision_bringup.launch.py config:=false
```

### 有硬件（连接车）
```bash
source install/setup.bash
source /opt/ros/humble/setup.bash
ros2 launch rm_vision_bringup vision_bringup.launch.py hardware:=true
```

启动可视化界面采用 foxglove，首先安装：
```bash
sudo apt install ros-humble-foxglove-bridge
```

然后在终端运行 foxglove 订阅节点：
```bash
source install/setup.bash
source /opt/ros/humble/setup.bash 
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

对于 zsh 用户：
```bash
source install/setup.zsh
source /opt/ros/humble/setup.zsh
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

## 【拓展】其他操作

### 查看相机帧率
```bash
ros2 topic hz /camera_info
```

### 相机标定
```bash
source install/setup.sh
ros2 launch hik_camera hik_camera.launch.py
ros2 run camera_calibration cameracalibrator --size 8x11 --square 0.02 image:=/image_raw camera:=/camera
```

### 关闭所有节点
```bash
ros2 node killall
```
