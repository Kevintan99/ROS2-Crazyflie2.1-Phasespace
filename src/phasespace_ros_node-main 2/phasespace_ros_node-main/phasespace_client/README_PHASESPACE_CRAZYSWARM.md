# Phase Space与Crazyswarm2集成指南

本指南说明如何将Phase Space motion capture系统与crazyswarm2框架集成。

## 概述

虽然crazyswarm2目前不直接支持Phase Space，但我们可以通过创建一个桥接节点来实现集成。这个桥接节点将Phase Space的数据格式转换为crazyswarm2期望的motion_capture_tracking格式。

## 系统架构

```
Phase Space系统 → Phase Space客户端 → 桥接节点 → Crazyswarm2
```

1. **Phase Space客户端**: 连接到Phase Space系统并发布rigid body数据
2. **桥接节点**: 将Phase Space数据转换为motion_capture_tracking格式
3. **Crazyswarm2**: 接收转换后的数据并控制无人机

## 安装和编译

### 1. 编译Phase Space包

```bash
cd ~/ros2_ws
colcon build --packages-select phasespace_client phasespace_msgs
source install/setup.bash
```

### 2. 编译Crazyswarm2

```bash
cd ~/ros2_ws
colcon build --packages-select crazyflie
source install/setup.bash
```

## 配置

### 1. Phase Space配置

确保您已经：
- 配置了Phase Space系统
- 创建了rigid body配置文件（JSON格式）
- 将配置文件放在`phasespace_client/rigid_body_objects/`目录下

### 2. Crazyswarm2配置

修改`src/crazyswarm2/crazyflie/config/motion_capture.yaml`：

```yaml
/motion_capture_tracking:
  ros__parameters:
    type: "phasespace"  # 改为phasespace
    hostname: "your_phasespace_ip"
    # ... 其他配置保持不变
```

### 3. Rigid Body映射

在启动时指定Phase Space ID到Crazyflie名称的映射：

```bash
# 格式: "PhaseSpace_ID:Crazyflie_Name"
rigid_body_mapping="1:cf1,2:cf2,3:cf3"
```

## 使用方法

### 方法1: 使用启动脚本（推荐）

```bash
cd ~/ros2_ws
source install/setup.bash

# 使用默认参数
./src/phasespace_ros_node-main/phasespace_client/scripts/start_phasespace_crazyswarm.sh

# 或指定参数
./src/phasespace_ros_node-main/phasespace_client/scripts/start_phasespace_crazyswarm.sh \
    192.168.1.17 \
    drone.json \
    1 \
    "1:cf1,2:cf2,3:cf3"
```

### 方法2: 手动启动

```bash
# 终端1: 启动Phase Space客户端和桥接节点
ros2 launch phasespace_client phasespace_crazyswarm.launch.py \
    address:=192.168.1.17 \
    json_file_name:=drone.json \
    body_to_track:=1 \
    rigid_body_mapping:="1:cf1,2:cf2,3:cf3"

# 终端2: 启动Crazyswarm2
ros2 launch crazyflie launch.py \
    crazyflies_yaml_file:=src/crazyswarm2/crazyflie/config/crazyflies.yaml \
    motion_capture_yaml_file:=src/crazyswarm2/crazyflie/config/motion_capture.yaml \
    backend:=cpp \
    mocap:=True
```

## 坐标系转换

桥接节点会自动处理以下坐标系转换：

1. **单位转换**: Phase Space使用毫米，转换为米
2. **坐标系转换**: Phase Space使用XZY坐标系，转换为ROS的XYZ坐标系
3. **姿态转换**: 四元数坐标系转换

## 故障排除

### 1. 检查Phase Space连接

```bash
# 检查Phase Space话题
ros2 topic list | grep phasespace
ros2 topic echo /phasespace_rigids
```

### 2. 检查桥接节点

```bash
# 检查转换后的话题
ros2 topic echo /poses
```

### 3. 检查Crazyswarm2

```bash
# 检查motion capture状态
ros2 topic echo /crazyflie_server/motion_capture_status
```

### 4. 常见问题

- **无数据**: 检查Phase Space IP地址和网络连接
- **坐标系错误**: 检查rigid body映射配置
- **频率问题**: 检查Phase Space数据频率设置

## 高级配置

### 自定义坐标系转换

如果需要自定义坐标系转换，可以修改`phasespace_to_mocap_bridge.cpp`中的转换矩阵。

### 多无人机配置

对于多无人机系统，确保：
1. 每个无人机有唯一的rigid body ID
2. 正确配置rigid body映射
3. 在`crazyflies.yaml`中配置所有无人机

## 性能优化

1. **数据频率**: 确保Phase Space数据频率足够高（建议100Hz）
2. **网络延迟**: 使用有线连接减少网络延迟
3. **CPU使用**: 监控桥接节点的CPU使用率

## 扩展功能

### 添加新的motion capture系统

可以参考这个桥接节点的实现，为其他motion capture系统创建类似的桥接节点。

### 数据记录

可以添加数据记录功能来保存motion capture数据用于后续分析。

## 技术支持

如果遇到问题，请检查：
1. ROS2话题连接状态
2. Phase Space系统状态
3. 网络连接
4. 配置文件格式

更多信息请参考：
- [Phase Space文档](https://customers.phasespace.com/)
- [Crazyswarm2文档](https://github.com/IMRCLab/crazyswarm2) 