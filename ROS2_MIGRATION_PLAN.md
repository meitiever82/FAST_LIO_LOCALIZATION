# FAST_LIO_LOCALIZATION ROS2 迁移计划

## 概述
将 FAST_LIO_LOCALIZATION 从 ROS1 (Noetic) 迁移到 ROS2 (Humble/Iron)

## 前提条件
- [ ] ROS2 环境已安装
- [ ] livox_ros_driver2 ROS2 版本已安装（用户自行处理）
- [ ] colcon 构建工具已安装

---

## 阶段 1: 构建系统迁移

### 1.1 修改 package.xml
- 更新为 format="3" 格式
- `catkin` → `ament_cmake`
- `roscpp` → `rclcpp`
- `tf` → `tf2`, `tf2_ros`, `tf2_geometry_msgs`
- `message_generation` → `rosidl_default_generators`
- `message_runtime` → `rosidl_default_runtime`
- 添加 `<member_of_group>rosidl_interface_packages</member_of_group>`

### 1.2 修改 CMakeLists.txt
- cmake_minimum_required 升级到 3.16
- `find_package(catkin ...)` → `find_package(ament_cmake ...)`
- 使用 `rosidl_generate_interfaces()` 生成消息
- 使用 `ament_target_dependencies()` 链接依赖
- 添加 `ament_package()` 结尾

---

## 阶段 2: 消息类型迁移

### 2.1 自定义消息 (msg/Pose6D.msg)
- 消息定义本身不需要改动
- CMakeLists.txt 中使用 rosidl 生成

### 2.2 标准消息命名空间变更
| ROS1 | ROS2 |
|------|------|
| `sensor_msgs::PointCloud2` | `sensor_msgs::msg::PointCloud2` |
| `sensor_msgs::Imu` | `sensor_msgs::msg::Imu` |
| `nav_msgs::Odometry` | `nav_msgs::msg::Odometry` |
| `nav_msgs::Path` | `nav_msgs::msg::Path` |
| `geometry_msgs::PoseStamped` | `geometry_msgs::msg::PoseStamped` |

---

## 阶段 3: 核心代码迁移 (src/laserMapping.cpp)

### 3.1 头文件替换
```cpp
// ROS1
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
```

### 3.2 节点结构重构
- 创建 `FastLIONode` 类继承 `rclcpp::Node`
- 将全局变量移入类成员
- 将回调函数改为类成员函数

### 3.3 参数系统迁移
```cpp
// ROS1
nh.param<double>("filter_size_map", filter_size_map_min, 0.5);

// ROS2
this->declare_parameter("filter_size_map", 0.5);
filter_size_map_min = this->get_parameter("filter_size_map").as_double();
```

### 3.4 发布者/订阅者迁移
```cpp
// ROS1
ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("/Odometry", 100);
ros::Subscriber sub = nh.subscribe(topic, 10, callback);

// ROS2
pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry", 100);
sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    topic, rclcpp::SensorDataQoS(),
    std::bind(&FastLIONode::callback, this, std::placeholders::_1));
```

### 3.5 TF 广播迁移
```cpp
// ROS1
static tf::TransformBroadcaster br;
tf::Transform transform;
br.sendTransform(tf::StampedTransform(transform, stamp, "camera_init", "base_link"));

// ROS2
tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
geometry_msgs::msg::TransformStamped t;
t.header.stamp = this->now();
t.header.frame_id = "camera_init";
t.child_frame_id = "base_link";
// 填充 transform 数据
tf_broadcaster_->sendTransform(t);
```

### 3.6 主循环迁移
```cpp
// ROS1
ros::Rate rate(5000);
while (ros::ok()) {
    ros::spinOnce();
    // 处理逻辑
    rate.sleep();
}

// ROS2 - 使用定时器
timer_ = this->create_wall_timer(
    std::chrono::microseconds(200),  // 5000Hz
    std::bind(&FastLIONode::timerCallback, this));
```

---

## 阶段 4: 预处理模块迁移 (src/preprocess.cpp)

### 4.1 更新消息类型
- `livox_ros_driver::CustomMsg` → `livox_ros_driver2::msg::CustomMsg`
- 更新所有 sensor_msgs 命名空间

### 4.2 更新头文件
- 对应 ROS2 消息头文件路径

---

## 阶段 5: 头文件迁移 (include/)

### 5.1 common_lib.h
- 更新 ROS 消息类型命名空间
- `ros::Time` → `rclcpp::Time` 或 `builtin_interfaces::msg::Time`

### 5.2 IMU_Processing.hpp
- 更新消息类型
- 检查时间戳处理逻辑

### 5.3 其他头文件
- `use-ikfom.hpp`, `so3_math.h`, `Exp_mat.h` - 纯数学库，无需改动
- `ikd-Tree/` - 纯 C++ 实现，无需改动
- `IKFoM_toolkit/` - 纯 C++ 实现，无需改动

---

## 阶段 6: Launch 文件迁移

### 6.1 转换为 Python 格式
将 `launch/*.launch` 转换为 `launch/*.launch.py`

示例:
```python
# launch/mapping_avia.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('fast_lio'),
        'config',
        'avia.yaml'
    )

    return LaunchDescription([
        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            name='laserMapping',
            output='screen',
            parameters=[config]
        ),
    ])
```

### 6.2 需要转换的文件
- mapping_avia.launch → mapping_avia.launch.py
- mapping_velodyne.launch → mapping_velodyne.launch.py
- mapping_ouster64.launch → mapping_ouster64.launch.py
- mapping_horizon.launch → mapping_horizon.launch.py
- mapping_mid360.launch → mapping_mid360.launch.py

---

## 阶段 7: 配置文件调整

### 7.1 YAML 参数格式
ROS2 要求参数在节点命名空间下:
```yaml
# ROS2 格式
/**:
  ros__parameters:
    common:
      lid_topic: "/livox/lidar"
      imu_topic: "/livox/imu"
    mapping:
      det_range: 300.0
```

---

## 阶段 8: 测试与验证

### 8.1 编译测试
```bash
cd ~/localization_ws
colcon build --packages-select fast_lio
source install/setup.bash
```

### 8.2 功能测试
- [ ] 节点正常启动
- [ ] 参数正确加载
- [ ] 订阅 LiDAR 和 IMU 话题
- [ ] 发布点云和里程计
- [ ] TF 变换正确广播
- [ ] 与 RViz2 正常交互

---

## 文件修改清单

| 文件 | 操作 | 优先级 |
|------|------|--------|
| package.xml | 重写 | P0 |
| CMakeLists.txt | 重写 | P0 |
| src/laserMapping.cpp | 大改 | P0 |
| src/preprocess.cpp | 中改 | P1 |
| src/preprocess.h | 小改 | P1 |
| include/common_lib.h | 小改 | P1 |
| include/IMU_Processing.hpp | 小改 | P2 |
| launch/*.launch | 重写为 .py | P2 |
| config/*.yaml | 格式调整 | P2 |

---

## 注意事项

1. **livox_ros_driver2**: 用户自行确保 ROS2 版本可用
2. **QoS 设置**: 传感器数据建议使用 `rclcpp::SensorDataQoS()`
3. **时间同步**: ROS2 时间 API 略有不同，注意 `now()` 用法
4. **线程安全**: ROS2 回调默认在不同线程，注意互斥锁

---

## 预计工作量

| 阶段 | 时间 |
|------|------|
| 阶段 1-2: 构建系统 | 2 小时 |
| 阶段 3: 核心代码 | 6 小时 |
| 阶段 4-5: 预处理和头文件 | 2 小时 |
| 阶段 6-7: Launch 和配置 | 2 小时 |
| 阶段 8: 测试调试 | 4-8 小时 |
| **总计** | **16-20 小时** |
