# laser_reflector_detect

ROS功能包，用于激光雷达反光柱检测、SLAM建图和定位。

## 项目结构

```
laser_reflector_detect/
├── cmake/
│   └── modules/
│       └── FindEigen3.cmake          # Eigen3库的CMake查找模块
├── include/ros_qt5/
│   ├── laser_processor.h             # 激光数据处理器头文件
│   ├── qnode.h                        # ROS Qt节点主类头文件
│   ├── reflector_localization_node.h # 反光柱定位节点头文件
│   └── reflector_slam_node.h          # 反光柱SLAM节点头文件
├── launch/
│   ├── laser_reflector_detect_landmark_navigation_one.launch  # 导航launch文件
│   ├── reflector_localization.launch  # 定位节点启动文件
│   └── reflector_slam.launch          # SLAM建图启动文件
├── msg/
│   ├── reflector.msg                  # 反光柱消息类型定义
│   └── reflectordata.msg              # 反光柱数据消息类型定义
├── param/
│   ├── laser_reflector_detect_navigation_one.yaml # 导航参数配置
│   ├── reflector_localization.yaml    # 定位参数配置
│   └── reflector_slam.yaml            # SLAM建图参数配置
├── src/
│   ├── laser_processor.cpp            # 激光数据处理器实现
│   ├── main.cpp                       # Qt应用主入口
│   ├── qnode.cpp                      # ROS Qt节点实现
│   ├── reflector_localization_node.cpp # 定位节点实现
│   ├── reflector_slam_node.cpp        # SLAM节点实现
│   └── savereflector.csv              # 反光柱地图数据文件
├── srv/
│   └── igk_reflector_srv.srv          # 反光柱服务类型定义
├── CMakeLists.txt                     # CMake构建配置
├── CMakeLists.txt.bak                 # CMake配置备份
├── CMakeLists.txt.user                # Qt Creator项目文件
└── package.xml                        # ROS包清单文件
```

## 文件详细说明

### 消息类型 (msg/)

| 文件 | 说明 |
|------|------|
| `reflector.msg` | 单个反光柱的基本信息，包含区域(area)、ID、坐标(x,y) |
| `reflectordata.msg` | 反光柱数据数组，包含消息头、命名空间和反光柱数组 |

### 服务类型 (srv/)

| 文件 | 说明 |
|------|------|
| `igk_reflector_srv.srv` | 反光柱管理服务，支持添加、删除、清空反光柱等操作 |

### 核心节点 (src/ + include/ros_qt5/)

| 源文件 | 头文件 | 说明 |
|--------|--------|------|
| `reflector_slam_node.cpp` | `reflector_slam_node.h` | SLAM建图节点，实现基于ICP算法的激光里程计，用于在不使用cartographer的情况下建立反光柱地图 |
| `reflector_localization_node.cpp` | `reflector_localization_node.h` | 定位节点，基于已知地图进行反光柱匹配定位 |
| `qnode.cpp` | `qnode.h` | ROS Qt节点，整合激光处理和可视化功能 |
| `laser_processor.cpp` | `laser_processor.h` | 激光数据处理器，负责从激光扫描数据中提取反光柱特征点 |

### 配置文件 (param/)

| 文件 | 说明 |
|------|------|
| `reflector_slam.yaml` | SLAM建图参数：反光柱强度阈值、ICP参数、地图保存路径等 |
| `reflector_localization.yaml` | 定位参数：匹配距离阈值、最小匹配数量、地图文件路径等 |
| `laser_reflector_detect_navigation_one.yaml` | 导航相关参数 |

### 启动文件 (launch/)

| 文件 | 说明 |
|------|------|
| `reflector_slam.launch` | 启动SLAM建图节点，加载建图参数 |
| `reflector_localization.launch` | 启动定位节点，加载定位参数 |
| `laser_reflector_detect_landmark_navigation_one.launch` | 启动完整导航流程 |

### 构建配置

| 文件 | 说明 |
|------|------|
| `CMakeLists.txt` | Catkin构建配置，定义编译规则、依赖项、消息/服务生成等 |
| `package.xml` | ROS包清单，定义包名、版本、依赖关系等元信息 |

## 依赖项

- roscpp
- std_msgs
- sensor_msgs
- nav_msgs
- geometry_msgs
- visualization_msgs
- tf / tf2
- cv_bridge
- cartographer_ros_msgs
- Qt5
- Ceres
- Eigen3

## 使用方法

### SLAM建图
```bash
roslaunch laser_reflector_detect reflector_slam.launch
```

### 定位
```bash
roslaunch laser_reflector_detect reflector_localization.launch
```
