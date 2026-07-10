# AirSLAM 迁移总结：ROS1→ROS2 Humble + TensorRT 10.9

> 日期：2026-07-10 | 构建环境：Ubuntu 22.04 · CUDA 13.0 · TensorRT 10.9.0.34 · ROS2 Humble
> 状态：✅ 编译通过 · ✅ EuRoC V1_01_easy 实时运行 (2-7ms/帧)

---

## 1. 迁移概览

| 阶段 | 方法 | 结果 |
|------|------|------|
| ROS1→ROS2 迁移 | 11-agent 工作流 (8×Haiku + 3×Opus) | 18 文件自动迁移 |
| TRT 8.x→10.9 API | 手动替换 deprecated API | 5 源文件、~60 处 API 变更 |
| ONNX 模型修复 | Python 脚本插入 Cast(int64→int32) | 1 个模型 (SuperPoint) |
| CUDA 13.0 适配 | link_directories + cudart | 2 CMake 修复 |
| 编译验证 | 11 次迭代编译 | libair_slam_lib.so 5.5MB + 4 可执行文件 |
| EuRoC 运行 | ros2 launch 直接启动 | V1_01_easy 稳定 140-500 FPS |

---

## 2. ROS1→ROS2 Humble 迁移

使用多模型工作流：**Haiku** 负责代码分析和编写，**Opus** 负责架构设计、编译验证和总结。

| 阶段 | 模型 | 代理 | 内容 |
|------|------|------|------|
| 分析 | Haiku ×3 | 3 | ROS1 API 清单、CMake 依赖、launch 配置 |
| 设计 | Opus ×1 | 1 | 20 文件逐步迁移计划 |
| 实现 | Haiku ×5 | 5 | package.xml, CMakeLists.txt, launch, 头文件, 源文件 |
| 验证 | Opus ×1 | 1 | 8 问题审查 (2 CRITICAL, 2 MEDIUM, 4 LOW) |
| 总结 | Opus ×1 | 1 | 完整迁移报告含构建说明 |

**工作流统计**：11/11 代理 · 463K tokens · 173 工具调用 · ~13.6 分钟

### 核心 API 映射

| ROS1 | ROS2 Humble |
|------|-------------|
| `ros::NodeHandle` | `rclcpp::Node::SharedPtr` |
| `ros::Publisher` / `ros::Subscriber` | `rclcpp::Publisher<T>::SharedPtr` |
| `ros::Time::now()` | `node->now()` |
| `nh.advertise<T>("topic", q)` | `node->create_publisher<T>("topic", q)` |
| `nh.param<T>("key", val, def)` | `node->declare_parameter<T>()` + `get_parameter()` |
| `ros::spinOnce()` | `rclcpp::spin_some(node)` |
| `ros::ok()` | `rclcpp::ok()` |
| `tf::TransformBroadcaster` | `tf2_ros::TransformBroadcaster` |
| `ROS_INFO(...)` | `RCLCPP_INFO(node->get_logger(), ...)` |
| `<geometry_msgs/PoseStamped.h>` | `<geometry_msgs/msg/pose_stamped.hpp>` |
| `<launch>` XML | Python `LaunchDescription` |
| catkin / `catkin_package()` | ament_cmake / `ament_package()` |

### 迁移的文件 (18)

- `package.xml` ×2：format 1→3, catkin→ament_cmake, roscpp→rclcpp
- `CMakeLists.txt` ×3：per-package find_package, target_include_directories, install 规则
- `launch/**/*.launch.py` ×10：XML→Python (ament_index_python, DeclareLaunchArgument)
- `include/*.h` ×5：rclcpp 头文件, msg:: 命名空间, tf2_ros
- `src/*.cc` ×4：~65 处 ros_publisher.cc 编辑 + 构造函数、Rate、Time
- `demo/*.cpp` ×4：rclcpp::init/shutdown, Node::SharedPtr, declare_parameter

---

## 3. TensorRT 8.x→10.9 API 迁移

### 已替代的 deprecated API

| TRT 8.x (deprecated) | TRT 10.9 (new) | 影响文件 |
|------|------|------|
| `engine->getBindingIndex(name)` | `getIOTensorIndex(engine, name)` [helper] | plnet, super_glue, light_glue |
| `context->setBindingDimensions(i, d)` | `context->setInputShape(name, d)` | 全部 4 模型 |
| `context->getBindingDimensions(i)` | `context->getTensorShape(name)` | super_glue, light_glue |
| `engine->getNbBindings()` | `engine->getNbIOTensors()` | buffers.h + 3 源文件 |
| `engine->getBindingDataType(i)` | `engine->getTensorDataType(name)` | buffers.h |
| `engine->getBindingVectorizedDim(i)` | `engine->getTensorVectorizedDim(name)` | buffers.h |
| `engine->getBindingComponentsPerElement(i)` | `engine->getTensorComponentsPerElement(name)` | buffers.h |
| `engine->bindingIsInput(i)` | `engine->getTensorIOMode(name) == kINPUT` | buffers.h |
| `NvCaffeParser.h` | **已移除** (TRT 10.x 不再支持 Caffe) | sample_entrypoints.h |
| `NvUffParser.h` | **已移除** (TRT 10.x 不再支持 UFF) | sample_entrypoints.h |

### 新增辅助函数 (`common.h`)

```cpp
inline int32_t getIOTensorIndex(nvinfer1::ICudaEngine const& engine, char const* name) {
    for (int32_t i = 0; i < engine.getNbIOTensors(); i++)
        if (std::strcmp(engine.getIOTensorName(i), name) == 0) return i;
    return -1;
}
```

### 修改文件 (14)

| 文件 | 变更描述 |
|------|------|
| `tensorrtbuffer/include/buffers.h` | 完整重写：9 deprecated API → tensor-name API |
| `tensorrtbuffer/include/common.h` | 新增 `getIOTensorIndex()` |
| `tensorrtbuffer/include/sample_entrypoints.h` | 移除 Caffe/Uff parser 头文件 |
| `tensorrtbuffer/include/safe_common.h` | `roundUp` int64_t 类型转换 |
| `tensorrtbuffer/CMakeLists.txt` | TRT 10.9 include/lib 路径, 移除 sample_*.cpp |
| `AirSLAM/CMakeLists.txt` | TRT 路径, ament_target_dependencies, G2O PUBLIC, cudart, curl |
| `AirSLAM/package.xml` | 移除 rclpy, image_transport, tf2_geometry_msgs |
| `src/super_point.cpp` | `setBindingDimensions→setInputShape` |
| `src/super_glue.cpp` | 20 deprecated API→tensor API |
| `src/light_glue.cpp` | 12 deprecated API→tensor API |
| `src/plnet.cpp` | 18 deprecated API→tensor API |
| `include/ros_publisher.h` | `cv_bridge.hpp→cv_bridge.h` (ROS2 Humble) |

---

## 4. ONNX 模型兼容性修复

### 问题：TRT 10.9 严格类型检查

SuperPoint ONNX 中 `Shape` 节点输出 Int64，但下游 `Concat`/`Mul` 包含 Int32 输入，TRT 10.9 拒绝：

```
Error Code 4: (Concat_43: concat input tensors have incompatible types Int64 and Int32)
Error Code 4: (Mul_47: IElementWiseLayer has incompatible input types Int64 and Int32)
```

### 修复：插入 Cast(int64→int32)

每个 `Shape` 节点后插入 Cast 转换，并将所有下游引用重定向到 Cast 输出：

| 模型 | Shape 节点数 | 修复状态 |
|------|-------------|------|
| `superpoint_v1_sim_int32.onnx` (5.0MB) | 5 Cast 节点 | ✅ |
| `superpoint_lightglue.onnx` (44MB) | 0 (无问题) | ✅ |
| `plnet_s0.onnx` (32MB) | 0 (无问题) | ✅ |
| `plnet_s1.onnx` (1.8MB) | 0 (无问题) | ✅ |

---

## 5. 构建系统关键修复

| 问题 | 严重性 | 修复 |
|------|--------|------|
| `Boost_SERIALIZATION_LIBRARY` undefined | CRITICAL | `find_package(Boost COMPONENTS serialization)` |
| OpenCV/Eigen/CUDA 头文件不传递给可执行文件 | CRITICAL | include dirs 移到 PUBLIC |
| `G2O_INCLUDE_DIR` 未定义 | CRITICAL | `find_package(g2o)` (小写, ROS2 包名) |
| `${cv_bridge_LIBRARIES}` 在 ROS2 中为空 | CRITICAL | `ament_target_dependencies` |
| `install(EXPORT)` — tensorrtbuffer/DBoW2 未导出 | CRITICAL | 移除 EXPORT |
| `-lcudart` not found (CUDA 13.0) | CRITICAL | `link_directories(${CUDA_TOOLKIT_ROOT_DIR}/lib64)` |
| `libgdal` curl 符号缺失 | MEDIUM | 可执行文件链接 `-lcurl` |
| `rclpy`/`image_transport`/`tf2_geometry_msgs` 未使用 | MEDIUM | 从 package.xml + CMakeLists 移除 |
| ONNX 文件未安装 | MEDIUM | `install(DIRECTORY output/)` |

---

## 6. 运行时引擎和性能

### 预编译 TRT 引擎 (首次运行自动生成)

| 引擎 | 大小 | 用途 |
|------|------|------|
| `superpoint_v1_sim_int32.engine` | 4.1 MB | 点特征检测 |
| `superpoint_lightglue.engine` | 30 MB | 特征匹配 |
| `plnet_s0.engine` | 56 MB | 线特征 (Stage 0) |
| `plnet_s1.engine` | 1.1 MB | 线特征 (Stage 1) |
| **总计** | **~91 MB** | |

### EuRoC V1_01_easy 运行日志

```
SuperPoint: loaded cached engine  ← 引擎缓存加载
map_builder done                  ← 建图器初始化
dataset done                      ← 数据集加载
i=0-4:  2ms/帧   (特征提取)       ← 初始化
i=5:    433ms    (IMU 初始化!)     ← 首次关键帧
i=6:    32ms     (关键帧插入)
i=7+:   2-7ms/帧 (稳态 140-500FPS) ← 实时运行
Initialization is done!           ← 建图开始
```

---

## 7. 运行 EuRoC 数据集

### 环境设置

```bash
source /opt/ros/humble/setup.bash
source ~/rosws/airslame_ws/install/setup.bash
export LD_LIBRARY_PATH=~/lib/Deploy/TensorRT-10.9.0.34/lib:$LD_LIBRARY_PATH
```

### Launch 文件运行

```bash
ros2 launch air_slam vo_euroc.launch.py \
    dataroot:=/path/to/euroc/V1_01_easy/mav0
```

### 直接运行 (调试用)

```bash
ros2 run air_slam visual_odometry --ros-args \
  -p config_path:=.../share/air_slam/configs/visual_odometry/vo_euroc.yaml \
  -p dataroot:=/path/to/euroc/V1_01_easy/mav0 \
  -p camera_config_path:=.../share/air_slam/configs/camera/euroc.yaml \
  -p model_dir:=.../share/air_slam/output \
  -p saving_dir:=/tmp/airslam_debug
```

### 数据集目录结构

```
/path/to/euroc/V1_01_easy/
└── mav0/                ← dataroot 必须指向这里!
    ├── cam0/data/*.png  ← 左相机 752×480
    ├── cam1/data/*.png  ← 右相机 752×480
    ├── imu0/data.csv    ← IMU 200Hz
    └── state_groundtruth_estimate0/
```

---

## 8. 文件变更总清单 (14 files)

```
AirSLAM/
├── CMakeLists.txt                          # TRT 10.9, ament, G2O, cudart, curl
├── package.xml                             # format 3, 移除未使用 deps
├── docs/summary.md                         # 本文档 (新增)
├── launch/**/*.launch.py                   # 10 ROS2 launch (XML→Python)
├── include/
│   ├── ros_publisher.h                     # rclcpp, cv_bridge path
│   ├── thread_publisher.h                  # boost→lambda
│   ├── map_builder.h / map_refiner.h / map_user.h  # Node::SharedPtr
├── src/
│   ├── ros_publisher.cc / map_builder.cc / map_refiner.cc / map_user.cc  # ROS2 API
│   ├── super_point.cpp / super_glue.cpp    # TRT 10.x API
│   ├── light_glue.cpp / plnet.cpp          # TRT 10.x API
├── demo/
│   ├── visual_odometry.cpp / map_refinement.cpp  # rclcpp
│   └── relocalization.cpp / test_feature.cpp     # rclcpp
└── 3rdparty/tensorrtbuffer/
    ├── CMakeLists.txt                       # TRT include/lib paths
    └── include/
        ├── buffers.h                        # Full TRT 10.x rewrite
        ├── common.h                         # getIOTensorIndex()
        ├── sample_entrypoints.h             # Remove Caffe/UFF
        └── safe_common.h                    # Type fix
```

---

## 9. 构建命令

```bash
cd ~/rosws/airslame_ws
source /opt/ros/humble/setup.bash

# 增量构建
colcon build --packages-select air_slam --cmake-args -DCMAKE_BUILD_TYPE=Release

# 完全重构建 (CMakeLists.txt 变更后)
colcon build --packages-select air_slam --cmake-args -DCMAKE_BUILD_TYPE=Release --cmake-clean-first
```

---

## 10. 待办

| 优先级 | 任务 |
|--------|------|
| 🔴 | TRT 10.9 `LD_LIBRARY_PATH` 需永久配置 |
| 🟡 | 演示程序默认 config 路径硬编码 |
| 🟡 | `CMAKE_BUILD_TYPE` 防护条件 |
| 🟢 | 变量命名: `nh`→`node` |
| 🟢 | `rclcpp::Rate` 使用节点时钟 (支持仿真时间) |
| 🟢 | `add_compile_options(-w)`→`-Wall -Wextra` |
