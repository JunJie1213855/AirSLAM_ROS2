# AirVO ROS 1 → ROS 2 移植记录

> 日期：2026-11-19
> 目标栈（与 `AirSLAM_ROS2` 一致）：Ubuntu 22.04 · ROS 2 Humble · ament_cmake ·
> CUDA 13.0 · TensorRT 10.9.0.34 · OpenCV 4.2+ · `ros-humble-libg2o`

原始 ROS 1 包保留在 `src/AirVO`（已加 `COLCON_IGNORE`，不参与 colcon 构建）。
本目录 `src/AirVO_ROS2` 为移植后的包，包名仍为 `air_vo`。

---

## 1. 变更总览

| 类别 | 文件 | 内容 |
|---|---|---|
| 构建系统 | `package.xml` | format 1 → 3，`catkin` → `ament_cmake`；依赖改为 `rclcpp` / `cv_bridge` / `message_filters` / 标准消息包；移除 `roscpp`、`rospy`、`image_transport` |
| 构建系统 | `CMakeLists.txt` | ament 构建；逐包 `find_package`；`g2o::*` 目标；`ament_target_dependencies`；install 规则；TensorRT 路径可覆盖 |
| ROS 发布层 | `include/ros_publisher.h`、`src/ros_publisher.cc` | `ros::NodeHandle/Publish` → `rclcpp::Node::SharedPtr` / `create_publisher`；消息加 `msg::` 命名空间；时间戳改为 `rclcpp::Time`；`GetCurrentTime()` 改用 `rclcpp::Clock` |
| 线程 | `include/thread_publisher.h`、`src/map_builder.cc` | `boost::bind` → lambda |
| 入口 | `demo/visual_odometry.cpp`（原 `main.cpp`） | `ros::init/param/ok` → `rclcpp::init` + `declare_parameter` |
| 入口 | `demo/visual_odometry_ros.cpp`（原 `ros_main.cpp`） | ROS 2 `message_filters::Synchronizer` + `ApproximateTime`；订阅使用 `rmw_qos_profile_sensor_data`；`cv_bridge` / 时间戳 API 更新 |
| 新增 | `demo/test_feature.cpp` | 单帧 SuperPoint/SuperGlue 匹配自检工具 |
| Launch | `launch/*.launch.py` | 5 个 ROS 1 XML → ROS 2 Python launch |
| RViz | `rviz/airvo.rviz` | 新建 RViz2 配置，话题 `/AirVO/*` |
| 模型 | `scripts/fix_onnx_shape_dtype.py` | 修复严格类型检查下的 ONNX int32/int64 混用 |
| 模型 | `output/*.onnx` | 见第 4 节 |
| 文档 | `README.md`、`docs/migration.md` | 更新 |

### ROS 1 → ROS 2 关键 API 映射

| ROS 1 | ROS 2 |
|---|---|
| `ros::NodeHandle` | `rclcpp::Node::SharedPtr` |
| `nh.advertise<T>(topic, q)` | `node->create_publisher<T>(topic, rclcpp::QoS(q))` |
| `ros::param::get("~x", v)` | `declare_parameter<T>("x", default)` + `get_parameter` |
| `ros::Time::now()/fromSec()` | `node->now()` / `rclcpp::Time(ns)` |
| `ros::spin()` | `rclcpp::spin(node)` |
| `boost::bind` | lambda / `std::function` |
| `sensor_msgs::Image` | `sensor_msgs::msg::Image` |
| `geometry_msgs::PoseStamped` | `geometry_msgs::msg::PoseStamped` |
| `cv_bridge::CvImage(..., std_msgs::Header(), ...)` | `std_msgs::msg::Header()` |
| XML `<node><param>` | Python `LaunchDescription` + `parameters` |

---

## 2. TensorRT 8.4 → 10.9 API 迁移

`3rdparty/tensorrtbuffer` 直接采用 `AirSLAM_ROS2` 中已迁移到 TRT 10.x
tensor-name API 的版本（原 `Thirdparty/TensorRTBuffer` 已移除）。其命名空间为
`tensorrt_buffer`，`TensorRTUniquePtr` 亦在该命名空间下。

推理封装中的 36 处旧 binding API 已替换：

| TRT 8.x | TRT 10.x |
|---|---|
| `engine->getNbBindings()` | `engine->getNbIOTensors()` |
| `engine->getBindingIndex(name)` | 不再需要，直接使用 tensor 名字 |
| `context->setBindingDimensions(i, dims)` | `context->setInputShape(name, dims)` |
| `context->getBindingDimensions(i)` | `context->getTensorShape(name)` |
| `config->setMaxWorkspaceSize(...)` | 移除（TRT 10 起自动管理） |
| `NetworkDefinitionCreationFlag::kEXPLICIT_BATCH` | TRT 11 移除；10.9 保留，代码用 `#if` 兼容 |

`IExecutionContext::executeV2()` 在 TensorRT 11.3 中仍然存在，因此执行路径未改写为
`enqueueV3`。

### TensorRT 11.x 兼容（可选）

本机没有 TRT 10.9，验证时使用了 TRT 11.3。为同时支持两者，代码加入版本保护：

- `#if NV_TENSORRT_MAJOR < 11` 内保留 `BuilderFlag::kFP16`；
  TRT 11 移除了该 flag（强类型网络），引擎将以 FP32/TF32 构建。
- `common.h` 中已移除的 INT8 校准 / DLA 动态范围示例工具用 `#if` 隔离；
  `buffers.h` 中 `hasImplicitBatchDimension()` 同样加保护。
- TRT 11 不再提供 `kEXPLICIT_BATCH`，使用 `createNetworkV2(0U)`。

在 TRT 10.9 上这些保护不会改变原有行为（FP16 仍启用）。

---

## 3. 线程与退出修复

`MapBuilder::ShutDown()` 原来只 `join()` 自己的两个工作线程，未停止
`RosPublisher` 内部的 5 个发布线程。ROS 1 下进程直接退出掩盖了该问题；
ROS 2 下会触发 `terminate called without an active exception`。现已在该函数中
调用 `_ros_publisher->ShutDown()`，离线/在线节点均可干净退出。

---

## 4. ONNX 模型处理

TensorRT 10.9/11 对 ONNX 类型检查更严格，原模型中的 `Concat` 节点混用
Int32 常量与 Int64 形状张量，会直接解析失败。

| 模型 | 处理 |
|---|---|
| `superpoint_v1_sim_int32.onnx` | 采用 `AirSLAM_ROS2/output` 中已做 Cast 修复的版本（网络权重相同，仅图结构修正）。验证 400 个关键点。 |
| `superglue_indoor_sim_int32.onnx` | `scripts/fix_onnx_shape_dtype.py` 将 94 个 int32 形状常量转为 int64（ONNX 规范要求的形状类型），解析通过。 |
| `superglue_outdoor_sim_int32.onnx` | 同上。 |

`scripts/fix_onnx_shape_dtype.py` 保留在包内，便于从原始模型重新生成。

---

## 5. 构建与验证结果

构建（本机 TRT 11.3 / CUDA 12.1，覆盖默认的 10.9 路径）：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select air_vo \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DTENSORRT_DIR=$HOME/lib/TensorRT-11.3.0.99
```

| 检查项 | 结果 |
|---|---|
| colcon 构建（TRT 11.3 头文件/库） | ✅ 通过 |
| SuperPoint 引擎构建 + 推理 | ✅ 400 个关键点 |
| SuperGlue 引擎构建 + 推理 | ✅ 相同特征输入：374 匹配，其中 366 对索引一致 |
| 离线流程（20 帧合成双目） | ✅ 完整跑完并干净退出，约 100–120 ms/帧（FP32，RTX 4060 Ti） |
| 初始化 / 建图精度 | ⚠️ 未验证：合成数据不是有效标定双目对，初始化条件不满足；本机无 EuRoC 数据 |
| TRT 10.9 实机运行 | ❌ 未验证（本机无 10.9） |
| 在线节点启动 / 订阅 | ✅ 已验证：`/cam0/image_raw`、`/cam1/image_raw` 订阅建立，发布 `/AirVO/*` |
| 完整 bag 回放（EuRoC V1_03_difficult） | ⚠️ 实测有输出；精度未评估（bag 无真值，仅有 `/vicon/firefly_sbx/firefly_sbx`） |

`test_feature` 自检（使用单张图像、两次相同特征）：

```bash
ros2 run air_vo test_feature --ros-args \
  -p config_path:=install/air_vo/share/air_vo/configs/configs_euroc.yaml \
  -p model_dir:=install/air_vo/share/air_vo/output \
  -p image0:=/path/to/image.png
# image0 features = 400
# matches = 374
# diagonal matches = 366
```

---

## 6. 已知问题与注意事项

1. **TensorRT 版本**：目标版本是 10.9；本机为 11.3。11.3 下无 FP16，帧率低于论文
   数据（论文为 TRT 8 + FP16）。
2. **模型来源**：SuperPoint 模型取自 `AirSLAM_ROS2`（权重一致，图结构针对严格类型
   检查做过 Cast 修复）。SuperGlue 模型由本包脚本从原模型转换。
3. **许可证**：原 AirVO 为 BSD-3-Clause；`AirSLAM_ROS2` 为 GPLv3。复用的
   `tensorrtbuffer` 头文件源自 NVIDIA TensorRT samples（Apache-2.0），但修改版本
   随 GPLv3 仓库分发。若用于再发布，建议核对来源与许可证义务。
4. **无自动化测试**：目前仅有 `test_feature` 自检；精度回归需在 EuRoC/OIVIO/UMA 上
   用 evo 完成。
5. **默认路径**：launch 中的 `dataroot` 仍保留上游示例路径，运行时应显式传入。

---

## 7. 后续建议

1. 在 TRT 10.9 + CUDA 13.0 环境重新构建并跑 EuRoC V1_01_easy，记录 APE/RPE 与帧率。
2. 用 rosbag2 回放 `/cam0/image_raw`、`/cam1/image_raw` 验证在线模式与 QoS。
3. 如需在 TRT 11.x 上提升性能，可评估将 ONNX 权重转 FP16 后重新构建引擎。
4. 补充初始化/跟踪失败的回归用例。
