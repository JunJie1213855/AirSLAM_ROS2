# AirSLAM 运行与精度评估指南

> 适用环境：ROS2 Humble + TensorRT 10.9 + CUDA 13.0

---

## 1. 环境配置

```bash
source /opt/ros/humble/setup.bash
source ~/rosws/airslame_ws/install/setup.bash
export LD_LIBRARY_PATH=~/lib/Deploy/TensorRT-10.9.0.34/lib:$LD_LIBRARY_PATH
```

首次运行时 TensorRT 会将 ONNX 编译为 `.engine` 缓存（每个模型 2-5 分钟），后续启动毫秒级加载。

---

## 2. 运行视觉里程计

### 通过 launch 文件

```bash
ros2 launch air_slam vo_euroc.launch.py \
    dataroot:=/path/to/euroc/V1_01_easy/mav0 \
    saving_dir:=/path/to/output
```

`dataroot` 必须指向包含 `cam0/`、`cam1/`、`imu0/` 的目录（EuRoC 需含 `mav0/` 后缀）。

### 直接运行节点 (调试)

```bash
ros2 run air_slam visual_odometry --ros-args \
  -p config_path:=install/air_slam/share/air_slam/configs/visual_odometry/vo_euroc.yaml \
  -p dataroot:=/path/to/euroc/V1_01_easy/mav0 \
  -p camera_config_path:=install/air_slam/share/air_slam/configs/camera/euroc.yaml \
  -p model_dir:=install/air_slam/share/air_slam/output \
  -p saving_dir:=/tmp/output
```

### 参数说明

| 参数 | 默认值 | 说明 |
|------|------|------|
| `dataroot` | — | **必填**，数据集路径 |
| `config_path` | `configs/visual_odometry/vo_euroc.yaml` | 视觉里程计 YAML 配置 |
| `camera_config_path` | `configs/camera/euroc.yaml` | 相机内参 + IMU 噪声参数 |
| `model_dir` | `output/` | ONNX 模型和 TRT engine 目录 |
| `saving_dir` | `debug/` | 轨迹和地图保存目录 |

### 其他数据集

```bash
# EuRoC 暗光
ros2 launch air_slam vo_euroc_dark.launch.py dataroot:=/path/to/mav0

# OIVIO
ros2 launch air_slam vo_oivio.launch.py dataroot:=/path/to/oivio

# TartanAir
ros2 launch air_slam vo_tartanair.launch.py dataroot:=/path/to/tartanair

# UMA Bumblebee
ros2 launch air_slam vo_uma_bumblebee.launch.py dataroot:=/path/to/uma
```

### 输出文件

运行结束后 `saving_dir` 下生成：

```
saving_dir/
├── trajectory_v0.txt          # TUM 格式轨迹 (timestamp tx ty tz qx qy qz qw)
├── keyframe_trajectory.txt    # 关键帧轨迹
└── AirSLAM_mapv0.bin          # 地图文件 (二进制, boost::archive)
```

---

## 3. 精度评估

### 安装 evo

```bash
pip install evo
```

### Step 1 — 转换真值轨迹 (EuRoC → TUM)

EuRoC 真值在 `mav0/state_groundtruth_estimate0/data.csv`，格式为：

```
timestamp(ns), px, py, pz, qw, qx, qy, qz, ...
```

转换为 TUM 格式 (`t tx ty tz qx qy qz qw`)：

```bash
python3 -c "
import csv
with open('PATH/TO/EUROC/mav0/state_groundtruth_estimate0/data.csv') as f:
    reader = csv.reader(f)
    next(reader)
    for row in reader:
        t = int(row[0]) / 1e9
        print(f'{t:.9f} {row[1]} {row[2]} {row[3]} {row[5]} {row[6]} {row[7]} {row[4]}')
" > /tmp/gt_tum.txt
```

> 注意 EuRoC 四元数顺序是 `qw,qx,qy,qz`，TUM 格式需要 `qx,qy,qz,qw`。

### Step 2 — 绝对轨迹误差 (APE)

APE 衡量全局轨迹一致性：

```bash
evo_ape tum /tmp/gt_tum.txt /tmp/output/trajectory_v0.txt \
    -a --plot --plot_mode xyz --save_results /tmp/ape.zip
```

- `-a`：自动对齐轨迹（处理 SE(3) 变换，消除原点/朝向差异）
- `--plot`：生成可视化图
- `--plot_mode xyz`：3D 轨迹对比图
- 输出指标：**RMSE** (均方根误差), Mean, Median, Std, Min, Max

查看数值结果：

```bash
evo_res /tmp/ape.zip -p
```

### Step 3 — 相对位姿误差 (RPE)

RPE 衡量局部漂移（每单位距离的位姿变化误差）：

```bash
evo_rpe tum /tmp/gt_tum.txt /tmp/output/trajectory_v0.txt \
    -a --plot --delta_unit m --delta 1
```

- `--delta_unit m`：按距离间隔计算
- `--delta 1`：每 1 米评估一次

### 评估指标

| 指标 | 含义 |
|------|------|
| RMSE | 均方根误差 — 核心精度指标 |
| Mean | 平均误差 |
| Median | 中位数误差 — 排除异常值的典型精度 |
| Std | 标准差 — 误差波动程度 |
| Min / Max | 最小/最大误差 |

---

## 4. 批量评估脚本

脚本 `scripts/eval_euroc.sh` 可自动处理 EuRoC 序列：
```bash
bash scripts/eval_euroc.sh \
/path/to/trajectory_v0.txt \
/path/to/mav0 \
--plot --align
```

---

## 5. V1_02_medium 运行示例

完整流程：

```bash
# 1. 环境
source /opt/ros/humble/setup.bash
source ~/rosws/airslame_ws/install/setup.bash
export LD_LIBRARY_PATH=~/lib/Deploy/TensorRT-10.9.0.34/lib:$LD_LIBRARY_PATH

# 2. 运行
mkdir -p /tmp/airslam_v102
ros2 launch air_slam vo_euroc.launch.py \
    dataroot:=/home/ros/dataset/Euroc/vicon_room1/V1_02_medium/mav0 \
    saving_dir:=/tmp/airslam_v102

# 3. 转换真值
python3 -c "
import csv
with open('/home/ros/dataset/Euroc/vicon_room1/V1_02_medium/mav0/state_groundtruth_estimate0/data.csv') as f:
    for row in list(csv.reader(f))[1:]:
        t = int(row[0]) / 1e9
        print(f'{t:.9f} {row[1]} {row[2]} {row[3]} {row[5]} {row[6]} {row[7]} {row[4]}')
" > /tmp/airslam_v102/gt_tum.txt

# 4. APE 评估
evo_ape tum /tmp/airslam_v102/gt_tum.txt /tmp/airslam_v102/trajectory_v0.txt \
    -a --plot --plot_mode xyz --save_results /tmp/airslam_v102/ape.zip

# 5. 输出数值
evo_res /tmp/airslam_v102/ape.zip -p
```

---

## 6. 常见问题

| 问题 | 解决方案 |
|------|------|
| `trajectory_v0.txt` 不存在 | 程序可能未正常退出（crash/abort），轨迹在退出时才写入 |
| APE 非常大 (>10m) | 初始化可能失败，检查是否输出 "Initialization is done!" |
| imu0/data.csv 不存在 | `dataroot` 需包含 `mav0/` 子目录 |
| evo 报格式错误 | 确认真值顺序为 `t x y z qx qy qz qw` |
| map 地图点不显示 | RViz Fixed Frame 设为 `map` |

---

## 7. 参考

- [evo 文档](https://github.com/MichaelGrupp/evo)
- [EuRoC 数据集](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)
- [AirSLAM 迁移总结](summary.md)
