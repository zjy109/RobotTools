# RobotTools

`RobotTools` 是面向 Hermes / SLAMTEC 移动底盘与 Intel RealSense D435i 的机器人视觉工具集合，目前包含相机—底盘外参标定和 RGB-D + 底盘位姿数据发布两部分。

## 项目结构

```text
RobotTools/
├── calibration/       # 相机—底盘外参标定与相机测高
└── data_publisher/    # RGB-D、底盘位姿同步、发布、录制与回放
```

### Calibration

提供统一的 OpenCV 交互界面，支持：

- ChArUco 标定样本采集；
- 相机离地高度测量与配置保存；
- 基础外参解算；
- 使用相机高度约束的外参重解算。

启动：

```bash
python calibration/main.py
```

配置、操作方法和输出格式参见 [Calibration 使用说明](calibration/README_calibration.md)。

### Data Publisher

用于采集 RealSense RGB-D 数据和 Hermes 底盘位姿，在主机时间轴上完成同步，并通过 ZeroMQ 发布统一的 `rgbd.pose` 数据流。同时提供：

- Rerun 实时可视化；
- 数据流无损录制；
- 已录制数据集回放；
- 本机 IPC 或跨机器 TCP 传输。

实时发布：

```bash
python data_publisher/rgbd_pose_publisher.py
```

协议、配置、录制、回放和可视化说明参见 [Data Publisher 使用说明](data_publisher/README_data.md)。

## 推荐使用流程

```text
Calibration 标定相机—底盘外参
                ↓
将 T_base_camera 配置到数据发布端
                ↓
Data Publisher 发布同步后的 RGB-D + Pose
                ↓
可视化、录制或提供给下游建图与感知模块
```

首次使用时，建议先完成标定并确认 `T_base_camera` 的坐标方向，再启动数据发布端。具体依赖、硬件要求和参数配置分别以两个子目录中的 README 为准。

## 硬件与运行环境

当前代码主要面向：

- Linux；
- Python 3；
- Intel RealSense D435i；
- Hermes / SLAMTEC 底盘；
- 支持 OpenCV GUI 的桌面环境。

各模块依赖不同，请不要仅依据本页安装环境，具体安装要求参见对应模块文档。
