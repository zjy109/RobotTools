# Hermes 相机—底盘外参标定工具

本目录提供一个统一的 OpenCV 交互界面，用于完成：

1. ChArUco 标定样本采集与基础解算；
2. RealSense 相机离地高度测量；
3. 使用确认后的相机高度进行约束重解算。

现有 ChArUco 检测、深度测高公式和两套最小二乘解算算法保持不变，代码仅进行了入口、配置、资源管理和文件组织上的整合。

## 文件结构

```text
calibration/
├── main.py                  # 唯一程序入口和 OpenCV 交互界面
├── config.py                # 底盘、相机、标定板、测高和解算配置
├── solver.py                # 基础解算与高度约束解算
├── samples.json             # 标定样本
└── README_calibration.md
```

程序运行后可能生成：

```text
T_base_camera_basic.yaml     # 无高度约束的基础解算结果
T_base_camera_refined.yaml   # 使用相机高度约束的最终解算结果
```

两个结果分别保存，互不覆盖。通常应使用 `T_base_camera_refined.yaml` 作为最终外参；基础结果主要用于诊断和对比。

## 环境依赖

需要 Python 3，并安装：

```bash
pip install numpy scipy PyYAML opencv-contrib-python pyrealsense2
```

注意必须使用包含 `cv2.aruco` 的 OpenCV 构建，通常对应 `opencv-contrib-python`。

硬件与网络要求：

- Intel RealSense D435i；
- Hermes / SLAMTEC 底盘；
- 主机能够访问底盘 Pose API；
- 与实际打印尺寸一致的 ChArUco 标定板。

## 启动

在 `calibration/` 目录运行：

```bash
python main.py
```

也可以从仓库根目录运行：

```bash
python calibration/main.py
```

所有默认输入输出路径都根据 `config.py` 所在目录解析，因此不依赖启动时的工作目录。

## 主菜单

程序启动后会显示三个模式：

```text
[1] ChArUco Calibration
[2] Camera Height Measurement
[3] Refine Calibration
[ESC/Q] Quit
```

可以按数字键选择，也可以用鼠标点击对应按钮。各模式中的 `B` 或 `ESC` 返回主菜单。

## 1. ChArUco 标定

该模式同时读取 RealSense 彩色图像与 Hermes 底盘位姿，并估计 `T_camera_board`。

快捷键：

| 按键 | 功能 |
|---|---|
| `SPACE` | 在标定板和底盘位姿都有效时采集样本 |
| `D` | 删除最后一个样本 |
| `S` | 保存到 `samples.json` |
| `L` | 从 `samples.json` 加载 |
| `ENTER` | 运行基础解算 |
| `R` | 清空当前内存中的样本 |
| `B` / `ESC` | 返回主菜单 |

每个样本包含：

```json
{
  "T_map_base": [[...], [...], [...], [...]],
  "T_camera_board": [[...], [...], [...], [...]],
  "time": 0.0
}
```

按 `ENTER` 后，结果写入：

```text
T_base_camera_basic.yaml
```

移动底盘主要产生平面运动，相机高度通常无法仅由基础解算可靠确定，因此基础结果不应直接替代高度约束结果。

## 2. 相机测高

该模式启用 RealSense 彩色和深度流，将 Depth 对齐到 Color，在设定 ROI 内按原有反投影公式计算高度，并使用中值和指数平滑显示结果。

快捷键：

| 按键 | 功能 |
|---|---|
| `C` | 选择当前平滑高度并进入确认状态 |
| `Y` | 将待确认高度写入 `config.py` |
| `N` | 取消本次确认 |
| `B` / `ESC` | 返回主菜单 |

推荐等待画面和高度读数稳定后再按 `C`。界面会显示：

```text
Write 0.347 m? Y/N
```

按 `Y` 后，程序会原子更新 `config.py` 中的：

```python
CAMERA_HEIGHT = 0.347000
```

同时更新当前进程内的配置，因此无需重启程序，返回主菜单后可以立即执行高度约束重解算。

## 3. 高度约束重解算

该模式自动：

1. 读取 `samples.json`；
2. 读取 `config.py` 中确认过的 `CAMERA_HEIGHT`；
3. 执行原有高度约束最小二乘解算；
4. 保存 `T_base_camera_refined.yaml`；
5. 在界面和终端显示结果。

结果界面显示样本数量、cost、相机高度约束、平移和欧拉角。按 `B`、`ESC` 或 `ENTER` 返回主菜单。

## 推荐操作流程

```text
配置底盘、相机和标定板参数
        ↓
模式 1：采集并保存 samples.json
        ↓
模式 2：测量高度，按 C 后按 Y 写入 config.py
        ↓
模式 3：执行高度约束重解算
        ↓
使用 T_base_camera_refined.yaml
```

基础解算不是执行模式 3 的前置条件；只要 `samples.json` 已保存，模式 3 就可以直接读取样本并求解。

## 配置说明

常用配置均位于 `config.py`。

### 底盘

```python
ROBOT_IP = "192.168.11.1"
POSE_HTTP_PORT = 1448
POSE_API = "/api/core/slam/v1/localization/pose"
```

### RealSense

```python
RGB_WIDTH = 640
RGB_HEIGHT = 480
DEPTH_WIDTH = 640
DEPTH_HEIGHT = 480
FPS = 30
```

### ChArUco

```python
ARUCO_DICT = "DICT_5X5_100"
SQUARES_X = 7
SQUARES_Y = 5
SQUARE_LENGTH = 0.05
MARKER_LENGTH = 0.035
MIN_CORNERS = 9
```

`SQUARE_LENGTH` 和 `MARKER_LENGTH` 的单位为米，必须与实际打印标定板一致。

### 测高

```python
ROI_X1_RATIO = 0.20
ROI_X2_RATIO = 0.80
ROI_Y1_RATIO = 0.60
ROI_Y2_RATIO = 0.95
TILT_ANGLE_DEG = 0.0
MIN_DEPTH = 0.20
MAX_DEPTH = 5.00
SMOOTH_ALPHA = 0.2
```

### 高度约束解算

```python
CAMERA_HEIGHT = 0.347000
BOARD_HEIGHT = None
W_CAMERA_Z = 1.0e4
W_BOARD_Z = 0.0
```

默认使用相机高度约束，不使用标定板高度约束。

## 坐标变换约定

矩阵名称采用 `T_A_B`，表示将 B 坐标系中的点变换到 A 坐标系：

```text
p_A = T_A_B @ p_B
```

标定关系为：

```text
T_map_board = T_map_base @ T_base_camera @ T_camera_board
```

最终输出的 `T_base_camera` 满足：

```text
p_base = T_base_camera @ p_camera
```

## 输出 YAML

基础结果包含：

```yaml
method: basic
T_base_camera: []
cost: 0.0
samples: 0
```

高度约束结果还包含：

```yaml
method: height_constrained
T_map_board: []
camera_height_measured: 0.347
camera_height_solved: 0.347
```

## 常见问题

### 无法启动 RealSense

确认相机没有被其他程序占用。模式切换时程序会先停止当前 pipeline，再启动下一模式所需的数据流。

### Board 一直显示 FAIL

检查：

- 是否安装了包含 ArUco 的 OpenCV；
- `ARUCO_DICT` 是否与标定板一致；
- 行列数和实际标定板是否一致；
- 方格与 Marker 尺寸是否正确；
- 光照、清晰度和可见角点数量是否满足 `MIN_CORNERS`。

### Robot 显示 FAIL

确认 `ROBOT_IP`、端口和 Pose API 地址正确，并检查主机是否能访问底盘网络。

### 无法确认相机高度

只有存在有效深度点并计算出高度后，`C` 才会进入确认状态。检查 ROI 是否覆盖地面，以及 `MIN_DEPTH`、`MAX_DEPTH` 和相机俯仰角配置。

### 重解算失败

确认 `samples.json` 存在且至少包含 5 个样本，并确认已经在测高界面保存正确的 `CAMERA_HEIGHT`。
