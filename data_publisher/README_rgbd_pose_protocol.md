# RGB-D + 底盘位姿数据中转协议与实现说明

## 1. 项目简介

本项目用于把 **Intel RealSense RGB-D 相机数据** 与 **商用底盘（Hermes / SLAMTEC）提供的 2D/3D 位姿数据** 在主机侧进行时间同步，并通过 **ZeroMQ（ZMQ）PUB/SUB** 方式统一发布。

整体链路如下：

```text
        ┌────────────────────┐
        │ RealSense D435i    │
        │ RGB + Depth        │
        └─────────┬──────────┘
                  │
                  │ 相机时间戳（主时间轴）
                  ▼
        ┌────────────────────┐
        │ rgbd_pose_publisher│
        │                    │
        │ 1. 读取 RGB-D      │
        │ 2. 轮询底盘 pose   │
        │ 3. 位姿时间插值    │
        │ 4. 外参变换        │
        │ 5. ZMQ 发布        │
        └─────────┬──────────┘
                  │
                  │ ZMQ multipart
                  │ [topic, meta, rgb, depth]
                  ▼
        ipc:///tmp/rgbd_pose.ipc
                  │
                  ▼
        ┌────────────────────┐
        │ vis_receiver_      │
        │ rgbd_pose.py       │
        │                    │
        │ 1. ZMQ 订阅        │
        │ 2. 协议解码        │
        │ 3. Rerun 可视化    │
        └────────────────────┘
```

这里的核心不是单纯转发图像，而是把每一帧 RGB-D 数据与对应时刻的底盘位姿绑定在一起，从而得到：

- RGB 图像；
- 深度图；
- 相机采集时间戳；
- 同步后的底盘位姿 `T_map_base`；
- 由相机外参计算得到的相机全局位姿 `T_map_camera`；
- 相机内参；
- 时间同步质量指标；
- 发布端运行统计信息。

因此，任何订阅该 ZMQ 协议的程序都可以在不直接访问 RealSense 和底盘 REST API 的情况下，获得已经同步好的 **RGB-D + 6DoF 相机位姿观测**。

---

# 2. 数据中转协议

## 2.1 传输方式

发布端使用 ZeroMQ `PUB`，接收端使用 `SUB`。

默认配置：

```python
ZMQ_ENDPOINT = "ipc:///tmp/rgbd_pose.ipc"
ZMQ_TOPIC = "rgbd.pose"
```

即默认采用本机 IPC：

```text
ipc:///tmp/rgbd_pose.ipc
```

消息主题为：

```text
rgbd.pose
```

发布端：

```python
sock = ctx.socket(zmq.PUB)
sock.bind(ZMQ_ENDPOINT)
```

接收端：

```python
sock = ctx.socket(zmq.SUB)
sock.connect(ZMQ_ENDPOINT)
sock.setsockopt(zmq.SUBSCRIBE, ZMQ_TOPIC.encode("utf-8"))
```

> 当前配置主要面向同一台 Linux 主机上的进程间通信。  
> 如果以后需要跨机器传输，可以把 endpoint 改为 `tcp://IP:PORT`，协议本身不需要改变。

### 跨机传输示例

如果发布端和接收端运行在不同机器上，可以将 IPC 改为 TCP。

假设：

```text
发布端 IP：192.168.1.100
接收端 IP：192.168.1.101
ZMQ 端口：5555
```

发布端 `rgbd_pose_publisher.py`：

```python
ZMQ_ENDPOINT = "tcp://0.0.0.0:5555"
ZMQ_TOPIC = "rgbd.pose"
```

其中：

```text
tcp://0.0.0.0:5555
```

表示发布端在本机所有网卡上监听 `5555` 端口。

接收端 `vis_receiver_rgbd_pose.py`：

```python
ZMQ_ENDPOINT = "tcp://192.168.1.100:5555"
ZMQ_TOPIC = "rgbd.pose"
```

这里接收端需要填写**发布端机器的实际 IP 地址**。

对应的数据链路为：

```text
发布端
192.168.1.100
rgbd_pose_publisher.py
        │
        │ tcp://0.0.0.0:5555
        ▼
     局域网 TCP
        │
        ▼
接收端
192.168.1.101
vis_receiver_rgbd_pose.py
```

启动前可先在接收端确认网络连通：

```bash
ping 192.168.1.100
```

如果发布端启用了 Ubuntu 防火墙，还需要开放对应 TCP 端口：

```bash
sudo ufw allow 5555/tcp
```

跨机传输只需要修改 `ZMQ_ENDPOINT`，消息主题、multipart 消息结构以及 metadata 协议均不需要改变。

---

## 2.2 ZMQ 消息结构

每一帧数据由一个 **4-part multipart message** 构成：

```text
Part 0: topic
Part 1: metadata (MessagePack)
Part 2: RGB raw bytes
Part 3: Depth raw bytes
```

即：

```python
[
    topic_bytes,
    meta_bytes,
    rgb.tobytes(),
    depth_raw.tobytes(),
]
```

对应协议形式：

```text
┌───────────────┐
│ Part 0        │
│ ZMQ Topic     │
│ "rgbd.pose"   │
└───────────────┘
        +
┌───────────────┐
│ Part 1        │
│ MessagePack   │
│ metadata      │
└───────────────┘
        +
┌───────────────┐
│ Part 2        │
│ RGB raw bytes │
└───────────────┘
        +
┌───────────────┐
│ Part 3        │
│ Depth bytes   │
└───────────────┘
```

接收端必须确认：

```python
len(parts) == 4
```

否则该消息应视为异常帧。

---

# 3. Metadata 协议

Metadata 使用 `msgpack` 编码：

```python
meta_bytes = msgpack.packb(meta, use_bin_type=True)
```

接收端：

```python
meta = msgpack.unpackb(meta_bytes, raw=False)
```

当前协议版本：

```text
version = 1
type = "rgbd_pose"
```

---

## 3.1 顶层字段

| 字段 | 类型 | 说明 |
|---|---|---|
| `version` | int | 协议版本，目前为 `1` |
| `type` | string | 数据类型，目前固定为 `rgbd_pose` |
| `frame_id` | int | 发布帧序号 |
| `timestamp_ns` | int | 当前观测主时间戳，Unix ns |
| `timestamp_source` | string | 时间戳来源 |
| `width` | int | RGB/对齐后深度图宽度 |
| `height` | int | RGB/对齐后深度图高度 |
| `rgb_dtype` | string | RGB 数组数据类型，通常 `uint8` |
| `depth_dtype` | string | 深度数组数据类型，通常 `uint16` |
| `depth_scale_m` | float | 深度 raw value 到米的比例 |
| `pose_valid` | bool | 当前帧是否存在有效同步位姿 |
| `T_map_base` | 4×4 list / null | 底盘在 map 坐标系下的位姿 |
| `T_map_camera` | 4×4 list / null | 相机在 map 坐标系下的位姿 |
| `sync` | dict | 时间同步诊断信息 |
| `publisher_stats` | dict | 发布端统计信息 |
| `color_intr` | dict | RGB 相机内参 |
| `depth_intr` | dict | Depth 相机内参 |
| `camera_debug` | dict，可选 | 相机时间戳与处理延迟调试信息 |

---

## 3.2 图像数据

### RGB

发布端 RealSense 原始彩色流配置为：

```text
format = BGR8
```

在发布前转换为：

```python
rgb = bgr[:, :, ::-1]
```

因此协议中的 RGB 数据实际为：

```text
RGB
shape = (height, width, 3)
dtype = meta["rgb_dtype"]
```

接收端恢复：

```python
rgb = np.frombuffer(
    rgb_bytes,
    dtype=np.dtype(meta.get("rgb_dtype", "uint8")),
).reshape(height, width, 3)
```

---

### Depth

深度图采用 RealSense 原始深度值：

```text
shape = (height, width)
dtype = meta["depth_dtype"]
```

通常为：

```text
uint16
```

恢复方式：

```python
depth = np.frombuffer(
    depth_bytes,
    dtype=np.dtype(meta.get("depth_dtype", "uint16")),
).reshape(height, width)
```

实际距离：

```text
distance_meter = depth_raw * depth_scale_m
```

例如：

```text
depth_raw = 1250
depth_scale_m = 0.001
```

则：

```text
distance = 1.25 m
```

当前发布端默认将 Depth 对齐到 RGB：

```python
ALIGN_DEPTH_TO_COLOR = True
```

因此默认情况下 RGB 和 Depth 具有相同图像尺寸，可直接按像素对应。

---

# 4. 位姿与坐标变换协议

## 4.1 变换矩阵约定

所有位姿统一采用齐次变换矩阵：

```text
T_A_B
```

含义为：

```text
把 B 坐标系中的点变换到 A 坐标系
```

即：

```text
p_A = T_A_B @ p_B
```

矩阵形式：

```text
T = [ R  t ]
    [ 0  1 ]
```

其中：

- `R`：3×3 旋转矩阵；
- `t`：3×1 平移向量。

---

## 4.2 `T_base_camera`

相机相对于底盘的固定外参为：

```text
T_base_camera
```

定义：

```text
p_base = T_base_camera @ p_camera
```

该矩阵由相机—底盘外参标定获得。

发布端支持两种方式配置：

### 方式 A：代码内直接填写

```python
T_BASE_CAMERA_CONFIG = [
    [...],
    [...],
    [...],
    [0.0, 0.0, 0.0, 1.0],
]
```

### 方式 B：从文件读取

支持：

```text
.npy
.json
```

JSON 可以是：

```json
[
  [1, 0, 0, 0],
  [0, 1, 0, 0],
  [0, 0, 1, 0],
  [0, 0, 0, 1]
]
```

或者：

```json
{
  "T_base_camera": [[...], [...], [...], [...]]
}
```

也支持键名：

```text
T
matrix
```

如果文件中保存的是反向外参 `T_camera_base`，可以设置：

```python
EXTRINSIC_FILE_IS_T_BASE_CAMERA = False
```

发布端会自动求逆。

---

## 4.3 `T_map_base`

`T_map_base` 表示底盘在导航地图坐标系中的位姿：

```text
p_map = T_map_base @ p_base
```

该数据由底盘 `/pose` API 返回的：

```text
x
y
z
roll
pitch
yaw
```

构造得到。

旋转采用：

```python
Rotation.from_euler("xyz", [roll, pitch, yaw])
```

---

## 4.4 `T_map_camera`

相机全局位姿通过：

```text
T_map_camera = T_map_base @ T_base_camera
```

得到。

因此：

```text
p_map
= T_map_base @ p_base
= T_map_base @ T_base_camera @ p_camera
= T_map_camera @ p_camera
```

对于后续：

- SLAM；
- 点云拼接；
- TSDF；
- Occupancy / Voxel Map；
- RGB-D 全局重建；

通常可以直接使用 `T_map_camera`。

---

# 5. 时间同步协议

这是整个中转程序中最关键的部分之一。

## 5.1 为什么需要时间同步

RealSense RGB-D 数据与底盘位姿来自两个独立数据源：

```text
RealSense
    └── camera timestamp

Hermes REST API
    └── HTTP 请求返回 pose
```

底盘 pose REST API 本身没有提供严格的传感器测量时间戳，因此不能简单地认为：

```text
收到 pose 的时间 = pose 对应的真实时间
```

当前实现使用 HTTP 请求发送和响应之间的时间中点作为该 pose 的估计时间。

---

## 5.2 相机时间

RGB 帧时间戳作为整个观测的主时间：

```text
timestamp_ns = RGB capture timestamp
```

程序要求 RealSense 时间戳属于：

```text
GLOBAL_TIME
```

或者：

```text
SYSTEM_TIME
```

从而可以与主机 Unix 时间轴上的底盘 pose 时间进行比较。

当前：

```python
REQUIRE_GLOBAL_OR_SYSTEM_TIME = True
```

如果 RGB 时间戳不在主机时间轴上，该帧默认被丢弃。

---

## 5.3 底盘 pose 时间估计

一次 HTTP 请求记为：

```text
t_request
t_response
```

pose 的时间戳估计为：

```text
t_pose = (t_request + t_response) / 2
```

与此同时，通过 monotonic clock 测量 HTTP RTT：

```text
RTT = response_mono - request_mono
```

使用 midpoint 的目的，是减小单纯使用“请求开始时间”或者“响应到达时间”造成的单边网络延迟误差。

需要注意：

> 这仍然只是时间估计，因为底盘 REST API 并未提供 pose 的原始测量时间。

因此协议中保留 RTT 和同步前后时间差，用于判断同步质量。

---

## 5.4 位姿时间插值

对于一帧相机图像，时间戳记为：

```text
tc
```

从 Pose Buffer 中找到：

```text
P0.t <= tc <= P1.t
```

计算：

```text
alpha = (tc - P0.t) / (P1.t - P0.t)
```

平移采用线性插值：

```text
p(tc) = (1-alpha) * p0 + alpha * p1
```

旋转采用 quaternion SLERP：

```text
R(tc) = SLERP(R0, R1, alpha)
```

最终得到与相机时刻同步的：

```text
T_map_base(tc)
```

再计算：

```text
T_map_camera(tc)
```

---

## 5.5 同步质量判断

当前代码使用以下约束。

### Pose bracket 等待时间

```python
POSE_SYNC_WAIT_TIMEOUT_S = 0.12
```

即允许相机帧等待最多约 120 ms，让未来的一帧 pose 到达，以构成：

```text
P0 <= camera <= P1
```

---

### 单侧最大时间差

```python
MAX_POSE_SIDE_DT_MS = 100.0
```

要求：

```text
camera_time - P0.time <= 100 ms
P1.time - camera_time <= 100 ms
```

如果 pose 间隔过大，则同步结果无效。

---

### HTTP RTT 限制

```python
MAX_POSE_RTT_MS = 40.0
```

如果用于插值的任意一个 pose：

```text
RTT > 40 ms
```

则当前同步结果被拒绝。

---

## 5.6 `sync` 字段

Metadata：

```python
"sync": {
    "valid": ...,
    "reason": ...,
    "method": "linear_translation+slerp_rotation",
    "dt_before_ms": ...,
    "dt_after_ms": ...,
    "rtt_before_ms": ...,
    "rtt_after_ms": ...
}
```

字段含义：

| 字段 | 说明 |
|---|---|
| `valid` | 时间同步是否有效 |
| `reason` | 同步失败或成功原因 |
| `method` | 插值方法 |
| `dt_before_ms` | 相机时间距离前一个 pose 的时间 |
| `dt_after_ms` | 后一个 pose 距离相机时间的时间 |
| `rtt_before_ms` | 前一个 pose HTTP 请求 RTT |
| `rtt_after_ms` | 后一个 pose HTTP 请求 RTT |

可能出现的 `reason`：

```text
ok
no_pose_bracket
invalid_pose_time_order
camera_not_bracketed
pose_gap_too_large
pose_rtt_too_large
```

另外发布端还统计：

```text
bad_camera_clock
other_sync_error
```

---

# 6. 发布统计信息

Metadata 中包含：

```python
"publisher_stats": {
    "published": ...,
    "skipped_unsynced": ...,
    "skip_reasons": {...},
    "zmq_drop": ...,
    "pose_api_ok": ...,
    "pose_api_err": ...
}
```

其中：

| 字段 | 说明 |
|---|---|
| `published` | 已成功发布帧数 |
| `skipped_unsynced` | 因时间同步问题跳过的帧数 |
| `skip_reasons` | 各种同步失败原因统计 |
| `zmq_drop` | ZMQ 非阻塞发送失败次数 |
| `pose_api_ok` | 底盘 REST pose 获取成功次数 |
| `pose_api_err` | 底盘 REST pose 获取失败次数 |

这些统计信息主要用于判断系统运行时：

- pose API 是否稳定；
- 网络 RTT 是否过大；
- 相机时间域是否正确；
- pose 频率是否足够；
- ZMQ 消费端是否跟不上。

---

# 7. 相机内参协议

RGB 与 Depth 内参格式相同：

```python
{
    "fx": ...,
    "fy": ...,
    "ppx": ...,
    "ppy": ...,
    "model": ...,
    "coeffs": [...]
}
```

例如：

```python
fx = meta["color_intr"]["fx"]
fy = meta["color_intr"]["fy"]
cx = meta["color_intr"]["ppx"]
cy = meta["color_intr"]["ppy"]
```

对应相机矩阵：

```text
K = [ fx   0   cx ]
    [  0  fy   cy ]
    [  0   0    1 ]
```

---

# 8. 使用方式

## 8.1 Python 依赖

发布端主要依赖：

```bash
pip install numpy msgpack pyzmq scipy pyrealsense2
```

接收与可视化端还需要：

```bash
pip install rerun-sdk
```

当前可视化脚本明确针对：

```text
rerun-sdk == 0.31.4
numpy      == 1.26.4
```

如果使用其他版本，尤其是 Rerun 大版本变化后，Blueprint API 可能存在兼容性差异。

---

## 8.2 配置底盘 IP

修改：

```python
ROBOT_IP = "192.168.11.1"
```

底盘 pose API：

```text
http://<ROBOT_IP>:1448/api/core/slam/v1/localization/pose
```

---

## 8.3 配置相机外参

建议优先使用外部文件：

```python
EXTRINSIC_FILE = "/path/to/T_base_camera.npy"
```

或者：

```python
EXTRINSIC_FILE = "/path/to/T_base_camera.json"
```

也可以直接修改：

```python
T_BASE_CAMERA_CONFIG
```

必须确认该矩阵的定义是：

```text
T_base_camera
```

也就是：

```text
camera frame -> base frame
```

---

## 8.4 启动发布端

```bash
python rgbd_pose_publisher.py
```

正常情况下可看到：

```text
[extrinsic] T_base_camera =
...

[pose] polling http://192.168.11.1:1448/api/core/slam/v1/localization/pose ...

[zmq] bound ipc:///tmp/rgbd_pose.ipc topic=rgbd.pose

[camera] device=...
[camera] serial=...
```

随后周期性打印：

```text
[pub] frame=...
fps=...
pose_ok=True
dt=(...)
rtt=(...)
pose_api_ok=...
skip=...
zmq_drop=...
```

---

## 8.5 启动可视化接收端

另开一个终端：

```bash
python vis_receiver_rgbd_pose.py
```

正常情况下：

```text
[zmq] connected ipc:///tmp/rgbd_pose.ipc topic=rgbd.pose
[recv] Ctrl+C to stop
```

脚本会启动 Rerun：

```text
gRPC: 9876
Web : 9090
```

本机可以通过浏览器访问：

```text
http://127.0.0.1:9090
```

---

## 8.6 VSCode Remote SSH

如果程序运行在远程 Ubuntu 主机，而浏览器在本地电脑，需要转发：

```text
9090
9876
```

即：

```text
remote 9090 -> local 9090
remote 9876 -> local 9876
```

然后使用脚本启动时打印出的 Rerun Web URL。

---

# 9. 自己实现一个最小接收端

如果不需要 Rerun，可以只按照协议接收数据。

示例：

```python
import msgpack
import numpy as np
import zmq

endpoint = "ipc:///tmp/rgbd_pose.ipc"
topic = "rgbd.pose"

ctx = zmq.Context()
sock = ctx.socket(zmq.SUB)
sock.connect(endpoint)
sock.setsockopt(zmq.SUBSCRIBE, topic.encode("utf-8"))

while True:
    parts = sock.recv_multipart()

    if len(parts) != 4:
        continue

    topic_bytes, meta_bytes, rgb_bytes, depth_bytes = parts

    meta = msgpack.unpackb(meta_bytes, raw=False)

    h = int(meta["height"])
    w = int(meta["width"])

    rgb = np.frombuffer(
        rgb_bytes,
        dtype=np.dtype(meta.get("rgb_dtype", "uint8")),
    ).reshape(h, w, 3)

    depth = np.frombuffer(
        depth_bytes,
        dtype=np.dtype(meta.get("depth_dtype", "uint16")),
    ).reshape(h, w)

    if meta["pose_valid"]:
        T_map_camera = np.asarray(
            meta["T_map_camera"],
            dtype=np.float64,
        )

        print(
            meta["frame_id"],
            T_map_camera[:3, 3],
        )
```

---

# 10. 发布端实现方式

下面介绍 `rgbd_pose_publisher.py` 的内部实现逻辑。

## 10.1 双线程结构

程序实际上包含两个数据获取流程。

### 主线程

负责：

```text
RealSense RGB-D
    ↓
获取相机时间戳
    ↓
查询 PoseBuffer
    ↓
时间插值
    ↓
外参变换
    ↓
打包协议
    ↓
ZMQ Publish
```

### Pose Reader 子线程

负责：

```text
Hermes /pose REST API
    ↓
记录 request time
    ↓
发送 HTTP
    ↓
收到 pose
    ↓
记录 response time
    ↓
计算 midpoint timestamp + RTT
    ↓
PoseBuffer
```

这样做的原因是：

> 如果在每一帧 RealSense 图像到达后才同步调用一次 HTTP pose API，相机采集线程会直接受到 REST API RTT 影响。

独立轮询线程可以持续维护最近一段时间的 pose 序列，使图像线程只需要做时间查询和插值。

---

## 10.2 PoseBuffer

PoseBuffer 是一个线程安全的有限长度队列：

```python
deque(maxlen=POSE_BUFFER_SIZE)
```

默认：

```python
POSE_BUFFER_SIZE = 300
```

若 pose 约 30 Hz，则大约保存：

```text
300 / 30 ≈ 10 s
```

的历史数据。

内部使用：

```python
threading.Condition()
```

因此相机线程在等待 `P1` 到达时，可以被新 pose 主动唤醒，而不是纯轮询等待。

---

## 10.3 为什么要等待未来 Pose

对于相机时间：

```text
tc
```

插值必须同时存在：

```text
P0.t <= tc <= P1.t
```

当相机帧刚刚到达时：

```text
P0
camera
```

通常已经有 `P0`，但 `P1` 可能还没有通过 HTTP 返回。

因此：

```python
wait_for_bracket()
```

允许最多等待：

```text
POSE_SYNC_WAIT_TIMEOUT_S
```

等待下一条 pose 到达。

这会带来少量系统延迟，但可以避免使用外推，提高位姿同步稳定性。

---

## 10.4 深度对齐

程序先获取原始 RGB / Depth 时间戳：

```python
raw_color.get_timestamp()
raw_depth.get_timestamp()
```

之后才执行：

```python
aligner.process(frames)
```

这是一个重要实现细节。

因为用于时间同步的是原始相机帧 timestamp，而不是经过 `rs.align()` 之后再取时间信息。

对齐后：

```text
Depth -> RGB optical geometry
```

使最终发布的 depth 与 RGB 像素对应。

---

## 10.5 非阻塞 ZMQ

发布端：

```python
sock.send_multipart(..., flags=zmq.NOBLOCK)
```

并设置：

```python
ZMQ_SNDHWM = 2
```

设计目标是：

> 下游消费者跟不上时，宁愿丢掉旧帧，也不要让 RealSense 采集和同步主线程被阻塞。

因此该协议更偏向：

```text
实时流
```

而不是：

```text
可靠消息队列 / 每帧必达
```

这点对于 SLAM 和机器人在线感知通常是合理的。

如果 ZMQ 队列满，会增加：

```text
zmq_drop
```

计数。

---

# 11. 接收端实现方式

`vis_receiver_rgbd_pose.py` 是当前协议的一个参考消费者，同时也是运行状态可视化工具。

## 11.1 协议解码

接收端首先要求：

```text
multipart parts == 4
```

然后分别解析：

```text
meta
RGB
Depth
```

图像数组完全根据 Metadata 中：

```text
width
height
rgb_dtype
depth_dtype
```

恢复。

因此接收端没有把：

```text
640 × 480
uint8
uint16
```

硬编码进解码逻辑。

---

## 11.2 Rerun 可视化

接收端使用 Rerun 展示四类信息：

### RGB

```text
/images/rgb
```

### Depth

```text
/images/depth
```

### 3D 坐标关系

```text
/world/map_origin
/world/base
/world/camera
/world/camera_trajectory
```

### 同步诊断

```text
/metrics/sync/*
/metrics/camera_xyz/*
/metrics/skip/*
```

整个界面采用固定 Blueprint：

```text
左侧：
    RGB
    Depth

右侧：
    3D 场景
    同步 / Camera XYZ / Skip
```

---

## 11.3 可视化降采样

接收端不是对所有数据都按 30 FPS 写入 Rerun。

默认：

```python
RGB_EVERY_N = 6
DEPTH_EVERY_N = 10
POSE_EVERY_N = 1
METRICS_EVERY_N = 1
```

如果数据源约为 30 FPS，则：

```text
RGB   ≈ 5 Hz
Depth ≈ 3 Hz
Pose  ≈ 30 Hz
Metrics ≈ 30 Hz
```

这样可以明显降低：

- 浏览器渲染压力；
- Rerun Server 内存；
- JPEG 编码量；
- 网络转发量。

注意：

> 这是可视化端降采样，不影响发布协议本身的帧率。

---

## 11.4 RGB 压缩

协议中传输的是原始 RGB bytes。

但接收端写入 Rerun 前可以进行 JPEG 压缩：

```python
COMPRESS_RGB = True
RGB_JPEG_QUALITY = 80
```

因此：

```text
ZMQ transmission
    = raw RGB

Rerun storage/display
    = optionally JPEG compressed
```

二者不要混淆。

---

## 11.5 相机轨迹

接收端从：

```text
T_map_camera[:3, 3]
```

获取相机位置。

使用：

```python
deque(maxlen=TRAJECTORY_MAX_POINTS)
```

保存最近的轨迹点。

默认：

```python
TRAJECTORY_MAX_POINTS = 1000
```

同时并不是每一帧都重新生成整条轨迹，而是：

```python
TRAJECTORY_UPDATE_EVERY_N = 150
```

周期性更新，用于减少可视化开销。

---

# 12. 协议设计特点

当前实现有几个重要特征。

### 1. 单帧数据自描述

图像尺寸、dtype、depth scale、内参、位姿等都包含在 metadata 中。

消费者不需要预先知道相机的固定分辨率。

### 2. 图像与位姿属于同一个 ZMQ message

避免：

```text
图像一个 topic
pose 一个 topic
```

然后在下游再次做同步。

### 3. 相机时间作为主时间轴

每一个：

```text
T_map_camera
```

都对应当前图像的采集时间，而不是发布时刻。

### 4. 下游无需了解底盘 REST API

下游只需要理解统一协议。

底盘厂商 API 的具体形式被封装在 publisher 内部。

### 5. 实时优先

ZMQ HWM 较小，并使用非阻塞发送。

系统在消费者跟不上时选择丢帧，而不是无限缓存。

---

# 13. 使用该协议进行全局点云 / 体素建图

对于每一帧：

```text
RGB
Depth
K
T_map_camera
```

已经具备生成全局点云所需要的核心数据。

首先根据深度恢复相机坐标系点：

```text
Z = depth * depth_scale_m

X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
```

得到：

```text
p_camera = [X, Y, Z, 1]^T
```

然后：

```text
p_map = T_map_camera @ p_camera
```

即可得到 map 坐标系下的点。

因此后续模块可以直接实现：

```text
RGB-D frame
    ↓
camera point cloud
    ↓
T_map_camera
    ↓
global point cloud
    ↓
voxel downsample
    ↓
occupancy / TSDF / ESDF / semantic voxel map
```

无需重新访问底盘位姿。

---

# 14. 注意事项

## 14.1 外参必须正确

如果：

```text
T_base_camera
```

方向填反，即使时间同步完全正确，也会导致：

- 相机轨迹方向异常；
- 点云漂移；
- 地图旋转；
- 高度错误；
- 随底盘转向产生明显错位。

建议在正式建图前通过 Rerun 观察：

```text
/world/base
/world/camera
```

两套坐标轴是否符合真实安装关系。

---

## 14.2 REST Pose 的时间戳仍然是估计值

当前方案最主要的时间同步误差来源不是 RealSense，而是：

```text
底盘 REST API 无测量时间戳
```

当前采用 HTTP midpoint：

```text
(request_time + response_time) / 2
```

只能近似 pose 测量时刻。

因此：

```text
dt_before
dt_after
RTT
```

应持续监控。

在机器人快速运动或快速旋转情况下，时间误差会直接转化为外参看似不准或地图重影。

---

## 14.3 PUB/SUB 不保证历史数据补发

ZMQ PUB/SUB 是实时广播模型。

如果 Subscriber 启动得比 Publisher 晚：

```text
此前发布的数据不会补发
```

这符合当前实时感知用途。

如果以后需要录制、回放或可靠传输，需要额外增加：

- rosbag；
- MCAP；
- Rerun recording；
- 文件记录器；
- 或其他持久化模块。

---

# 15. 建议的协议兼容策略

后续如果继续扩展该协议，建议：

```text
version = 1
```

保持版本字段。

例如增加：

- IMU；
- confidence；
- odometry；
- wheel velocity；
- timestamp offset；
- semantic image；

时，可以新增字段，而不要修改已有字段的含义。

如果出现不兼容修改，再升级：

```text
version = 2
```

接收端建议至少检查：

```python
if meta.get("type") != "rgbd_pose":
    continue
```

以及：

```python
version = meta.get("version", 0)
```

以方便未来兼容多个协议版本。

---

# 16. 总结

当前两个脚本实现的是一套面向机器人实时感知的数据中转链路：

```text
RealSense RGB-D
        +
Hermes / SLAMTEC Pose
        ↓
PC 时间同步
        ↓
T_map_base
        ↓
T_map_camera
        ↓
ZMQ multipart
        ↓
下游 SLAM / 建图 / 可视化
```

其中 ZMQ 单帧协议固定为：

```text
[ topic, msgpack metadata, RGB bytes, Depth bytes ]
```

时间同步以 RealSense RGB 时间戳为主时间，通过底盘 REST pose 的 HTTP midpoint 时间戳建立 pose buffer，并采用：

```text
线性平移插值 + quaternion SLERP
```

计算与图像同一时刻的底盘位姿。

最终发布：

```text
RGB
Depth
T_map_base
T_map_camera
Camera Intrinsics
Sync Diagnostics
Publisher Statistics
```

这使后续模块可以把该中转程序视为统一的 **RGB-D + 全局相机位姿数据源**，不再直接依赖底盘厂商 API 和 RealSense 采集细节。
