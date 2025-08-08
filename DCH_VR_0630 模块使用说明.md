# DCH_VR_0630 模块使用说明

DCH_VR_0630 是一个完整的车辆方向盘力反馈控制系统，支持真实硬件和模拟环境，提供方向盘力反馈算法和数据处理功能。

## 1. 概述

该模块实现了以下主要功能：

- CAN 总线通信（支持真实硬件和模拟）
- 方向盘角度和踏板数据处理
- 车辆状态数据获取（支持真实游戏和模拟）
- 力反馈算法计算
- WiFi 无线通信支持
- 实时数据可视化

## 2. 主要组件

### 2.1 核心模块

- [DCH_VR_0630.py](file://c:\Users\Administrator\Desktop\g112RC730\DCH_VR_0630.py): 主程序文件
- [MockZCAN.py](file://c:\Users\Administrator\Desktop\g112RC730\MockZCAN.py): CAN 总线模拟器
- [MockACAPI.py](file://c:\Users\Administrator\Desktop\g112RC730\MockACAPI.py): 游戏API模拟器
- [ffb_cal_ori.py](file://c:\Users\Administrator\Desktop\g112RC730\ffb_cal_ori.py): 力反馈算法实现
- [wifi_module.py](file://c:\Users\Administrator\Desktop\g112RC730\wifi_module.py): WiFi 通信模块
- [gui.py](file://c:\Users\Administrator\Desktop\g112RC730\gui.py): 图形用户界面

### 2.2 主要类和结构体

#### 力反馈输出结构体

```python
class ForceFeedbackOutput(ctypes.Structure):
    _fields_ = [
        ("tire_effect", ctypes.c_float),
        ("lateral_effect", ctypes.c_float),
        ("road_effect", ctypes.c_float),
        ("desired_torque", ctypes.c_float),
        ("friction", ctypes.c_float),
        ("damping", ctypes.c_float)
    ]
```

#### CAN 帧结构体

```python
class ZCAN_CANFD_FRAME(Structure):
    _fields_ = [
        ("can_id", c_uint, 29),
        ("err", c_uint, 1),
        ("rtr", c_uint, 1),
        ("eff", c_uint, 1),
        ("len", c_ubyte),
        ("brs", c_ubyte, 1),
        ("esi", c_ubyte, 1),
        ("__res", c_ubyte, 6),
        ("__res0", c_ubyte),
        ("__res1", c_ubyte),
        ("data", c_ubyte * 64)
    ]
```

## 3. 配置选项

### 3.1 主要配置参数

```python
config = {
    'USE_WIFI': False,      # 是否使用WiFi通信
    'USE_REAL_CAN': False,  # 是否使用真实CAN硬件
    'USE_REAL_AC': False,   # 是否使用真实游戏API
    'USE_RC': False         # 是否使用RC模式
}
```

### 3.2 全局变量

```python
# CAN ID 配置
G_STEERING_CAN_ID = 0X11F
G_STEERING_SBW_CAN_ID = 0X11F
G_THROTTLE_BRAKE_CAN_ID = 0x342

# 方向盘相关全局变量
G_STEERING_RATE = 0.0
G_STEERING_WHEEL_ANGLE = 0.0
G_STEERING_WHEEL_ANGLE_OLD = 0.0
```

## 4. 使用方法

### 4.1 基本使用流程

```python
# 导入模块
from DCH_VR_0630 import start_main_process

# 配置参数
config = {
    'USE_WIFI': False,
    'USE_REAL_CAN': False,
    'USE_REAL_AC': False,
    'USE_RC': False
}

# 启动主进程
start_main_process(config)
```

### 4.2 启动参数

- 无参数: 启动图形界面
- `--no` 或 `-n`: 无GUI模式运行

### 4.3 不同模式说明

#### 模拟模式（默认）

```python
config = {
    'USE_WIFI': False,
    'USE_REAL_CAN': False,  # 使用 MockZCAN
    'USE_REAL_AC': False,   # 使用 MockACAPI
    'USE_RC': False
}
```

#### 真实硬件模式

```python
config = {
    'USE_WIFI': False,
    'USE_REAL_CAN': True,   # 使用真实CAN设备
    'USE_REAL_AC': True,    # 使用真实游戏API
    'USE_RC': False
}
```

#### WiFi 模式

```python
config = {
    'USE_WIFI': True,       # 启用WiFi通信
    'USE_REAL_CAN': False,
    'USE_REAL_AC': False,
    'USE_RC': False
}
```

## 5. 主要功能函数

### 5.1 CAN 通信相关

```python
# 初始化CAN设备
def initialize_can(config, window=None)

# 启动CAN通道
def can_start(zcanlib, device_handle, chn)

# 编码方向盘准备帧
def encode_sbw_ready_frame()

# 编码力反馈帧
def encode_sbw_ffb_frame(torque=None)
```

### 5.2 数据处理相关

#### 解码方向盘角度帧

def decode_steering_wheel_angle_frame(can_id, frame_data)

13.7代表第13字节的第7位

##### 代码逻辑分析

###### 1. 数据解析分支

```python
if can_id == G_STEERING_CAN_ID:
    angle_raw = (frame_data[1] << 8) | frame_data[0]  # 使用字节0和1
    steering_rate = int(frame_data[2]) * 4            # 使用字节2
else:
    angle_raw = (frame_data[6] << 8) | frame_data[5]  # 使用字节5和6
    steering_rate = int(frame_data[7]) * 4            # 使用字节7
```

###### 分支1: 当 CAN ID 等于 [G_STEERING_CAN_ID](file://c:\Users\Administrator\Desktop\g112RC730\DCH_VR_0630.py#L121-L121)

- **角度数据**: 由字节 0.7-0.0 和 1.7-1.0 组成（小端序）
- **转向速率**: 从字节 2.7-2.0 获取

#### 分支2: 当 CAN ID 不等于 [G_STEERING_CAN_ID](file://c:\Users\Administrator\Desktop\g112RC730\DCH_VR_0630.py#L121-L121)

- **角度数据**: 由字节 5.7-5.0 和 6.7-6.0 组成（小端序）
- **转向速率**: 从字节 7.7-7.0 获取

### 2. 负数处理逻辑

```python
if angle_raw > 0x7FFF:  # 检查是否为负数（16位有符号整数）
    angle_raw = (~angle_raw & 0xFFFF) + 1  # 二进制补码转换
    steering_wheel_angle = angle_raw / 10.0
else:
    steering_wheel_angle = -angle_raw / 10.0  # 注意这里有负号
```

这里有逻辑问题：正常情况下，如果最高位为1（> 0x7FFF），应该表示负数，但代码中的处理方式有些混乱。

## 按位分析（以分支1为例）

假设我们有一个8字节的CAN帧数据：

| 字节索引 | 位 7 | 位 6 | 位 5 | 位 4 | 位 3 | 位 2 | 位 1 | 位 0 |
| -------- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- |
| 0        | 0.7  | 0.6  | 0.5  | 0.4  | 0.3  | 0.2  | 0.1  | 0.0  |
| 1        | 1.7  | 1.6  | 1.5  | 1.4  | 1.3  | 1.2  | 1.1  | 1.0  |
| 2        | 2.7  | 2.6  | 2.5  | 2.4  | 2.3  | 2.2  | 2.1  | 2.0  |
| ...      | ...  | ...  | ...  | ...  | ...  | ...  | ...  | ...  |

### 角度数据提取：

1. 从字节 0.7-0.0 和 1.7-1.0 组合成16位值
2. 如果值 > 0x7FFF，则按二进制补码规则转换为负数
3. 除以10.0得到实际角度值

### 转向速率提取：

1. 从字节 2.7-2.0 获取
2. 乘以4得到实际转向速率

## 实际示例

假设接收到的数据：

```
frame_data = [0xE8, 0x03, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00]
```

处理过程：

1. `angle_raw = (0x03 << 8) | 0xE8 = 0x03E8 = 1000`
2. `angle_raw <= 0x7FFF`，所以进入else分支
3. `steering_wheel_angle = -1000 / 10.0 = -100.0` 度
4. `steering_rate = 0x64 * 4 = 100 * 4 = 400`

## 问题点

代码中的负数处理逻辑似乎有误：

```python
if angle_raw > 0x7FFF:
    # 处理负数情况
    angle_raw = (~angle_raw & 0xFFFF) + 1
    steering_wheel_angle = angle_raw / 10.0
else:
    # 正数情况，但这里用了负号
    steering_wheel_angle = -angle_raw / 10.0  # <- 这里可能有问题
```

正常逻辑应该是：

```python
if angle_raw > 0x7FFF:
    # 负数情况
    steering_wheel_angle = -(0x10000 - angle_raw) / 10.0
else:
    # 正数情况
    steering_wheel_angle = angle_raw / 10.0
```

# 解码油门刹车帧

def decode_throttle_brake_frame(frame_data)

# 编码车身姿态帧

def encode_roll_pitch_frame(vehicle_type, throttle, brake, game_roll, game_pitch, speed, steering_wheel_angle, steering_wheel_angle_old, steering_wheel_rate)

### 5.3 数据记录

```python
# 记录扭矩数据
def record_torque_data(start_time, desired_torque, damping, friction, total_torque, scale_torque, lateral_effect, suspension_effect)

# 读取车辆状态
def read_vehicle_status(ac_api)
```

### 5.4 线程处理

```python
# 发送消息线程
def send_messages(chn_handle, ac_api, zcanlib)

# 接收消息线程
def receive_messages(chn_handle, zcanlib)

# 更新全局变量
def update_g_vars(angle, rate, direction)
```

## 6. 数据流说明

### 6.1 方向盘角度数据流

```
MockZCAN 生成模拟数据 → CAN帧 → 解码 → 更新全局变量 G_STEERING_WHEEL_ANGLE
```

### 6.2 车辆状态数据流

```
MockACAPI 生成模拟数据 → API调用 → 力反馈算法 → 计算力矩 → 编码为CAN帧 → 发送
```

### 6.3 力反馈数据流

```
车辆状态数据 + 方向盘角度 → 力反馈算法 → 计算各种力效应 → 合成总力矩 → 编码发送
```

## 7. 力反馈算法

### 7.1 主要力效应

- **期望扭矩** (desired_torque): 基于车速和方向盘角度的基本力反馈
- **阻尼力** (damping): 与方向盘转速相关的阻尼力
- **摩擦力** (friction): 模拟机械摩擦
- **侧向力** (lateral_effect): 基于车辆侧向加速度的力反馈
- **悬挂力** (suspension_effect): 基于悬挂行程变化的力反馈

### 7.2 算法调用

```python
ffb = ForceFeedbackAlgorithm()
desired_torque = ffb.get_tanh_torque(speed, G_STEERING_WHEEL_ANGLE)
friction, damping = ffb.get_friction(ctypes.c_float(G_STEERING_WHEEL_ANGLE), ctypes.c_float(G_STEERING_RATE))
lateral_effect = ffb.get_lateral_effect(speed, acc_g, local_angular_vel, wheel_slip.slip)
suspension_effect = ffb.get_suspension_effect(speed, suspension_travel)
```

## 8. 图形界面

### 8.1 主要功能

- 配置参数设置（WiFi/CAN/AC/RC）
- 实时扭矩图表显示
- CAN 报文查看
- FFB 分析图表

### 8.2 操作说明

1. 启动程序后显示配置界面
2. 选择相应的模式（WiFi/CAN/AC/RC）
3. 点击"Connect"按钮开始运行
4. 切换到不同标签页查看数据

## 9. 注意事项

### 9.1 硬件要求

- 真实CAN模式需要 compatible CAN 设备
- 游戏API模式需要相应的游戏运行

### 9.2 环境配置

- 确保所需DLL文件在正确位置
- 安装必要的Python依赖包

### 9.3 故障排除

1. **CAN通信失败**: 检查硬件连接和驱动
2. **游戏API连接失败**: 确认游戏是否运行
3. **数据不更新**: 检查线程是否正常运行
4. **力反馈异常**: 检查算法参数和输入数据

## 10. 扩展开发

### 10.1 添加新的力反馈效应

在 [ForceFeedbackAlgorithm](file://c:\Users\Administrator\Desktop\g112RC730\ffb_cal_ori.py#L49-L324) 类中添加新的计算方法

### 10.2 修改模拟数据

调整 [MockZCAN](file://c:\Users\Administrator\Desktop\g112RC730\MockZCAN.py#L0-L0) 和 [MockACAPI](file://c:\Users\Administrator\Desktop\g112RC730\MockACAPI.py#L0-L0) 中的数据生成逻辑

### 10.3 添加新的CAN ID处理

在 [receive_messages](file://c:\Users\Administrator\Desktop\g112RC730\DCH_VR_0630.py#L1190-L1295) 函数中添加新的CAN ID处理逻辑

### 10.4 自定义GUI

修改 [gui.py](file://c:\Users\Administrator\Desktop\g112RC730\gui.py) 文件来定制用户界面


安装使用

pip **install** -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple/
