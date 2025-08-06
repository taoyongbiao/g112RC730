# MockZCAN 使用说明

MockZCAN 是一个用于模拟真实 ZCAN 设备的模块，主要用于在没有真实硬件设备的情况下进行开发和测试。

## 1. 概述

MockZCAN 模拟了真实 ZCAN 设备的基本功能，包括：
- 设备打开/关闭
- CAN 通道初始化和启动
- CAN 帧的发送和接收
- 模拟方向盘角度和油门/刹车数据

## 2. 主要类和方法

### 2.1 MockZCAN 类

这是主要的模拟类，提供了与真实 ZCAN 设备相同的接口。

#### 构造函数
```python
mock_zcan = MockZCAN()
```

#### 主要方法

| 方法 | 描述 |
|------|------|
| `OpenDevice(device_type, device_index, reserved)` | 模拟打开设备 |
| `CloseDevice(device_handle)` | 模拟关闭设备 |
| `GetDeviceInf(device_handle)` | 获取设备信息 |
| `InitCAN(device_handle, can_index, init_config)` | 初始化 CAN 通道 |
| `StartCAN(chn_handle)` | 启动 CAN 通道 |
| `ResetCAN(chn_handle)` | 重置 CAN 通道 |
| `ClearBuffer(chn_handle)` | 清除缓冲区 |
| `GetReceiveNum(chn_handle, can_type=1)` | 获取接收帧数量 |
| `ReceiveFD(chn_handle, rcv_num, wait_time=1000)` | 接收 CAN FD 帧 |
| `TransmitFD(chn_handle, fd_msg, len)` | 发送 CAN FD 帧 |
| `inject_frame(frame)` | 注入帧（用于测试） |

## 3. 使用方法

### 3.1 基本使用流程

```python
from MockZCAN import MockZCAN

# 创建 MockZCAN 实例
zcan = MockZCAN()

# 打开设备
device_handle = zcan.OpenDevice(device_type=41, device_index=0, reserved=0)

# 初始化 CAN 通道
# 需要先创建初始化配置（此处省略详细配置）
chn_handle = zcan.InitCAN(device_handle, can_index=0, init_config=init_config)

# 启动 CAN 通道
zcan.StartCAN(chn_handle)

# 发送数据
# 构造要发送的 CAN FD 帧
frame = ZCAN_TransmitFD_Data()
# 设置帧数据...
result = zcan.TransmitFD(chn_handle, frame, 1)

# 接收数据
num_frames = zcan.GetReceiveNum(chn_handle, 1)  # 1 表示 CAN FD
if num_frames > 0:
    frames, count = zcan.ReceiveFD(chn_handle, num_frames, 1000)
    # 处理接收到的帧

# 重置和关闭
zcan.ResetCAN(chn_handle)
zcan.CloseDevice(device_handle)
```

### 3.2 方向盘数据模拟

MockZCAN 会自动生成模拟的方向盘角度和油门/刹车数据：

- **方向盘角度**: 在 -200° 到 +200° 之间按正弦波变化
- **油门值**: 在 0-100 之间按正弦波变化
- **刹车值**: 固定为 0

模拟数据每 100ms 更新一次。

### 3.3 回调机制

MockZCAN 支持回调机制来实时更新方向盘数据：

```python
# 设置回调函数
zcan.on_steering_update = lambda angle, rate, direction: your_callback_function(angle, rate, direction)

def your_callback_function(angle, rate, direction):
    # 处理方向盘角度更新
    print(f"方向盘角度: {angle}, 转向速率: {rate}, 方向: {direction}")
```

## 4. 配置参数

### 4.1 CAN ID 配置

```python
G_STEERING_SBW_CAN_ID = 0x11F    # 方向盘角度数据 CAN ID
G_THROTTLE_BRAKE_CAN_ID = 0x342  # 油门/刹车数据 CAN ID
```

### 4.2 数据格式

方向盘角度帧格式：
- CAN ID: 0x11F
- 数据长度: 8 字节
- 数据[0:1]: 方向盘角度 (小端格式，放大10倍)
- 数据[2]: 转向速率

油门/刹车帧格式：
- CAN ID: 0x342
- 数据长度: 8 字节
- 数据[0]: 油门值 (0-100)
- 数据[1]: 刹车值 (0-100)

## 5. 注意事项

1. **CAN ID 匹配**: 确保 MockZCAN 中的 CAN ID 与主程序中的配置一致
2. **线程安全**: MockZCAN 使用后台线程生成模拟数据，请注意线程安全
3. **资源清理**: 使用完毕后调用 [ResetCAN](file://c:\Users\Administrator\Desktop\g112RC730\DCH_VR_0630.py#L449-L455) 和 [CloseDevice](file://c:\Users\Administrator\Desktop\g112RC730\DCH_VR_0630.py#L405-L411) 清理资源
4. **回调设置**: 如需实时获取方向盘数据，需要设置 [on_steering_update](file://c:\Users\Administrator\Desktop\g112RC730\MockZCAN.py#L0-L0) 回调

## 6. 故障排除

### 6.1 方向盘角度不更新

检查以下几点：
1. CAN ID 是否匹配
2. 是否正确设置了回调函数 [on_steering_update](file://c:\Users\Administrator\Desktop\g112RC730\MockZCAN.py#L0-L0)
3. [StartCAN](file://c:\Users\Administrator\Desktop\g112RC730\DCH_VR_0630.py#L439-L447) 是否被正确调用

### 6.2 无法接收数据帧

1. 确认 [StartCAN](file://c:\Users\Administrator\Desktop\g112RC730\DCH_VR_0630.py#L439-L447) 已被调用
2. 检查 [GetReceiveNum](file://c:\Users\Administrator\Desktop\g112RC730\DCH_VR_0630.py#L485-L491) 和 [ReceiveFD](file://c:\Users\Administrator\Desktop\g112RC730\DCH_VR_0630.py#L519-L527) 的使用方式
3. 确认模拟线程是否正常运行

### 6.3 模拟数据异常

1. 检查时间戳处理是否正确
2. 确认数据编码和解码是否匹配
3. 查看日志输出以获取更多信息

## 7. 日志输出

MockZCAN 使用 `loguru` 库进行日志记录，关键操作都会有相应的日志输出，便于调试和监控。

## 8. 扩展使用

如需扩展模拟功能，可以修改 [_simulate_canfd_frames](file://c:\Users\Administrator\Desktop\g112RC730\MockZCAN.py#L164-L214) 方法来添加更多类型的模拟数据，或者使用 [inject_frame](file://c:\Users\Administrator\Desktop\g112RC730\MockZCAN.py#L153-L161) 方法手动注入特定的 CAN 帧。