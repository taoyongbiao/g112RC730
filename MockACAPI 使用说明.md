# MockACAPI 使用说明

MockACAPI 是一个用于模拟真实游戏API（如神力科莎）的模块，主要用于在没有真实游戏运行时进行开发和测试。

## 1. 概述

MockACAPI 模拟了真实游戏API的基本功能，提供车辆状态数据，包括：
- 车辆速度
- 方向盘角度
- 轮胎滑移率
- 车辆加速度
- 悬挂行程
- 车辆角速度
- 车辆滚转角和俯仰角

## 2. 主要类和方法

### 2.1 MockACAPI 类

这是主要的模拟类，提供了与真实游戏API相同的接口。

#### 构造函数
```python
mock_api = MockACAPI()
```

#### 主要方法

| 方法 | 返回类型 | 描述 |
|------|----------|------|
| `AC_GetSpeedKmh()` | float | 获取车辆速度(km/h) |
| `AC_GetSteerAngle()` | float | 获取方向盘角度 |
| `AC_GetWheelSlip()` | WheelSlip | 获取轮胎滑移率 |
| `AC_GetAccG()` | AccG | 获取车辆加速度 |
| `AC_GetSuspensionTravel()` | SuspensionTravel | 获取悬挂行程 |
| `AC_GetLocalAngularVel()` | LocalAngularVel | 获取车辆角速度 |
| `AC_GetRoll()` | float | 获取车辆滚转角 |
| `AC_GetPitch()` | float | 获取车辆俯仰角 |
| `AC_StartUpdate()` | int | 初始化更新(始终返回1) |

### 2.2 数据结构

#### WheelSlip 结构体
```python
class WheelSlip(Structure):
    _fields_ = [("slip", c_float * 4)]  # 四个轮胎的滑移率
```

#### AccG 结构体
```python
class AccG(Structure):
    _fields_ = [("accg", c_float * 3)]  # XYZ三个方向的加速度
```

#### SuspensionTravel 结构体
```python
class SuspensionTravel(Structure):
    _fields_ = [("st", c_float * 4)]  # 四个悬挂的行程
```

#### LocalAngularVel 结构体
```python
class LocalAngularVel(Structure):
    _fields_ = [("VehAngVel", c_float * 3)]  # 车辆绕XYZ轴的角速度
```

## 3. 使用方法

### 3.1 基本使用流程

```python
from MockACAPI import MockACAPI

# 创建 MockACAPI 实例
api = MockACAPI()

# 获取车辆状态数据
speed = api.AC_GetSpeedKmh()
steer_angle = api.AC_GetSteerAngle()
wheel_slip = api.AC_GetWheelSlip()
acc_g = api.AC_GetAccG()
suspension = api.AC_GetSuspensionTravel()
angular_vel = api.AC_GetLocalAngularVel()
roll = api.AC_GetRoll()
pitch = api.AC_GetPitch()

# 打印数据
print(f"速度: {speed} km/h")
print(f"方向盘角度: {steer_angle} 度")
print(f"滚转角: {roll} 度")
print(f"俯仰角: {pitch} 度")
```

### 3.2 访问结构体数据

```python
# 访问轮胎滑移率
wheel_slip = api.AC_GetWheelSlip()
for i in range(4):
    print(f"轮胎 {i+1} 滑移率: {wheel_slip.slip[i]}")

# 访问加速度数据
acc_g = api.AC_GetAccG()
print(f"X轴加速度: {acc_g.accg[0]}")
print(f"Y轴加速度: {acc_g.accg[1]}")
print(f"Z轴加速度: {acc_g.accg[2]}")

# 访问悬挂行程
suspension = api.AC_GetSuspensionTravel()
for i in range(4):
    print(f"悬挂 {i+1} 行程: {suspension.st[i]}")

# 访问车辆角速度
angular_vel = api.AC_GetLocalAngularVel()
print(f"绕X轴角速度: {angular_vel.VehAngVel[0]}")
print(f"绕Y轴角速度: {angular_vel.VehAngVel[1]}")
print(f"绕Z轴角速度: {angular_vel.VehAngVel[2]}")
```

## 4. 模拟数据说明

### 4.1 速度 (AC_GetSpeedKmh)
- 当前固定返回值: 130 km/h
- 注释掉的代码提供正弦波变化模式(80±50 km/h)

### 4.2 方向盘角度 (AC_GetSteerAngle) 没有使用，使用can或模拟can进行读取
- 返回值范围: ±450度
- 按正弦波变化，周期约为6.28秒

### 4.3 轮胎滑移率 (AC_GetWheelSlip)
- 基于方向盘角度计算
- 滑移率 = min(|方向盘角度|/450, 1.0)
- 四个轮胎滑移率相同

### 4.4 车辆加速度 (AC_GetAccG)
- X轴: 0.0 g
- Y轴: 0.1 g
- Z轴: 0.05 g

### 4.5 悬挂行程 (AC_GetSuspensionTravel)
- 前悬挂(0,1): 0.02米
- 后悬挂(2,3): 0.01米

### 4.6 车辆角速度 (AC_GetLocalAngularVel)
- 所有轴角速度: 0.0 rad/s

### 4.7 滚转角和俯仰角 (AC_GetRoll, AC_GetPitch)
- 当前固定返回值: -50.0 度
- 注释掉的代码提供每秒循环变化模式(-50, -25, 0, 25, 50度)

## 5. 自定义配置

### 5.1 修改速度模拟
取消注释并修改 [AC_GetSpeedKmh](file://c:\Users\Administrator\Desktop\g112RC730\MockACAPI.py#L29-L32) 方法中的代码：
```python
def AC_GetSpeedKmh(self):
    # 取消注释下面一行来启用正弦波变化
    # return 80 + 50 * math.sin((time.time() - self.start_time) * 2 * math.pi)
    return 130
```

### 5.2 修改滚转角和俯仰角模拟
取消注释并修改 `AC_GetRoll` 和 `AC_GetPitch` 方法中的代码：
```python
def AC_GetRoll(self):
    # 取消注释下面的代码来启用循环变化
    now = time.time()
    if now - self.last_roll_time >= 1.0:  # 每秒更新一次
        self.value_index = (self.value_index + 1) % len(self.values)
        self.last_roll_time = now
    return self.values[self.value_index]
```

### 5.3 修改循环值
在构造函数中修改 `self.values` 列表：
```python
def __init__(self):
    self.start_time = time.time()
    # 修改这里的值来自定义循环序列
    self.values = [-50.0, -25.0, 0.0, 25.0, 50.0]
    self.value_index = 0
    self.last_roll_time = time.time()
```

## 6. 注意事项

1. **数据一致性**: 模拟数据是静态的或按简单规律变化的，可能与真实游戏数据有差异
2. **初始化**: [AC_StartUpdate()](file://c:\Users\Administrator\Desktop\g112RC730\MockACAPI.py#L80-L81) 始终返回1，表示初始化成功
3. **时间基准**: 使用 `time.time()` 作为时间基准进行数据变化计算
4. **结构体访问**: 访问结构体成员时需要使用正确的字段名

## 7. 故障排除

### 7.1 数据不变化
- 检查是否使用了固定返回值的函数版本
- 确认时间计算是否正确

### 7.2 结构体访问错误
- 确保使用正确的字段名访问结构体成员
- 检查数组索引是否越界

### 7.3 数据类型错误
- 确保正确处理返回的 ctypes 结构体
- 注意 float 类型的数据精度

## 8. 扩展使用

可以根据需要修改或扩展模拟数据：

```python
# 添加新的模拟方法
def AC_GetRPM(self):
    return 3000 + 1000 * math.sin((time.time() - self.start_time) * 0.5)

# 修改现有方法以提供更真实的模拟数据
def AC_GetSpeedKmh(self):
    # 模拟加速和减速过程
    elapsed = time.time() - self.start_time
    if elapsed < 10:
        return 0 + 20 * elapsed  # 0-10秒内加速到200km/h
    else:
        return 200  # 保持200km/h
```

## 9. 与其他模块的配合

MockACAPI 通常与 MockZCAN 配合使用，提供完整的车辆状态模拟：
- MockACAPI: 提供车辆状态数据(速度、姿态等)
- MockZCAN: 提供方向盘角度和踏板数据

这样可以在没有真实硬件和游戏的情况下进行完整的系统测试。