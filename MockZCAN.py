# MockZCAN.py
import time
import threading
from ctypes import Structure, c_uint, c_ubyte, c_ulonglong, byref
from loguru import logger
import math

G_STEERING_SBW_CAN_ID = 0x8E  # canid
# 与主程序一致的结构体定义F
class ZCAN_CANFD_FRAME(Structure):
    _pack_ = 1
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


class ZCAN_ReceiveFD_Data(Structure):
    _pack_ = 1
    _fields_ = [("frame", ZCAN_CANFD_FRAME), ("timestamp", c_ulonglong)]


class MockZCAN:
    
    def __init__(self):
        self.devices = {}
        self.channels = {
            "simulated_channel_0": {
                "device_handle": None,
                "can_index": 0,
                "running": False,
                "frames": []
            }
}
        self.running = True
        self.frame_queue = []  # 模拟的帧队列
        self.simulate_thread = None # 模拟线程
        self.on_steering_update = None  # 回调接口

    # 模拟 OpenDevice
    def OpenDevice(self, device_type, device_index, reserved):
        dev_handle = hash(f"mock_device_{device_type}_{device_index}")
        self.devices[dev_handle] = {
            "type": device_type,
            "index": device_index,
            "online": True
        }
        #print(f"[MockZCAN] Opened device handle: {dev_handle}")
        logger.debug(f"[MockZCAN] Opened device handle: {dev_handle}")
        return dev_handle

    # 模拟 CloseDevice
    def CloseDevice(self, device_handle):
        if device_handle in self.devices:
            del self.devices[device_handle]
            #print(f"[MockZCAN] Closed device handle: {device_handle}")
            logger.debug(f"[MockZCAN] Closed device handle: {device_handle}")

    # 模拟 GetDeviceInf
    class ZCAN_DEVICE_INFO(Structure):
        pass  # 可以返回任意 mock info

    def GetDeviceInf(self, device_handle):
        class MockDeviceInfo:
            def __str__(self):
                return "[Mock] Device Info: Mocked Device"

        return MockDeviceInfo()

    # 模拟 InitCAN
    def InitCAN(self, device_handle, can_index, init_config):
        chn_handle = hash(f"mock_channel_{device_handle}_{can_index}")
        self.channels[chn_handle] = {
            "device_handle": device_handle,
            "can_index": can_index,
            "running": False,
            "frames": []
        }
        #print(f"[MockZCAN] Initialized channel handle: {chn_handle}")
        logger.debug(f"[MockZCAN] Initialized channel handle: {chn_handle}")
        return chn_handle

    # 模拟 StartCAN
    def StartCAN(self, chn_handle):
        if chn_handle in self.channels:
            self.channels[chn_handle]["running"] = True
            #print(f"[MockZCAN] Channel started: {chn_handle}")
            logger.debug(f"[MockZCAN] Channel started: {chn_handle}")
            if not self.simulate_thread or not self.simulate_thread.is_alive():
                self.simulate_thread = threading.Thread(target=self._simulate_canfd_frames, daemon=True)
                self.simulate_thread.start()
            return 1
        return 0
        # 清理资源
    def stop(self): 
        if self.simulate_thread and self.simulate_thread.is_alive():
            self.running = False
            self.simulate_thread.join(timeout=1.0)
            #print("[MockZCAN] Simulate thread stopped")
            logger.debug("[MockZCAN] Simulate thread stopped") 
    # 模拟 ResetCAN
    def ResetCAN(self, chn_handle):
        if chn_handle in self.channels:
            self.channels[chn_handle]["running"] = False
            #print(f"[MockZCAN] Channel reset: {chn_handle}")
            logger.debug(f"[MockZCAN] Channel reset: {chn_handle}")

        # 结束模拟线程
        self.stop()
        # if self.simulate_thread and self.simulate_thread.is_alive():
        #     self.running = False  # 控制 _simulate_canfd_frames 循环退出
        #     self.simulate_thread.join(timeout=1.0)  # 最多等待 1 秒
        #     print("[MockZCAN] Simulate thread stopped")
     
            # return 1
        return 0

    # 模拟 ClearBuffer
    def ClearBuffer(self, chn_handle):
        #print(f"[MockZCAN] Buffer cleared for channel: {chn_handle}")
        logger.debug(f"[MockZCAN] Buffer cleared for channel: {chn_handle}")
        return 1

    # 模拟 GetReceiveNum
    def GetReceiveNum(self, chn_handle, can_type=1):  # ZCAN_TYPE_CANFD == 1
        return len(self.frame_queue)

    # 模拟 ReceiveFD
    def ReceiveFD(self, chn_handle, rcv_num, wait_time=1000):
        count = min(rcv_num, len(self.frame_queue))
        result = (ZCAN_ReceiveFD_Data * count)()
        for i in range(count):
            result[i].frame = self.frame_queue.pop(0)
            result[i].timestamp = int(time.time() * 1000)
        #print(f"[MockZCAN] Received {count} CAN FD frames")
        logger.debug(f"[MockZCAN] Received {count} CAN FD frames")
        return result, count

    # 模拟 TransmitFD  模拟发送消息
    def TransmitFD(self, chn_handle, fd_msg, len):
        #print(f"[MockZCAN] TransmitFD called with {len} messages")
        logger.debug(f"[MockZCAN] TransmitFD called with {len} messages")
        return 1

    # 注入帧的方法，可用于单元测试或手动控制
    def inject_frame(self, frame: ZCAN_CANFD_FRAME):
        self.frame_queue.append(frame)
        if self.on_steering_update and frame.can_id == G_STEERING_SBW_CAN_ID:
            angle = ((frame.data[1] << 8) | frame.data[0]) / 10.0
            rate = frame.data[2]
            direction = 1 if frame.data[0] < frame.data[1] else -1
            self.on_steering_update(angle, rate, direction)
        # #print(f"[MockZCAN] Injected frame with ID: {hex(frame.can_id)}")
            logger.debug(f"[MockZCAN] Injected frame with ID: {hex(frame.can_id)}")

    # 自动生成方向盘角度和油门/刹车帧的模拟函数
    def _simulate_canfd_frames(self):


        G_STEERING_SBW_CAN_ID = 0x8E  #转向系统的CAN总线ID
        # G_STEERING_SBW_CAN_ID = 0x11F
        G_THROTTLE_BRAKE_CAN_ID = 0x342 #油门/刹车的CAN总线ID

        while self.running:
            now = int(time.time() * 1000)
            angle = 200 * math.sin(now / 1000)
            throttle = int(50 + 50 * math.sin(now / 500))
            # brake = int(50 + 50 * math.cos(now / 500))
            brake=0

            
            # 构造方向盘角度帧
            steering_frame = ZCAN_CANFD_FRAME()  # 初始化一个 CAN FD 帧对象

            # 设置帧属性
            steering_frame.can_id = G_STEERING_SBW_CAN_ID  # CAN ID: 0x8E，表示方向盘系统专用通道
            steering_frame.eff = 0     # 标准帧（11位ID）
            steering_frame.rtr = 0     # 数据帧（非请求帧）
            steering_frame.len = 8     # 数据长度为 8 字节
            steering_frame.brs = 0     # 不启用比特率切换

            # 计算角度原始值（放大10倍后转为整数）
            angle_raw = int(angle * 10) & 0xFFFF  # 将浮点角度 ×10 并限制为 16 位无符号整数

            # 写入角度数据（小端格式）
            steering_frame.data[0] = angle_raw & 0xFF           # 写入低字节
            steering_frame.data[1] = (angle_raw >> 8) & 0xFF   # 写入高字节

            # 写入模拟的转向速率（单位可能是 °/s）
            steering_frame.data[2] = 100  # 模拟方向盘角速度为 100

            # 将构造好的帧注入到帧队列中，供接收函数读取
            self.inject_frame(steering_frame)

            # 构造油门刹车帧
            throttle_frame = ZCAN_CANFD_FRAME()
            throttle_frame.can_id = G_THROTTLE_BRAKE_CAN_ID
            throttle_frame.eff = 0
            throttle_frame.rtr = 0
            throttle_frame.len = 8
            throttle_frame.brs = 0
            throttle_frame.data[0] = throttle
            throttle_frame.data[1] = brake
            self.inject_frame(throttle_frame)

            # time.sleep(0.02)  # 每秒约 50 帧
            time.sleep(0.1)

