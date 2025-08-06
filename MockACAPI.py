import time
import math
from ctypes import c_float, Structure

# 模拟结构体定义（保持与主程序一致）
class WheelSlip(Structure):
    _pack_ = 1
    _fields_ = [("slip", c_float * 4)]

class AccG(Structure):
    _pack_ = 1
    _fields_ = [("accg", c_float * 3)]

class SuspensionTravel(Structure):
    _pack_ = 1
    _fields_ = [("st", c_float * 4)]

class LocalAngularVel(Structure):
    _pack_ = 1
    _fields_ = [("VehAngVel", c_float * 3)]


class MockACAPI:
    def __init__(self):
        self.start_time = time.time()
        self.values = [-50.0, -25.0, 0.0, 25.0, 50.0]# 定义要循环的值
        self.value_index = 0
        self.last_roll_time = time.time()        

    def AC_GetSpeedKmh(self):
        # return 80 + 50 * math.sin((time.time() - self.start_time) * 0.5)
        #  return 80 + 50 * math.sin((time.time() - self.start_time) * 2 * math.pi)#1s完成一个周期的变化
        return 130

    def AC_GetSteerAngle(self):
        return 450 * math.sin((time.time() - self.start_time) * 1.0)

    def AC_GetWheelSlip(self):
        steer_angle = self.AC_GetSteerAngle()
        slip_value = min(abs(steer_angle) / 450, 1.0)
        ws = WheelSlip()
        for i in range(4):
            ws.slip[i] = slip_value
        return ws

    def AC_GetAccG(self):
        acc = AccG()
        acc.accg[0] = 0.0
        acc.accg[1] = 0.1
        acc.accg[2] = 0.05
        return acc

    def AC_GetSuspensionTravel(self):
        st = SuspensionTravel()
        st.st[0] = st.st[1] = 0.02
        st.st[2] = st.st[3] = 0.01
        return st

    def AC_GetLocalAngularVel(self):
        lav = LocalAngularVel()
        lav.VehAngVel[0] = lav.VehAngVel[1] = 0.0
        lav.VehAngVel[2] = 0.0
        return lav
    
    def AC_GetRoll(self):
        return -50.0  # 模拟值
        # now = time.time()
        # if now - self.last_roll_time >= 1.0:  # 每秒更新一次
        #     self.value_index = (self.value_index + 1) % len(self.roll_values)
        #     self.last_roll_time = now
        # return self.values[self.value_index]        

    def AC_GetPitch(self):
        return -50.0  # 模拟值
        # now = time.time()
        # if now - self.last_roll_time >= 1.0:  # 每秒更新一次
        #     self.value_index = (self.value_index + 1) % len(self.roll_values)
        #     self.last_roll_time = now
        # return self.values[self.value_index]
    
    def AC_StartUpdate(self):
        return 1