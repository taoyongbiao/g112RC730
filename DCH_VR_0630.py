from ctypes import *
import platform
import ctypes
import time
import threading
import pyvjoy
import os
import math
from ffb_cal_ori import ForceFeedbackAlgorithm
# from ffb_cal_0630 import ForceFeedbackAlgorithm
# from ffb_cal_0624 import ForceFeedbackAlgorithm
# from ffb_rc import get_ffb_from_game_api,get_ffb_from_algorithm

from ffb_rc import FrameData,FrameInputData,read_vehicle_status_from_rc
# from ffb import ForceFeedbackAlgorithm
# import matplotlib
# matplotlib.use('TkAgg')
# import matplotlib.pyplot as plt
import sys
import numpy as np

import matplotlib.animation as animation

from MockACAPI import MockACAPI as ACAPI

from loguru import logger

import ffb_rc
fRead = None
fEnd = None
RC = False
ac_api=None

fWriteInput = None







from shared_state import run_main_flag_event

import wifi_module


main_thread_list = []


# 模块级缓存
_real_zcan = None
_mock_zcan = None
_zcan_imported = False


def _init_zcan(real_can):
    global _real_zcan, _mock_zcan, _zcan_imported
    if _zcan_imported:
        return

    if real_can:
        # 延迟导入真实 CAN 模块（只有第一次才加载）
        
        _real_zcan = ZCAN
    else:
        # 延迟导入模拟 CAN 模块
        from MockZCAN import MockZCAN as MockZCAN
        _mock_zcan = MockZCAN

    _zcan_imported = True


# 数据缓存
MAX_DATA_POINTS = 5000 # 显示最近100个点
torque_data = {
    'time': [],
    'total_torque': [],
    'scale_torque': [],
    'desired_torque': [],
    'damping': [],
    'friction': [],

    # 新增方向盘相关数据
    # 'steering_angle_old': [],
    'steering_angle': [],
    'steering_rate': [],
    'rate_dir': [],
    'lateral_effect':[],
    'suspension_effect':[],
}


'''
 SBW CRC ROLL CNT
'''
G_ROLL_CNT = 0
G_READY_ROLL_CNT = 0
G_RATE_DIR = 1



class ForceFeedbackOutput(ctypes.Structure):
    _fields_ = [
        ("tire_effect", ctypes.c_float),
        ("lateral_effect", ctypes.c_float),
        ("road_effect", ctypes.c_float),
        ("desired_torque", ctypes.c_float),
        ("friction", ctypes.c_float),
        ("damping", ctypes.c_float)
    ]


'''
 Global Variable 
'''
G_STEERING_RATE = 0.0
G_STEERING_WHEEL_ANGLE = 0.0
G_STEERING_WHEEL_ANGLE_OLD = 0.0
G_THROTTLE = 0.0
G_BRAKE = 0.0

'''
 CAN ID
'''
G_STEERING_CAN_ID = 0X11F
# G_STEERING_SBW_CAN_ID = 0X8E
G_STEERING_SBW_CAN_ID = 0X11F
G_THROTTLE_BRAKE_CAN_ID = 0x342

INVALID_DEVICE_HANDLE = 0
INVALID_CHANNEL_HANDLE = 0

ZCAN_DEVICE_TYPE = c_uint
'''
 Device Type
'''
ZCAN_PCI5121 = ZCAN_DEVICE_TYPE(1)
ZCAN_PCI9810 = ZCAN_DEVICE_TYPE(2)
ZCAN_USBCAN1 = ZCAN_DEVICE_TYPE(3)
ZCAN_USBCAN2 = ZCAN_DEVICE_TYPE(4)
ZCAN_PCI9820 = ZCAN_DEVICE_TYPE(5)
ZCAN_CAN232 = ZCAN_DEVICE_TYPE(6)
ZCAN_PCI5110 = ZCAN_DEVICE_TYPE(7)
ZCAN_CANLITE = ZCAN_DEVICE_TYPE(8)
ZCAN_ISA9620 = ZCAN_DEVICE_TYPE(9)
ZCAN_ISA5420 = ZCAN_DEVICE_TYPE(10)
ZCAN_PC104CAN = ZCAN_DEVICE_TYPE(11)
ZCAN_CANETUDP = ZCAN_DEVICE_TYPE(12)
ZCAN_CANETE = ZCAN_DEVICE_TYPE(12)
ZCAN_DNP9810 = ZCAN_DEVICE_TYPE(13)
ZCAN_PCI9840 = ZCAN_DEVICE_TYPE(14)
ZCAN_PC104CAN2 = ZCAN_DEVICE_TYPE(15)
ZCAN_PCI9820I = ZCAN_DEVICE_TYPE(16)
ZCAN_CANETTCP = ZCAN_DEVICE_TYPE(17)
ZCAN_PCIE_9220 = ZCAN_DEVICE_TYPE(18)
ZCAN_PCI5010U = ZCAN_DEVICE_TYPE(19)
ZCAN_USBCAN_E_U = ZCAN_DEVICE_TYPE(20)
ZCAN_USBCAN_2E_U = ZCAN_DEVICE_TYPE(21)
ZCAN_PCI5020U = ZCAN_DEVICE_TYPE(22)
ZCAN_EG20T_CAN = ZCAN_DEVICE_TYPE(23)
ZCAN_PCIE9221 = ZCAN_DEVICE_TYPE(24)
ZCAN_WIFICAN_TCP = ZCAN_DEVICE_TYPE(25)
ZCAN_WIFICAN_UDP = ZCAN_DEVICE_TYPE(26)
ZCAN_PCIe9120 = ZCAN_DEVICE_TYPE(27)
ZCAN_PCIe9110 = ZCAN_DEVICE_TYPE(28)
ZCAN_PCIe9140 = ZCAN_DEVICE_TYPE(29)
ZCAN_USBCAN_4E_U = ZCAN_DEVICE_TYPE(31)
ZCAN_CANDTU_200UR = ZCAN_DEVICE_TYPE(32)
ZCAN_CANDTU_MINI = ZCAN_DEVICE_TYPE(33)
ZCAN_USBCAN_8E_U = ZCAN_DEVICE_TYPE(34)
ZCAN_CANREPLAY = ZCAN_DEVICE_TYPE(35)
ZCAN_CANDTU_NET = ZCAN_DEVICE_TYPE(36)
ZCAN_CANDTU_100UR = ZCAN_DEVICE_TYPE(37)
ZCAN_PCIE_CANFD_100U = ZCAN_DEVICE_TYPE(38)
ZCAN_PCIE_CANFD_200U = ZCAN_DEVICE_TYPE(39)
ZCAN_PCIE_CANFD_400U = ZCAN_DEVICE_TYPE(40)
ZCAN_USBCANFD_200U = ZCAN_DEVICE_TYPE(41)
ZCAN_USBCANFD_100U = ZCAN_DEVICE_TYPE(42)
ZCAN_USBCANFD_MINI = ZCAN_DEVICE_TYPE(43)
ZCAN_CANFDCOM_100IE = ZCAN_DEVICE_TYPE(44)
ZCAN_CANSCOPE = ZCAN_DEVICE_TYPE(45)
ZCAN_CLOUD = ZCAN_DEVICE_TYPE(46)
ZCAN_CANDTU_NET_400 = ZCAN_DEVICE_TYPE(47)
ZCAN_VIRTUAL_DEVICE = ZCAN_DEVICE_TYPE(99)

'''
 Interface return status
'''
ZCAN_STATUS_ERR = 0
ZCAN_STATUS_OK = 1
ZCAN_STATUS_ONLINE = 2
ZCAN_STATUS_OFFLINE = 3
ZCAN_STATUS_UNSUPPORTED = 4

'''
 CAN type
'''
ZCAN_TYPE_CAN = c_uint(0)
ZCAN_TYPE_CANFD = c_uint(1)

'''
 Device information
'''


class ZCAN_DEVICE_INFO(Structure):
    _fields_ = [("hw_Version", c_ushort),
                ("fw_Version", c_ushort),
                ("dr_Version", c_ushort),
                ("in_Version", c_ushort),
                ("irq_Num", c_ushort),
                ("can_Num", c_ubyte),
                ("str_Serial_Num", c_ubyte * 20),
                ("str_hw_Type", c_ubyte * 40),
                ("reserved", c_ushort * 4)]

    def __str__(self):
        return "Hardware Version:%s\nFirmware Version:%s\nDriver Interface:%s\nInterface Interface:%s\nInterrupt Number:%d\nCAN Number:%d\nSerial:%s\nHardware Type:%s\n" % ( \
            self.hw_version, self.fw_version, self.dr_version, self.in_version, self.irq_num, self.can_num, self.serial,
            self.hw_type)

    def _version(self, version):
        return ("V%02x.%02x" if version // 0xFF >= 9 else "V%d.%02x") % (version // 0xFF, version & 0xFF)

    @property
    def hw_version(self):
        return self._version(self.hw_Version)

    @property
    def fw_version(self):
        return self._version(self.fw_Version)

    @property
    def dr_version(self):
        return self._version(self.dr_Version)

    @property
    def in_version(self):
        return self._version(self.in_Version)

    @property
    def irq_num(self):
        return self.irq_Num

    @property
    def can_num(self):
        return self.can_Num

    @property
    def serial(self):
        serial = ''
        for c in self.str_Serial_Num:
            if c > 0:
                serial += chr(c)
            else:
                break
        return serial

    @property
    def hw_type(self):
        hw_type = ''
        for c in self.str_hw_Type:
            if c > 0:
                hw_type += chr(c)
            else:
                break
        return hw_type


class _ZCAN_CHANNEL_CAN_INIT_CONFIG(Structure):
    _fields_ = [("acc_code", c_uint),
                ("acc_mask", c_uint),
                ("reserved", c_uint),
                ("filter", c_ubyte),
                ("timing0", c_ubyte),
                ("timing1", c_ubyte),
                ("mode", c_ubyte)]


class _ZCAN_CHANNEL_CANFD_INIT_CONFIG(Structure):
    _fields_ = [("acc_code", c_uint),
                ("acc_mask", c_uint),
                ("abit_timing", c_uint),
                ("dbit_timing", c_uint),
                ("brp", c_uint),
                ("filter", c_ubyte),
                ("mode", c_ubyte),
                ("pad", c_ushort),
                ("reserved", c_uint)]


class _ZCAN_CHANNEL_INIT_CONFIG(Union):
    _fields_ = [("can", _ZCAN_CHANNEL_CAN_INIT_CONFIG), ("canfd", _ZCAN_CHANNEL_CANFD_INIT_CONFIG)]


class ZCAN_CHANNEL_INIT_CONFIG(Structure):
    _fields_ = [("can_type", c_uint),
                ("config", _ZCAN_CHANNEL_INIT_CONFIG)]


class ZCAN_CHANNEL_ERR_INFO(Structure):
    _fields_ = [("error_code", c_uint),
                ("passive_ErrData", c_ubyte * 3),
                ("arLost_ErrData", c_ubyte)]


class ZCAN_CHANNEL_STATUS(Structure):
    _fields_ = [("errInterrupt", c_ubyte),
                ("regMode", c_ubyte),
                ("regStatus", c_ubyte),
                ("regALCapture", c_ubyte),
                ("regECCapture", c_ubyte),
                ("regEWLimit", c_ubyte),
                ("regRECounter", c_ubyte),
                ("regTECounter", c_ubyte),
                ("Reserved", c_ubyte)]


class ZCAN_CAN_FRAME(Structure):
    _fields_ = [("can_id", c_uint, 29),
                ("err", c_uint, 1),
                ("rtr", c_uint, 1),
                ("eff", c_uint, 1),
                ("can_dlc", c_ubyte),
                ("__pad", c_ubyte),
                ("__res0", c_ubyte),
                ("__res1", c_ubyte),
                ("data", c_ubyte * 8)]


class ZCAN_CANFD_FRAME(Structure):
    _fields_ = [("can_id", c_uint, 29),
                ("err", c_uint, 1),
                ("rtr", c_uint, 1),
                ("eff", c_uint, 1),
                ("len", c_ubyte),
                ("brs", c_ubyte, 1),
                ("esi", c_ubyte, 1),
                ("__res", c_ubyte, 6),
                ("__res0", c_ubyte),
                ("__res1", c_ubyte),
                ("data", c_ubyte * 64)]


class ZCAN_Transmit_Data(Structure):
    _fields_ = [("frame", ZCAN_CAN_FRAME), ("transmit_type", c_uint)]


class ZCAN_Receive_Data(Structure):
    _fields_ = [("frame", ZCAN_CAN_FRAME), ("timestamp", c_ulonglong)]


class ZCAN_TransmitFD_Data(Structure):
    _fields_ = [("frame", ZCAN_CANFD_FRAME), ("transmit_type", c_uint)]


class ZCAN_ReceiveFD_Data(Structure):
    _fields_ = [("frame", ZCAN_CANFD_FRAME), ("timestamp", c_ulonglong)]


class ZCAN_AUTO_TRANSMIT_OBJ(Structure):
    _fields_ = [("enable", c_ushort),
                ("index", c_ushort),
                ("interval", c_uint),
                ("obj", ZCAN_Transmit_Data)]


class ZCANFD_AUTO_TRANSMIT_OBJ(Structure):
    _fields_ = [("enable", c_ushort),
                ("index", c_ushort),
                ("interval", c_uint),
                ("obj", ZCAN_TransmitFD_Data)]


class IProperty(Structure):
    _fields_ = [("SetValue", c_void_p),
                ("GetValue", c_void_p),
                ("GetPropertys", c_void_p)]


class ZCAN(object):
    def __init__(self):
        if platform.system() == "Windows":
            try:
                # self.__dll = windll.LoadLibrary("C:/Users/tao.yongbiao/Desktop/新建文件夹/g112RC/zlgcan.dll")
                self.__dll = windll.LoadLibrary(os.path.join(os.path.dirname(__file__), "zlgcan.dll"))
                if not self.__dll:
                    raise FileNotFoundError("LoadLibrary 返回空句柄")
            except Exception as e:
                #print(f"[ERROR] 加载 zlgcan.dll 失败，请确认文件是否存在。错误信息: {e}")
                logger.error(f"[ERROR] 加载 zlgcan.dll 失败，请确认文件是否存在。错误信息: {e}")
                self.__dll = None
        else:
            #print("No support now!")
            logger.error("No support now!")

    def OpenDevice(self, device_type, device_index, reserved):
        try:
            return self.__dll.ZCAN_OpenDevice(device_type, device_index, reserved)
        except:
            #print("Exception on OpenDevice!")
            logger.error("Exception on OpenDevice!")
            
            raise

    def CloseDevice(self, device_handle):
        try:
            return self.__dll.ZCAN_CloseDevice(device_handle)
        except:
            #print("Exception on CloseDevice!")
            logger.error("Exception on CloseDevice!")
            raise

    def GetDeviceInf(self, device_handle):
        try:
            info = ZCAN_DEVICE_INFO()
            ret = self.__dll.ZCAN_GetDeviceInf(device_handle, byref(info))
            return info if ret == ZCAN_STATUS_OK else None
        except:
            #print("Exception on ZCAN_GetDeviceInf")
            logger.error("Exception on ZCAN_GetDeviceInf")
            raise

    def DeviceOnLine(self, device_handle):
        try:
            return self.__dll.ZCAN_IsDeviceOnLine(device_handle)
        except:
            #print("Exception on ZCAN_ZCAN_IsDeviceOnLine!")
            logger.error("Exception on ZCAN_ZCAN_IsDeviceOnLine!")
            raise

    def InitCAN(self, device_handle, can_index, init_config):
        try:
            return self.__dll.ZCAN_InitCAN(device_handle, can_index, byref(init_config))
        except:
            #print("Exception on ZCAN_InitCAN!")
            logger.error("Exception on ZCAN_InitCAN!")
            raise

    def StartCAN(self, chn_handle):
        try:
            #print("ZCAN_StartCAN 真实can 启动")
            logger.info("ZCAN_StartCAN 真实can 启动")
            return self.__dll.ZCAN_StartCAN(chn_handle)
        except:
            #print("Exception on ZCAN_StartCAN!")
            logger.error("Exception on ZCAN_StartCAN!")
            raise

    def ResetCAN(self, chn_handle):
        try:
            return self.__dll.ZCAN_ResetCAN(chn_handle)
        except:
            #print("Exception on ZCAN_ResetCAN!")
            logger.error("Exception on ZCAN_ResetCAN!")
            raise

    def ClearBuffer(self, chn_handle):
        try:
            return self.__dll.ZCAN_ClearBuffer(chn_handle)
        except:
            #print("Exception on ZCAN_ClearBuffer!")
            logger.error("Exception on ZCAN_ClearBuffer!")
            raise

    def ReadChannelErrInfo(self, chn_handle):
        try:
            ErrInfo = ZCAN_CHANNEL_ERR_INFO()
            ret = self.__dll.ZCAN_ReadChannelErrInfo(chn_handle, byref(ErrInfo))
            return ErrInfo if ret == ZCAN_STATUS_OK else None
        except:
            #print("Exception on ZCAN_ReadChannelErrInfo!")
            logger.error("Exception on ZCAN_ReadChannelErrInfo!")
            raise

    def ReadChannelStatus(self, chn_handle):
        try:
            status = ZCAN_CHANNEL_STATUS()
            ret = self.__dll.ZCAN_ReadChannelStatus(chn_handle, byref(status))
            return status if ret == ZCAN_STATUS_OK else None
        except:
            #print("Exception on ZCAN_ReadChannelStatus!")
            logger.error("Exception on ZCAN_ReadChannelStatus!")
            raise

    def GetReceiveNum(self, chn_handle, can_type=ZCAN_TYPE_CAN):
        try:
            return self.__dll.ZCAN_GetReceiveNum(chn_handle, can_type)
        except:
            #print("Exception on ZCAN_GetReceiveNum!")
            logger.error("Exception on ZCAN_GetReceiveNum!")
            raise

    def Transmit(self, chn_handle, std_msg, len):
        try:
            return self.__dll.ZCAN_Transmit(chn_handle, byref(std_msg), len)
        except:
            #print("Exception on ZCAN_Transmit!")
            logger.error("Exception on ZCAN_Transmit!")
            raise

    def Receive(self, chn_handle, rcv_num, wait_time=c_int(-1)):
        try:
            rcv_can_msgs = (ZCAN_Receive_Data * rcv_num)()
            ret = self.__dll.ZCAN_Receive(chn_handle, byref(rcv_can_msgs), rcv_num, wait_time)
            return rcv_can_msgs, ret
        except:
            #print("Exception on ZCAN_Receive!")
            logger.error("Exception on ZCAN_Receive!")
            raise

    def TransmitFD(self, chn_handle, fd_msg, len):
        try:
            return self.__dll.ZCAN_TransmitFD(chn_handle, byref(fd_msg), len)
        except:
            #print("Exception on ZCAN_TransmitFD!")
            logger.error("Exception on ZCAN_TransmitFD!")
            raise

    def ReceiveFD(self, chn_handle, rcv_num, wait_time=c_int(-1)):
        try:
            rcv_canfd_msgs = (ZCAN_ReceiveFD_Data * rcv_num)()
            ret = self.__dll.ZCAN_ReceiveFD(chn_handle, byref(rcv_canfd_msgs), rcv_num, wait_time)
            return rcv_canfd_msgs, ret
        except:
            #print("Exception on ZCAN_ReceiveFD!")
            logger.error("Exception on ZCAN_ReceiveFD!")
            raise

    def GetIProperty(self, device_handle):
        try:
            self.__dll.GetIProperty.restype = POINTER(IProperty)
            return self.__dll.GetIProperty(device_handle)
        except:
            #print("Exception on ZCAN_GetIProperty!")
            logger.error("Exception on ZCAN_GetIProperty!")
            raise

    def SetValue(self, iproperty, path, value):
        try:
            func = CFUNCTYPE(c_uint, c_char_p, c_char_p)(iproperty.contents.SetValue)
            return func(c_char_p(path.encode("utf-8")), c_char_p(value.encode("utf-8")))
        except:
            #print("Exception on IProperty SetValue")
            logger.error("Exception on IProperty SetValue")
            raise

    def GetValue(self, iproperty, path):
        try:
            func = CFUNCTYPE(c_char_p, c_char_p)(iproperty.contents.GetValue)
            return func(c_char_p(path.encode))
        except:
            #print("Exception on IProperty GetValue")
            logger.error("Exception on IProperty GetValue")
            raise

    def ReleaseIProperty(self, iproperty):
        try:
            return self.__dll.ReleaseIProperty(iproperty)
        except:
            #print("Exception on ZCAN_ReleaseIProperty!")
            logger.error("Exception on ZCAN_ReleaseIProperty!")
            raise


###############################################################################


'''
ENCODE AND DECODE
'''

def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc


def crc8(data):
    crc = sum(data) % 256
    crc = 255 - crc
    return crc


def encode_sbw_ready_frame():
    global G_READY_ROLL_CNT
    G_READY_ROLL_CNT = (G_READY_ROLL_CNT + 1) % 0xFFFF

    data_frame = bytearray(8)
    data_frame[0] = 0x00
    data_frame[1] = 0x01
    data_frame[7] = crc8(data_frame[:-1])

    return data_frame

# #将输入的扭矩值编码为一个具有校验、计数和固定格式的64字节数据帧，
# def encode_sbw_ffb_frame(torque):
#     global G_ROLL_CNT

#     data_frame = bytearray(64)

#     torque = int((torque + 20) * 50)

#     torque_bin = format(torque, '013b')

#     data_frame[0] = 0x20
#     data_frame[1] = 0x1C
#     data_frame[2] = 0x01
#     data_frame[3] = int(torque_bin[-4:], 2) << 4
#     data_frame[4] = int(torque_bin[1:-4], 2)

#     data_frame[5] = 0x4A
#     data_frame[6] = 0x40  #high 0000 1001 0x09#low 0000 0101 0x05

#     G_ROLL_CNT = (G_ROLL_CNT + 1) % 0xFFFF

#     data_frame[60] = G_ROLL_CNT & 0xFF
#     data_frame[61] = (G_ROLL_CNT >> 8) & 0xFF

#     crc_data = crc16(data_frame[:-2])

#     data_frame[62] = crc_data & 0xFF
#     data_frame[63] = (crc_data >> 8) & 0xFF

#     return data_frame



def encode_sbw_ffb_frame(torque=None):
    global G_ROLL_CNT

    data_frame = bytearray(64)

    # 如果没有提供扭矩值，则设置为0
    if torque is None:
        torque = 0

    torque = int((torque + 20) * 50)

    torque_bin = format(torque, '013b')

    data_frame[0] = 0x20
    data_frame[1] = 0x1C
    # 根据扭矩值设置data_frame[2]
    if torque == 0:
        data_frame[2] = 0x00
    else:
        data_frame[2] = 0x01
    data_frame[3] = int(torque_bin[-4:], 2) << 4
    data_frame[4] = int(torque_bin[1:-4], 2)

    data_frame[5] = 0x4A
    data_frame[6] = 0x40  #high 0000 1001 0x09#low 0000 0101 0x05

    G_ROLL_CNT = (G_ROLL_CNT + 1) % 0xFFFF

    data_frame[60] = G_ROLL_CNT & 0xFF
    data_frame[61] = (G_ROLL_CNT >> 8) & 0xFF

    crc_data = crc16(data_frame[:-2])

    data_frame[62] = crc_data & 0xFF
    data_frame[63] = (crc_data >> 8) & 0xFF

    return data_frame


def limit_and_encode_angle(angle: float, limit: float) -> int:
    # pitch 3.5 roll4
    limited = min(abs(angle), limit)
    signed_limited = limited if angle >= 0 else -limited
    return int((signed_limited + 1) * 90000)


def throttle_brake_response(vehicle_type, throttle, brake, speed):
    is_u7 = vehicle_type == 'U7'
    max_game_data = 0.02 if is_u7 else 0.08

    if speed < 0.5:
        return 0

    sign = 1 if brake > 10 else -1

    speed_factor = (
        0.4 if speed >= 200
        else 0.6 if speed >= 120
        else 0.8 if speed >= 60
        else 1.0
    )

    brake_factor = 0.6 if brake < 30 else 1.0

    pitch_add = max(throttle, brake) * 0.01 * max_game_data * speed_factor * brake_factor

    return pitch_add * sign

#该函数 encode_roll_pitch_frame 的主要功能是根据车辆状态和控制输入计算车身姿态（roll 和 pitch）的模拟数据，并将其编码为一个8字节的数据帧返回。
def encode_roll_pitch_frame(vehicle_type, throttle, brake, game_roll, game_pitch, speed, steering_wheel_angle,
                            steering_wheel_angle_old, steering_wheel_rate):
    if speed < 0.5:
        game_roll = 0
        game_pitch = 0

    # if game_roll > 0.04:
    #     game_roll = 0.04
    # if game_pitch > 0.04:
    #     game_pitch = 0.04

    roll_add = turning_response(vehicle_type,steering_wheel_angle, steering_wheel_angle_old, steering_wheel_rate, speed)
    pitch_add = throttle_brake_response(vehicle_type,throttle, brake, speed)

    is_u7 = vehicle_type == 'U7'
    roll_limit = 0.044 if is_u7 else 0.12
    pitch_limit = 0.039 if is_u7 else 0.12

    if is_u7:
        game_roll = min(abs(game_roll), 0.014)
        game_pitch = min(abs(game_pitch), 0.014)
    else :
        game_roll = min(abs(game_roll), 0.04)
        game_pitch = min(abs(game_pitch), 0.04)

    pitch_real = game_pitch * 90
#TODO: 需要明确roll_total_game和pitch_total_game
    roll_total_game = roll_add + game_roll
    pitch_total_game = game_pitch + pitch_add

    roll_raw_real = game_roll * 0.5 * 90
    roll_add_real = roll_total_game * 90


#TODO: 需要明确roll_total_game和pitch_total_game
    roll_int = limit_and_encode_angle(pitch_total_game, roll_limit)
    pitch_int = limit_and_encode_angle(roll_total_game, pitch_limit)

    pitch_real = pitch_total_game * 0.5 * 90

    data_frame = bytearray(8)

    pitch_bin = format(pitch_int, '020b')
    roll_bin = format(roll_int, '020b')

    data_frame[0] = 1
    data_frame[1] = int(roll_bin[12:20], 2)
    data_frame[2] = int(roll_bin[4:12], 2)
    data_frame[3] = int(roll_bin[:4], 2)

    data_frame[3] |= int(pitch_bin[16:20], 2) << 4
    data_frame[4] = int(pitch_bin[8:16], 2)
    data_frame[5] = int(pitch_bin[:8], 2)

    return data_frame, roll_raw_real, roll_add_real, pitch_real


def encode_switch_vr_frame(state):
    switch_vr_frame = bytearray(8)
    if state in (0x1, 0x2, 0x3):
        switch_vr_frame[0] = 1
        switch_vr_frame[1] = state

    return switch_vr_frame


def decode_steering_wheel_angle_frame(can_id, frame_data):
    if isinstance(frame_data, str):
        frame_data = [int(b, 16) for b in frame_data.strip().split()]

    if can_id == G_STEERING_CAN_ID:
        angle_raw = (frame_data[1] << 8) | frame_data[0]
        steering_rate = int(frame_data[2]) * 4
    else:
        angle_raw = (frame_data[6] << 8) | frame_data[5]
        steering_rate = int(frame_data[7]) * 4

    if angle_raw > 0x7FFF:
        angle_raw = (~angle_raw & 0xFFFF) + 1
        steering_wheel_angle = angle_raw / 10.0

    else:
        steering_wheel_angle = -angle_raw / 10.0

    return steering_wheel_angle, steering_rate


def turning_response(vehicle_type, steering_wheel_angle, steering_wheel_angle_old, steering_wheel_rate, speed):
    """
    根据方向盘转向速率和车速计算 roll 的附加效果，用于模拟转向反馈。
    
    参数:
    - vehicle_type (str): 车辆类型，如 'U7'，用于差异化处理
    - steering_wheel_angle (float): 当前方向盘角度
    - steering_wheel_angle_old (float): 上一次方向盘角度
    - steering_wheel_rate (float): 方向盘转速
    - speed (float): 当前车速

    返回:
    - roll_add (float): 计算出的 roll 附加值
    """
    # 将方向盘转速放大 6 倍以增强反馈效果
    steering_wheel_rate = steering_wheel_rate * 6

    # 如果方向盘转速过小或速度过低，返回 0.0（无反馈）
    if abs(steering_wheel_rate) < 50 or speed < 0.5:
        return 0.0

    # 根据车速设置速度因子
    if speed > 200:
        speed_factor = 0.6
    elif speed > 120:
        speed_factor = 0.4
    elif speed > 60:
        speed_factor = 0.2
    elif speed > 0.5:
        speed_factor = 0.1
    else:
        speed_factor = 1  # 默认值（理论上不会走到这里）

    # 判断方向盘角度变化方向
    delta_angle = steering_wheel_angle_old - steering_wheel_angle
    direction = -1 if delta_angle > 0 else 1  # 方向决定反馈正负

    # 根据车型设置最大游戏反馈系数
    if vehicle_type == 'U7':
        max_game_data = 0.02  # U7 车型反馈较弱
    else:
        max_game_data = 0.09  # 其他车型反馈较强

    # 对方向盘速率进行微调（缩放）
    steering_wheel_rate = steering_wheel_rate * 0.0009

    # 计算最终的 roll 附加值
    roll_add = direction * speed_factor * max_game_data * steering_wheel_rate
    return roll_add


def decode_throttle_brake_frame(frame_data):
    if isinstance(frame_data, str):
        frame_data = [int(b, 16) for b in frame_data.strip().split()]

    throttle = int(frame_data[0])
    brake = int(frame_data[1])
    return throttle, brake


def can_start(zcanlib, device_handle, chn):
    ip = zcanlib.GetIProperty(device_handle)
    ret = zcanlib.SetValue(ip, str(chn) + "/clock", "60000000")
    if ret != ZCAN_STATUS_OK:
        #print("Set CH%d CANFD clock failed!" % (chn))
        logger.error("Set CH%d CANFD clock failed!" % (chn))
    ret = zcanlib.SetValue(ip, str(chn) + "/canfd_standard", "0")
    if ret != ZCAN_STATUS_OK:
        #print("Set CH%d CANFD standard failed!" % (chn))
        logger.error("Set CH%d CANFD standard failed!" % (chn))
    ret = zcanlib.SetValue(ip, str(chn) + "/initenal_resistance", "1")
    if ret != ZCAN_STATUS_OK:
        #print("Open CH%d resistance failed!" % (chn))
        logger.error("Open CH%d resistance failed!" % (chn))
    zcanlib.ReleaseIProperty(ip)

    chn_init_cfg = ZCAN_CHANNEL_INIT_CONFIG()
    chn_init_cfg.can_type = ZCAN_TYPE_CANFD
    chn_init_cfg.config.canfd.abit_timing = 104286  # 101166 #1Mbps仲裁位速率
    chn_init_cfg.config.canfd.dbit_timing = 4260362  # 101166 #1Mbps数据位速率
    chn_init_cfg.config.canfd.mode = 0
    chn_handle = zcanlib.InitCAN(device_handle, chn, chn_init_cfg)
    if chn_handle is None:
        return None
    zcanlib.StartCAN(chn_handle)
    return chn_handle

#记录扭矩数据
def record_torque_data(start_time, desired_torque, damping, friction, total_torque, scale_torque,lateral_effect,suspension_effect):
    current_time = time.time() - start_time
    torque_data['time'].append(current_time)
    torque_data['desired_torque'].append(-desired_torque)
    torque_data['damping'].append(-damping)
    torque_data['friction'].append(-friction)
    torque_data['total_torque'].append(-total_torque)
    torque_data['scale_torque'].append(scale_torque)

    
    # 确保转向角数据也被记录
    torque_data['steering_angle'].append(G_STEERING_WHEEL_ANGLE)
    torque_data['steering_rate'].append(G_STEERING_RATE)
    torque_data['rate_dir'].append(G_RATE_DIR)
    
    
    
     # 添加lateral_effect记录（如果还不存在）
    if 'lateral_effect' not in torque_data:
        torque_data['lateral_effect'] = []
    torque_data['lateral_effect'].append(lateral_effect)

         # 添加suspension_effect记录（如果还不存在）
    if 'suspension_effect' not in torque_data:
        torque_data['suspension_effect'] = []
    torque_data['suspension_effect'].append(suspension_effect)


    # 控制最大数据量
    if len(torque_data['time']) > MAX_DATA_POINTS:
        for key in torque_data:
            if torque_data[key]:
                torque_data[key].pop(0)
#读取车辆状态
def  read_vehicle_status(ac_api):
    if ac_api:
        roll = ac_api.AC_GetRoll()
        pitch = ac_api.AC_GetPitch()
        speed = ac_api.AC_GetSpeedKmh()
        wheel_slip = ac_api.AC_GetWheelSlip()
        acc_g = ac_api.AC_GetAccG()
        suspension_travel = ac_api.AC_GetSuspensionTravel()
        local_angular_vel = ac_api.AC_GetLocalAngularVel()
        return roll, pitch, speed, wheel_slip, acc_g, suspension_travel, local_angular_vel
    else:
        return 0.0, 0.0, 0.0, None, None, None, None
def send_messages(chn_handle, ac_api, zcanlib):

    #初始化CAN FD消息结构和ID列表
    transmit_canfd_num = 7
    # transmit_canfd_num = 6
    canfd_msgs = (ZCAN_TransmitFD_Data * transmit_canfd_num)()

    can_ids = [0x0C3, 0x0C3, 0x0C3, 0x29F, 0x29F, 0x29F, 0x341]
    # can_ids = [0x0C3, 0x0C3, 0x0C3, 0x4EF, 0x4EF, 0x4EF, 0x341] #最新
    # can_ids = [0x0C3, 0x0C3, 0x0C3, 0x29F, 0x29F, 0x29F]

    for i, can_id in enumerate(can_ids):
        # 遍历 canfd_msgs 数组，初始化每个 CAN FD 消息的属性
        msg = canfd_msgs[i]  # 获取当前 CAN FD 消息对象

        # 设置消息的发送类型为正常发送（1 表示正常发送，0 表示保留或其他用途）
        msg.transmit_type = 1

        # 设置帧格式标志：
        # 0 表示标准帧（11位CAN ID），1 表示扩展帧（29位CAN ID）
        msg.frame.eff = 0

        # 设置远程帧标志：
        # 0 表示数据帧，1 表示请求帧（Remote Transmission Request）
        msg.frame.rtr = 0

        # 设置比特率切换标志：
        # 0 表示不切换比特率，1 表示使用更高的数据段比特率（CAN FD 特性）
        msg.frame.brs = 0

        # 设置数据长度码（DLC）：
        # 表示数据段长度为 8 字节（CAN FD 支持最大 64 字节）
        msg.frame.len = 8

        # 设置 CAN 帧 ID：
        # 从 can_ids 列表中取出当前索引的 CAN ID，标识该帧的用途或来源
        msg.frame.can_id = can_id

    start_time = time.time()  # 记录开始时间


    try:
        # if ac_api:
            # ac_api.AC_GetRoll.restype = ctypes.c_float
            # ac_api.AC_GetPitch.restype = ctypes.c_float


            # ac_api.AC_GetWheelSlip.restype = WheelSlip
            # ac_api.AC_GetAccG.restype = AccG
            # ac_api.AC_GetSuspensionTravel.restype = SuspensionTravel
            # ac_api.AC_GetLocalAngularVel.restype = LocalAngularVel

            #新增


            # class WheelSlip(ctypes.Structure):
            #     _fields_ = [("slip", ctypes.c_float * 4)]
            #
            # ac_api.AC_GetWheelSlip.restype = WheelSlip
            #
            # class AccG(ctypes.Structure):
            #     _fields_ = [("accg", ctypes.c_float * 3)]
            #
            # ac_api.AC_GetAccG.restype = AccG
            #
            # class SuspensionTravel(ctypes.Structure):
            #     _fields_ = [("st", ctypes.c_float * 4)]
            #
            # ac_api.AC_GetSuspensionTravel.restype = SuspensionTravel
            #
            # class LocalAngularVel(ctypes.Structure):
            #     _fields_ = [("VehAngVel", ctypes.c_float * 3)]
            #
            # ac_api.AC_GetLocalAngularVel.restype = LocalAngularVel

            # if not isinstance(ac_api, ACAPI): 
            #     ac_api.AC_GetWheelSlip.restype = WheelSlip
            #     ac_api.AC_GetAccG.restype = AccG
            #     ac_api.AC_GetSuspensionTravel.restype = SuspensionTravel
            #     ac_api.AC_GetLocalAngularVel.restype = LocalAngularVel
        cnt = 0
        while run_main_flag_event.is_set():

            roll = 0.0
            pitch = 0.0
            speed = 0.0

#===================================整合，从游戏中获取数据===========================================
            # if ac_api:
            #     roll = ac_api.AC_GetRoll()  # 获取游戏中的车辆滚转角
            #     pitch = ac_api.AC_GetPitch()
            #     speed = ac_api.AC_GetSpeedKmh()

                
            #     # #新增
            #     # try:
            #     #     wheel_slip = ac_api.AC_GetWheelSlip()
            #     #     acc_g = ac_api.AC_GetAccG()
            #     #     suspension_travel = ac_api.AC_GetSuspensionTravel()
            #     #     local_angular_vel = ac_api.AC_GetLocalAngularVel()
            #     # except Exception as e:
            #     #     print(f"[ERROR] 获取 WheelSlip 失败: {e}")

            #     wheel_slip = ac_api.AC_GetWheelSlip()
            #     acc_g = ac_api.AC_GetAccG()
            #     suspension_travel = ac_api.AC_GetSuspensionTravel()
            #     local_angular_vel = ac_api.AC_GetLocalAngularVel()
#=====================================================================================================
            
            if RC:
                roll, pitch, speed= read_vehicle_status_from_rc()
            else:
                roll, pitch, speed, wheel_slip, acc_g, suspension_travel, local_angular_vel = read_vehicle_status(ac_api)   

            # print(f"++++++++++++++++++++ acc_g: {acc_g.accg[1]}")

            # 编码滚转和俯仰数据帧
            roll_pitch_frame_data, roll_raw_real, roll_add_real, pitch_real = encode_roll_pitch_frame('U9', G_THROTTLE,
                                                            G_BRAKE, roll,
                                                            pitch, speed,
                                                            G_STEERING_WHEEL_ANGLE,
                                                            G_STEERING_WHEEL_ANGLE_OLD,
                                                            G_STEERING_RATE)
            # 将编码好的车身姿态数据（roll和pitch）填充到前3个CAN FD消息的数据域中
            for i in range(len(roll_pitch_frame_data)):
                for msg in canfd_msgs[:3]:  # 遍历前三个消息对象
                    msg.frame.data[i] = roll_pitch_frame_data[i]  # 填充数据到每个消息帧的对应位置

            # 计数器 cnt 自增1，用于控制某些操作的频率
            cnt += 1

            # 每当 cnt 达到5时，执行一次特定操作（发送VR模式切换帧）
            if cnt == 5:
                cnt = 0  # 重置计数器
                
                # 调用 encode_switch_vr_frame 函数生成一个VR模式切换帧，参数0x2表示某种特定状态
                switch_vr_frame_data = encode_switch_vr_frame(0x2)
                
                # 将VR模式切换帧的数据填充到第4至第6个CAN FD消息的数据域中
                for i in range(len(switch_vr_frame_data)):
                    for msg in canfd_msgs[3:-1]:  # 遍历第4到第6个消息对象（索引3到-1前一个）
                        msg.frame.data[i] = switch_vr_frame_data[i]

            ready_frame = encode_sbw_ready_frame()
            for i in range(len(ready_frame)):
                canfd_msgs[6].frame.data[i] = ready_frame[i]
            #修改 在没有can时也能启用
            if chn_handle is not None:
                roll_pitch_steer_ready_ret = zcanlib.TransmitFD(chn_handle, canfd_msgs, transmit_canfd_num)
                #print("roll,pitch", roll_pitch_steer_ready_ret)
                logger.info("roll,pitch", roll_pitch_steer_ready_ret)




            # total_torque =(desired_torque+ friction + damping + tire_effect + lateral_effect + road_effect)*-1



            # lateral_effect = ffb.get_lateral_effect(speed,acc_g,ctypes.c_float(G_STEERING_WHEEL_ANGLE), ctypes.c_float(G_STEERING_RATE/1080.0),wheel_slip)





            #从新函数得到数据
            ffb=ForceFeedbackAlgorithm()
                
            desired_torque=ffb.get_tanh_torque(speed,G_STEERING_WHEEL_ANGLE)
            # desired_torque = ffb.get_tanh_torque(speed, local_angular_vel.VehAngVel[2])

            # 0630函数异常影响力矩输出

            lateral_effect = ffb.get_lateral_effect(
                speed,
                acc_g,  # 提取数组部分
                local_angular_vel,
                wheel_slip.slip
            )
            suspension_effect=ffb.get_suspension_effect(speed, suspension_travel)

            # friction, damping = ffb.get_friction(ctypes.c_float(G_STEERING_WHEEL_ANGLE), ctypes.c_float(G_STEERING_RATE), speed)
            friction, damping = ffb.get_friction(ctypes.c_float(G_STEERING_WHEEL_ANGLE),
                                                 ctypes.c_float(G_STEERING_RATE))
            total_torque = desired_torque - damping - friction+lateral_effect+suspension_effect

            logger.info(f"desired_torque: {desired_torque:.2f}, damping: {damping:.2f}, friction: {friction:.2f}, total: {total_torque:.2f}")

            scale_torque = total_torque * 0.001 * 0.05 * -1  # 54330.05重 #另一台u9是0.06
    
            # if config.USE_GAME_FFB_SOURCE:
            #     result, error = get_ffb_from_game_api(g112RC_api, G_STEERING_WHEEL_ANGLE, ctypes.c_float(G_STEERING_RATE))
            # else:
            #     result, error = get_ffb_from_algorithm(speed, G_STEERING_WHEEL_ANGLE, ctypes.c_float(G_STEERING_RATE))

            # if result:
            #     desired_torque = result['desired_torque']
            #     damping = result['damping']
            #     friction = result['friction']
            #     total_torque = result['total_torque']
            #     scale_torque = result['scale_torque']
            # else:
            #     print(f"[ERROR] {'Game API' if config.USE_GAME_FFB_SOURCE else 'Algorithm'} FFB 获取失败: {error}")
            #     desired_torque = damping = friction = total_torque = scale_torque = 0.0
            
            
            
            
            
            
            
            
            
            
            
            record_torque_data(start_time, desired_torque, damping, friction, total_torque, scale_torque,lateral_effect,suspension_effect)
            print(f"desired: {desired_torque:.2f}, damping: {damping:.2f}, friction: {friction:.2f}, "
                  f"total: {total_torque:.2f}, scale: {scale_torque:.6f}")


            #print("scale_torque", scale_torque)
            logger.info(f"scale_torque: {scale_torque:.6f}")

            ffb_frame_data = encode_sbw_ffb_frame(scale_torque)

            steer_canfd_msgs = ZCAN_TransmitFD_Data()

            # 设置 CAN FD 消息的发送类型为正常发送（transmit_type=1）
            steer_canfd_msgs.transmit_type = 1

            # frame.eff: 帧格式标志，0 表示标准帧（11位ID），1 表示扩展帧（29位ID）
            steer_canfd_msgs.frame.eff = 0

            # frame.rtr: 远程帧标志，0 表示数据帧，1 表示请求帧（Remote Transmission Request）
            steer_canfd_msgs.frame.rtr = 0

            # frame.brs: 比特率切换标志，0 表示不切换比特率，1 表示使用更高的数据段比特率（CAN FD 特性）
            steer_canfd_msgs.frame.brs = 0

            # frame.len: 数据长度码（DLC），表示数据段长度为 64 字节（CAN FD 最大支持 64 字节）
            steer_canfd_msgs.frame.len = 64

            # frame.can_id: 设置 CAN 帧 ID 为 0x57（十进制 87），用于标识方向盘力反馈数据
            steer_canfd_msgs.frame.can_id = 0x57
            # steer_canfd_msgs.frame.can_id = 0x0C3

            for i in range(len(ffb_frame_data)):
                steer_canfd_msgs.frame.data[i] = ffb_frame_data[i]

            # 修改 在没有can时也能启用
            if chn_handle is not None:
                ret_steer = zcanlib.TransmitFD(chn_handle, steer_canfd_msgs, 1)
                #print("steer", ret_steer)
                logger.debug("steer", ret_steer)

            time.sleep(0.1)
            # time.sleep(0.005)

    except KeyboardInterrupt:
        #print("Send interrupted by user")
        logger.error("Send interrupted by user")
    # finally:
    #     ac_api.AC_Close()


def receive_messages(chn_handle, zcanlib):
    global G_STEERING_WHEEL_ANGLE_OLD, G_STEERING_WHEEL_ANGLE, G_STEERING_RATE, G_RATE_DIR
    try:
        while run_main_flag_event.is_set():


            # 油门 0630屏蔽固定油门踏板
            # brake = 0
            # throttle = 100
            # pyvjoy.VJoyDevice(1).set_axis(pyvjoy.HID_USAGE_Y, int(brake * 3 / 100 * 32767))
            # pyvjoy.VJoyDevice(1).set_axis(pyvjoy.HID_USAGE_X, int(throttle / 100 * 32767))
            # rcv_num = zcanlib.GetReceiveNum(chn_handle, ZCAN_TYPE_CAN)

            rcv_canfd_num = zcanlib.GetReceiveNum(chn_handle, ZCAN_TYPE_CANFD) #获取当前帧队列长度

            # 在 while 循环中更新
            # torque_data['steering_angle_old'].append(G_STEERING_WHEEL_ANGLE_OLD)
            torque_data['steering_angle'].append(G_STEERING_WHEEL_ANGLE)
            torque_data['steering_rate'].append(G_STEERING_RATE)
            torque_data['rate_dir'].append(G_RATE_DIR)

            # 控制长度
            for key in [ 'steering_angle', 'steering_rate', 'rate_dir']:
                if len(torque_data[key]) > MAX_DATA_POINTS:
                    try:
                        torque_data[key].pop(0)
                    except IndexError:
                        pass
                    

            if rcv_canfd_num:
                rcv_canfd_msgs, rcv_canfd_num = zcanlib.ReceiveFD(chn_handle, rcv_canfd_num, 1000)
                for i in range(rcv_canfd_num):
                    frame = rcv_canfd_msgs[i].frame
                    can_id = frame.can_id
                    data = frame.data

                    # 油门


                    #if can_id == G_STEERING_CAN_ID or
                    # if can_id == G_STEERING_SBW_CAN_ID:
                    #     G_STEERING_WHEEL_ANGLE_OLD = G_STEERING_WHEEL_ANGLE
                    #
                    #     steering_angle, G_STEERING_RATE = decode_steering_wheel_angle_frame(can_id, data)
                    #
                    #     delta_angle = G_STEERING_WHEEL_ANGLE_OLD - G_STEERING_WHEEL_ANGLE
                    #     G_RATE_DIR = 1 if delta_angle > 0 else -1
                    #     G_STEERING_RATE = G_STEERING_RATE * G_RATE_DIR
                    #
                    #     steering_wheel_factor = 1.6
                    #     G_STEERING_WHEEL_ANGLE = steering_angle * steering_wheel_factor

                    if can_id == G_STEERING_SBW_CAN_ID:
                        G_STEERING_WHEEL_ANGLE_OLD = G_STEERING_WHEEL_ANGLE

                        steering_angle, G_STEERING_RATE = decode_steering_wheel_angle_frame(can_id, data)
                        steering_wheel_factor = 1.6
                        G_STEERING_WHEEL_ANGLE = steering_angle * steering_wheel_factor
                        #print(f"G_STEERING_WHEEL_ANGLE_OLD: {G_STEERING_WHEEL_ANGLE_OLD}")
                        logger.debug(f"G_STEERING_WHEEL_ANGLE_OLD: {G_STEERING_WHEEL_ANGLE_OLD}")
                        #print(f"G_STEERING_WHEEL_ANGLE: {G_STEERING_WHEEL_ANGLE}")
                        logger.debug(f"G_STEERING_WHEEL_ANGLE: {G_STEERING_WHEEL_ANGLE}")
                        delta_angle = G_STEERING_WHEEL_ANGLE_OLD - G_STEERING_WHEEL_ANGLE
                        G_RATE_DIR = 1 if delta_angle > 0 else -1
                        #print(f"G_RATE_DIR: {G_RATE_DIR}")
                        logger.debug(f"G_RATE_DIR: {G_RATE_DIR}")

                        G_STEERING_RATE = G_STEERING_RATE * G_RATE_DIR
                        if RC:
                            rc_inputdata = FrameInputData()
                            rc_inputdata.steer = steering_angle / 360
                            #print(f"rc_inputdata.steer: {rc_inputdata.steer}")
                            logger.debug(f"rc_inputdata.steer: {rc_inputdata.steer}") 
                             
                            fWriteInput(rc_inputdata)
                        else:
                            pyvjoy.VJoyDevice(1).set_axis(pyvjoy.HID_USAGE_Z,
                                        int((G_STEERING_WHEEL_ANGLE + 720) / 1440 * 32767))
                            logger.debug(f"steering_angle: {steering_angle}")

                    if can_id == G_THROTTLE_BRAKE_CAN_ID:
                        throttle, brake = decode_throttle_brake_frame(data)
                        # brake = 0
                        # throttle = 200
                        #print(f"throttle:{throttle}    brake:{brake}\n")
                        logger.debug(f"throttle:{throttle}    brake:{brake}\n")
                        if RC:
                            rc_inputdata = FrameInputData()
                            rc_inputdata.throttle = throttle/ 100
                            rc_inputdata.brake = brake/ 100
                            #print(f"rc_inputdata.throttle:{rc_inputdata.throttle}\n")
                            logger.debug(f"rc_inputdata.throttle:{rc_inputdata.throttle}\n")
                            #print(f"rc_inputdata.brake:{rc_inputdata.brake}\n")
                            logger.debug(f"rc_inputdata.brake:{rc_inputdata.brake}\n")
                            fWriteInput(rc_inputdata)
                        else:
                            pyvjoy.VJoyDevice(1).set_axis(pyvjoy.HID_USAGE_Y, int(brake * 3 / 100 * 32767))
                            pyvjoy.VJoyDevice(1).set_axis(pyvjoy.HID_USAGE_X, int(throttle / 100 * 32767))
            else:
                pass
                # print("wait")
                # break
    except KeyboardInterrupt:
        #print("Rec interrupted by user")
        logger.error("Rec interrupted by user")






def update_g_vars(angle, rate, direction):
    global G_STEERING_WHEEL_ANGLE_OLD, G_STEERING_WHEEL_ANGLE, G_STEERING_RATE, G_RATE_DIR

    G_STEERING_WHEEL_ANGLE_OLD = G_STEERING_WHEEL_ANGLE
    G_STEERING_WHEEL_ANGLE = angle
    G_STEERING_RATE = rate
    G_RATE_DIR = direction

    # torque_data['steering_angle_old'].append(G_STEERING_WHEEL_ANGLE_OLD)
    torque_data['steering_angle'].append(G_STEERING_WHEEL_ANGLE)
    torque_data['steering_rate'].append(G_STEERING_RATE)
    torque_data['rate_dir'].append(G_RATE_DIR)

    for key in [ 'steering_angle', 'steering_rate', 'rate_dir']:
        if len(torque_data[key]) > MAX_DATA_POINTS:
            torque_data[key].pop(0)


def initialize_can(config, window=None):


    use_real_can = config.get('USE_REAL_CAN', False)

    # 初始化 ZCAN 类型（仅第一次有效）
    _init_zcan(use_real_can)

    # 根据配置选择实际类
    ZCAN = _mock_zcan if not use_real_can else _real_zcan

    try :
        zcanlib = ZCAN()
    except Exception as e:
        #print("Error initializing ZCAN:", e)
        logger.error("Error initializing ZCAN:", e)
        exit(1)
    if window is not None:
        window.set_zcanlib(zcanlib)

    if config['USE_REAL_CAN']:
        handle = zcanlib.OpenDevice(ZCAN_USBCANFD_MINI, 0, 0)
        if handle == INVALID_DEVICE_HANDLE:
            #print("Open Device failed!")
            logger.error("Open Device failed!")
            exit(0)
        #print(f"device handle: {handle}")
        logger.debug(f"device handle: {handle}")

        info = zcanlib.GetDeviceInf(handle)
        #print(f"Device Info:\n{info}")
        logger.debug(f"Device Info:\n{info}")

        chn_handle = can_start(zcanlib, handle, 0)
        #print(f"channel handle: {chn_handle}")
        logger.debug(f"channel handle: {chn_handle}")
    else:
        # 使用 MockZCAN 模拟 CAN 设备
        handle = "simulated_device"
        chn_handle = "simulated_channel_0"
        zcanlib.StartCAN(chn_handle)
        #print("+++++++++++++++++++++++++Running in simulation mode without ZCAN device.++++++++++++++++")
        logger.info("+++++++++++++++++++++++++Running in simulation mode without ZCAN device.++++++++++++++++")
        # 设置回调
        zcanlib.on_steering_update = lambda a, r, d: update_g_vars(a, r, d)

    return zcanlib, chn_handle,handle

def start_main_process(config,window=None):
    global main_thread_list,fRead,fWriteInput,fEnd,ac_api,RC

    # 清空旧线程列表
    main_thread_list.clear()

    if config['USE_REAL_AC']:

        if config['USE_RC']:
           # 初始化 DLL 接口
            # if not ffb_rc.init_game_api():
            #     exit(1) 
            # 获取函数引用
            ffb_rc.init_game_api()
            fRead = ffb_rc.get_fRead()
            fEnd = ffb_rc.get_fEnd()
            fWriteInput=ffb_rc.get_fWriteInput()
            RC = True
        else :
            RC= False
            # dll_path = "C:/Users/tao.yongbiao/Desktop/新建文件夹/g112RC/3_ac_api.dll"
            #
            ac_api = windll.LoadLibrary(os.path.join(os.path.dirname(__file__), "4_ac_api.dll"))
            # ac_api = ctypes.WinDLL(dll_path)

            ac_api.AC_GetRoll.restype = ctypes.c_float
            ac_api.AC_GetPitch.restype = ctypes.c_float

            ac_api.AC_StartUpdate.restype = ctypes.c_int
            ac_api.AC_GetSpeedKmh.restype = ctypes.c_float
            ac_api.AC_GetSteerAngle.restype = ctypes.c_float
            ac_api.AC_FFB_GetForceFeedbackWithEffects.restype = ForceFeedbackOutput

            class WheelSlip(ctypes.Structure):
                _fields_ = [("slip" , ctypes.c_float*4)]
            ac_api.AC_GetWheelSlip.restype = WheelSlip


            class AccG(ctypes.Structure):
                _fields_ = [("accg", ctypes.c_float * 3)]

            # class AccG(ctypes.Structure):
            #     _fields_ = [("x", ctypes.c_float),
            #                 ("y", ctypes.c_float),
            #                 ("z", ctypes.c_float)]
            ac_api.AC_GetAccG.restype = AccG

            class SuspensionTravel(ctypes.Structure):
                _fields_ = [("st", ctypes.c_float * 4)]
            ac_api.AC_GetSuspensionTravel.restype = SuspensionTravel

            class LocalAngularVel(ctypes.Structure):
                _fields_ = [("VehAngVel", ctypes.c_float * 3)]
            ac_api.AC_GetLocalAngularVel.restype = LocalAngularVel

            if ac_api.AC_StartUpdate() != 0:
                #print("请先打开神力科莎")
                logger.error("请先打开神力科莎")
                exit(1)
    else:
        #print("Using mock ac_api.")
        logger.info("Using mock ac_api.")
        ac_api = ACAPI()

    # if config['USE_REAL_CAN']:
    #     pass
    # else:
    #     from MockZCAN import MockZCAN as ZCAN #使用了覆盖机制来覆盖默认的ZCAN类

    # use_real_can = config.get('USE_REAL_CAN', False)

    # # 初始化 ZCAN 类型（仅第一次有效）
    # _init_zcan(use_real_can)

    # # 根据配置选择实际类
    # ZCAN = _mock_zcan if not use_real_can else _real_zcan


    # zcanlib = ZCAN()
    # if window is not None:
    #     window.set_zcanlib(zcanlib)

    # if config['USE_REAL_CAN']:
    #     handle = zcanlib.OpenDevice(ZCAN_USBCANFD_MINI, 0, 0)
    #     if handle == INVALID_DEVICE_HANDLE:
    #         print("Open Device failed!")
    #         exit(0)
    #     print(f"device handle: {handle}")

    #     info = zcanlib.GetDeviceInf(handle)
    #     print(f"Device Info:\n{info}")

    #     chn_handle = can_start(zcanlib, handle, 0)
    #     print(f"channel handle: {chn_handle}")
    # else:
    #     # 使用 MockZCAN 模拟 CAN 设备
    #     handle = "simulated_device"  # 虚拟设备句柄
    #     chn_handle = "simulated_channel_0"  # 虚拟通道句柄
    #     zcanlib.StartCAN(chn_handle)
    #     print("+++++++++++++++++++++++++Running in simulation mode without ZCAN device.++++++++++++++++")
    #     # 设置回调 当有新的方向盘角度数据到达时（例如：通过 inject_frame() 注入模拟帧），MockZCAN 会触发 on_steering_update(angle, rate, direction)；
    #     zcanlib.on_steering_update = lambda a, r, d: update_g_vars(a, r, d)

    if config['USE_WIFI']:
        # 创建线程

        send_broadcast_thread = threading.Thread(target=wifi_module.wifi_send_broadcast_messages)
        sender_heartbeat_thread = threading.Thread(target=wifi_module.wifi_send_heartbeat_messages)
        sender_thread = threading.Thread(target=wifi_module.wifi_send_messages, args=(ac_api,))
        receiver_thread = threading.Thread(target=wifi_module.wifi_receive_messages, )
        heartbeat_monitor_thread = threading.Thread(target=wifi_module.wifi_heartbeat_monitor)
        #线程状态
        running_threads = [
            receiver_thread,
            send_broadcast_thread,
            heartbeat_monitor_thread,
            sender_heartbeat_thread,
            sender_thread
        ]
    else:
        # 初始化 CAN
        zcanlib, chn_handle,handle = initialize_can(config, window)
        if window is not None:
            window.chn_handle = chn_handle  # 将 chn_handle 传递给 GUI
        sender_thread = threading.Thread(target=send_messages, args=(chn_handle, ac_api, zcanlib))
        receiver_thread = threading.Thread(target=receive_messages, args=(chn_handle, zcanlib))
        #线程状态
        running_threads = [sender_thread, receiver_thread]
        
  

    #启动所有线程    
    for t in running_threads:
        t.daemon = True
        t.start()
    
    # # 将线程列表传入 GUI 层（假设 window 是全局可用的）
    # if hasattr(window, 'set_running_threads'):
    #     window.set_running_threads(running_threads)
    
    


    # 新开线程用于实时绘图
    # time.sleep(2)
    # plot_thread = threading.Thread(target=real_time_plot)
    # plot_thread.daemon = True  # 设置为守护线程，主程序退出时自动关闭
    # plot_thread.start()


    # receiver_thread.join()
    # if config['USE_WIFI']:
    #     send_broadcast_thread.join()
    #     heartbeat_monitor_thread.join()
    #     sender_heartbeat_thread.join()
    # sender_thread.join()

    # 非 GUI 模式下等待线程结束
    try:
        for t in running_threads:
            t.join()
    except KeyboardInterrupt:
        #print("Main process interrupted by user.")
        logger.error("Main process interrupted by user.")



        # 绘图展示数据
    # plot_torque_data()


    # zcanlib.ResetCAN(chn_handle)
    # zcanlib.CloseDevice(handle)
        
    # 清理资源
    if not config['USE_WIFI']:
        zcanlib.ResetCAN(chn_handle)
        zcanlib.CloseDevice(handle)

    # sys.exit(app.exec_())



# 定义事件对象
# config_ready_event = threading.Event()
if __name__ == "__main__":

    config = {
    'USE_WIFI': False,
    'USE_REAL_CAN': False,
    'USE_REAL_AC': False,
    "USE_RC": False,
    }
    config_ready_event = threading.Event()

    


    # 判断是否不启用 GUI
    no_gui = "--no" in sys.argv or "-n" in sys.argv
    if no_gui:
        #print("Running without GUI...")
        logger.info("Running without GUI...")
        start_main_process(config)
    else:


        
        from PySide6.QtWidgets import QApplication
        from PySide6.QtCore import QTimer
        from gui import RealTimePlotWindow
        

        app = QApplication(sys.argv)
        window = RealTimePlotWindow(config_ready_event, config=config)
        window.show()
        

    #     print("请在绘图界面中设置 USE_WIFI / USE_REAL_CAN / USE_REAL_AC 变量并点击 Confirm 开始...")
    # # config_ready_event.wait()  # 等待用户点击 "Confirm"

    #     def wait_for_config():
    #         if config_ready_event.is_set():
    #             print("开始执行主流程，当前配置：")
    #             print(f"USE_WIFI: {config['USE_WIFI']}, USE_REAL_CAN: {config['USE_REAL_CAN']}, USE_REAL_AC: {config['USE_REAL_AC']}")
    #             # start_main_process(config)
    #                     # 在新线程中启动主逻辑
    #             # 在新线程中启动主逻辑
    #             main_thread = threading.Thread(target=start_main_process, args=(config,))
    #             main_thread.daemon = True
    #             main_thread.start()
    #         else:
    #             QTimer.singleShot(100, wait_for_config)  # 每隔 100ms 再次检查

    #     # wait_for_config()
    #     QTimer.singleShot(100, wait_for_config)
    #     print("开始执行主流程，当前配置：")
    #     print(f"USE_WIFI: {config['USE_WIFI']}, USE_REAL_CAN: {config['USE_REAL_CAN']}, USE_REAL_AC: {config['USE_REAL_AC']}")
        # sys.exit(app.exec_())
        sys.exit(app.exec())

    
