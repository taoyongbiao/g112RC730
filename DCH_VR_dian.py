from ctypes import *
import platform
import ctypes
import time
import threading
#import pyvjoy
import os

# 巅峰极速结构体
# 定义嵌套结构体
class FrameSuspensionData(ctypes.Structure):
    _pack_ = 4
    _fields_ = [
        ("height", ctypes.c_float),
        ("velocity", ctypes.c_float),
    ]

class FrameTyreData(ctypes.Structure):
    _pack_ = 4
    _fields_ = [
        ("slipRatio", ctypes.c_float),
        ("slipAngle", ctypes.c_float),
        ("load", ctypes.c_float),
        ("rotationRate", ctypes.c_float),
    ]

# 定义主结构体
class FrameData(ctypes.Structure):
    _pack_ = 4
    _fields_ = [
        ("clock", ctypes.c_float),
        ("throttle", ctypes.c_float),
        ("brake", ctypes.c_float),
        ("gear", ctypes.c_int),
        ("rpm", ctypes.c_float),
        ("steering", ctypes.c_float),
        ("speed", ctypes.c_float),

        ("velocity", ctypes.c_float * 3),
        ("acceleration", ctypes.c_float * 3),

        ("yaw", ctypes.c_float),
        ("pitch", ctypes.c_float),
        ("roll", ctypes.c_float),

        ("comy", ctypes.c_float),

        ("tyreData", FrameTyreData * 4),
        ("suspensionData", FrameSuspensionData * 4),

        ("angular", ctypes.c_float * 3),

        ("extraMessage", ctypes.c_uint),
        ("running", ctypes.c_uint),
    ]

class FrameInputData(ctypes.Structure):
    _pack_ = 4
    _fields_ = [
        ("steer", c_float),
        ("throttle", c_float),
        ("brake", c_float),
        ("handbrake", c_float),
        ("clutch", c_float),
        ("gear", c_int),
    ]

# 定义函数指针类型
RCInit = WINFUNCTYPE(None, c_bool)
RCReadFrameData = WINFUNCTYPE(None, ctypes.POINTER(FrameData))
RCSetInputData = WINFUNCTYPE(None, ctypes.POINTER(FrameInputData))
RCEnd = WINFUNCTYPE(None)

G_RC_RUNING_STATUS = 0

G_ROLL_CNT = 0
G_READY_ROLL_CNT = 0

ZCAN_DEVICE_TYPE = c_uint
G_STEERING_RATE = 0.0
G_STEERING_WHEEL_ANGLE = 0.0
G_STEERING_WHEEL_ANGLE_OLD = 0.0
G_THROTTLE = 0.0
G_BRAKE = 0.0

G_STEERING_CAN_ID = 0X11F
G_STEERING_SBW_CAN_ID = 0X8E
G_THROTTLE_BRAKE_CAN_ID = 0x342

INVALID_DEVICE_HANDLE  = 0
INVALID_CHANNEL_HANDLE = 0

'''
 Device Type
'''
ZCAN_PCI5121          = ZCAN_DEVICE_TYPE(1) 
ZCAN_PCI9810          = ZCAN_DEVICE_TYPE(2) 
ZCAN_USBCAN1          = ZCAN_DEVICE_TYPE(3) 
ZCAN_USBCAN2          = ZCAN_DEVICE_TYPE(4) 
ZCAN_PCI9820          = ZCAN_DEVICE_TYPE(5) 
ZCAN_CAN232           = ZCAN_DEVICE_TYPE(6) 
ZCAN_PCI5110          = ZCAN_DEVICE_TYPE(7) 
ZCAN_CANLITE          = ZCAN_DEVICE_TYPE(8) 
ZCAN_ISA9620          = ZCAN_DEVICE_TYPE(9) 
ZCAN_ISA5420          = ZCAN_DEVICE_TYPE(10)
ZCAN_PC104CAN         = ZCAN_DEVICE_TYPE(11)
ZCAN_CANETUDP         = ZCAN_DEVICE_TYPE(12)
ZCAN_CANETE           = ZCAN_DEVICE_TYPE(12)
ZCAN_DNP9810          = ZCAN_DEVICE_TYPE(13)
ZCAN_PCI9840          = ZCAN_DEVICE_TYPE(14)
ZCAN_PC104CAN2        = ZCAN_DEVICE_TYPE(15)
ZCAN_PCI9820I         = ZCAN_DEVICE_TYPE(16)
ZCAN_CANETTCP         = ZCAN_DEVICE_TYPE(17)
ZCAN_PCIE_9220        = ZCAN_DEVICE_TYPE(18)
ZCAN_PCI5010U         = ZCAN_DEVICE_TYPE(19)
ZCAN_USBCAN_E_U       = ZCAN_DEVICE_TYPE(20)
ZCAN_USBCAN_2E_U      = ZCAN_DEVICE_TYPE(21)
ZCAN_PCI5020U         = ZCAN_DEVICE_TYPE(22)
ZCAN_EG20T_CAN        = ZCAN_DEVICE_TYPE(23)
ZCAN_PCIE9221         = ZCAN_DEVICE_TYPE(24)
ZCAN_WIFICAN_TCP      = ZCAN_DEVICE_TYPE(25)
ZCAN_WIFICAN_UDP      = ZCAN_DEVICE_TYPE(26)
ZCAN_PCIe9120         = ZCAN_DEVICE_TYPE(27)
ZCAN_PCIe9110         = ZCAN_DEVICE_TYPE(28)
ZCAN_PCIe9140         = ZCAN_DEVICE_TYPE(29)
ZCAN_USBCAN_4E_U      = ZCAN_DEVICE_TYPE(31)
ZCAN_CANDTU_200UR     = ZCAN_DEVICE_TYPE(32)
ZCAN_CANDTU_MINI      = ZCAN_DEVICE_TYPE(33)
ZCAN_USBCAN_8E_U      = ZCAN_DEVICE_TYPE(34)
ZCAN_CANREPLAY        = ZCAN_DEVICE_TYPE(35)
ZCAN_CANDTU_NET       = ZCAN_DEVICE_TYPE(36)
ZCAN_CANDTU_100UR     = ZCAN_DEVICE_TYPE(37)
ZCAN_PCIE_CANFD_100U  = ZCAN_DEVICE_TYPE(38)
ZCAN_PCIE_CANFD_200U  = ZCAN_DEVICE_TYPE(39)
ZCAN_PCIE_CANFD_400U  = ZCAN_DEVICE_TYPE(40)
ZCAN_USBCANFD_200U    = ZCAN_DEVICE_TYPE(41)
ZCAN_USBCANFD_100U    = ZCAN_DEVICE_TYPE(42)
ZCAN_USBCANFD_MINI    = ZCAN_DEVICE_TYPE(43)
ZCAN_CANFDCOM_100IE   = ZCAN_DEVICE_TYPE(44)
ZCAN_CANSCOPE         = ZCAN_DEVICE_TYPE(45)
ZCAN_CLOUD            = ZCAN_DEVICE_TYPE(46)
ZCAN_CANDTU_NET_400   = ZCAN_DEVICE_TYPE(47)
ZCAN_VIRTUAL_DEVICE   = ZCAN_DEVICE_TYPE(99)

'''
 Interface return status
'''
ZCAN_STATUS_ERR         = 0
ZCAN_STATUS_OK          = 1
ZCAN_STATUS_ONLINE      = 2
ZCAN_STATUS_OFFLINE     = 3
ZCAN_STATUS_UNSUPPORTED = 4

'''
 CAN type
'''
ZCAN_TYPE_CAN    = c_uint(0)
ZCAN_TYPE_CANFD  = c_uint(1)

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
        return "Hardware Version:%s\nFirmware Version:%s\nDriver Interface:%s\nInterface Interface:%s\nInterrupt Number:%d\nCAN Number:%d\nSerial:%s\nHardware Type:%s\n" %( \
                self.hw_version, self.fw_version, self.dr_version, self.in_version, self.irq_num, self.can_num, self.serial, self.hw_type)
                
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
                ("filter",   c_ubyte),
                ("timing0",  c_ubyte),
                ("timing1",  c_ubyte),
                ("mode",     c_ubyte)]

class _ZCAN_CHANNEL_CANFD_INIT_CONFIG(Structure):
    _fields_ = [("acc_code",     c_uint),
                ("acc_mask",     c_uint),
                ("abit_timing",  c_uint),
                ("dbit_timing",  c_uint),
                ("brp",          c_uint),
                ("filter",       c_ubyte),
                ("mode",         c_ubyte),
                ("pad",          c_ushort),
                ("reserved",     c_uint)]

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
                ("regMode",      c_ubyte),
                ("regStatus",    c_ubyte), 
                ("regALCapture", c_ubyte),
                ("regECCapture", c_ubyte),
                ("regEWLimit",   c_ubyte),
                ("regRECounter", c_ubyte),
                ("regTECounter", c_ubyte),
                ("Reserved",     c_ubyte)]

class ZCAN_CAN_FRAME(Structure):
    _fields_ = [("can_id",  c_uint, 29),
                ("err",     c_uint, 1),
                ("rtr",     c_uint, 1),
                ("eff",     c_uint, 1), 
                ("can_dlc", c_ubyte),
                ("__pad",   c_ubyte),
                ("__res0",  c_ubyte),
                ("__res1",  c_ubyte),
                ("data",    c_ubyte * 8)]

class ZCAN_CANFD_FRAME(Structure):
    _fields_ = [("can_id", c_uint, 29), 
                ("err",    c_uint, 1),
                ("rtr",    c_uint, 1),
                ("eff",    c_uint, 1), 
                ("len",    c_ubyte),
                ("brs",    c_ubyte, 1),
                ("esi",    c_ubyte, 1),
                ("__res",  c_ubyte, 6),
                ("__res0", c_ubyte),
                ("__res1", c_ubyte),
                ("data",   c_ubyte * 64)]

class ZCAN_Transmit_Data(Structure):
    _fields_ = [("frame", ZCAN_CAN_FRAME), ("transmit_type", c_uint)]

class ZCAN_Receive_Data(Structure):
    _fields_  = [("frame", ZCAN_CAN_FRAME), ("timestamp", c_ulonglong)]

class ZCAN_TransmitFD_Data(Structure):
    _fields_ = [("frame", ZCAN_CANFD_FRAME), ("transmit_type", c_uint)]

class ZCAN_ReceiveFD_Data(Structure):
    _fields_ = [("frame", ZCAN_CANFD_FRAME), ("timestamp", c_ulonglong)]

class ZCAN_AUTO_TRANSMIT_OBJ(Structure):
    _fields_ = [("enable",   c_ushort),
                ("index",    c_ushort),
                ("interval", c_uint),
                ("obj",      ZCAN_Transmit_Data)]

class ZCANFD_AUTO_TRANSMIT_OBJ(Structure):
    _fields_ = [("enable",   c_ushort),
                ("index",    c_ushort),
                ("interval", c_uint),
                ("obj",      ZCAN_TransmitFD_Data)]

class IProperty(Structure):
    _fields_ = [("SetValue", c_void_p), 
                ("GetValue", c_void_p),
                ("GetPropertys", c_void_p)]

class ZCAN(object):
    def __init__(self):
        if platform.system() == "Windows":
            self.__dll = windll.LoadLibrary(os.path.join(os.path.dirname(__file__), "zlgcan.dll"))
            #self.__dll = windll.LoadLibrary("D:/python_code/VRProject/python型号合集20240805/python型号合集20240805/python带界面demo/demo/kerneldlls/zlgcan.dll")
        else:
            print("No support now!")
        if self.__dll == None:
            print("DLL couldn't be loaded!")

    def OpenDevice(self, device_type, device_index, reserved):
        try:
            return self.__dll.ZCAN_OpenDevice(device_type, device_index, reserved)
        except:
            print("Exception on OpenDevice!") 
            raise

    def CloseDevice(self, device_handle):
        try:
            return self.__dll.ZCAN_CloseDevice(device_handle)
        except:
            print("Exception on CloseDevice!")
            raise

    def GetDeviceInf(self, device_handle):
        try:
            info = ZCAN_DEVICE_INFO()
            ret = self.__dll.ZCAN_GetDeviceInf(device_handle, byref(info))
            return info if ret == ZCAN_STATUS_OK else None
        except:
            print("Exception on ZCAN_GetDeviceInf")
            raise

    def DeviceOnLine(self, device_handle):
        try:
            return self.__dll.ZCAN_IsDeviceOnLine(device_handle)
        except:
            print("Exception on ZCAN_ZCAN_IsDeviceOnLine!")
            raise

    def InitCAN(self, device_handle, can_index, init_config):
        try:
            return self.__dll.ZCAN_InitCAN(device_handle, can_index, byref(init_config))
        except:
            print("Exception on ZCAN_InitCAN!")
            raise

    def StartCAN(self, chn_handle):
        try:
            return self.__dll.ZCAN_StartCAN(chn_handle)
        except:
            print("Exception on ZCAN_StartCAN!")
            raise

    def ResetCAN(self, chn_handle):
        try:
            return self.__dll.ZCAN_ResetCAN(chn_handle)
        except:
            print("Exception on ZCAN_ResetCAN!")
            raise

    def ClearBuffer(self, chn_handle):
        try:
            return self.__dll.ZCAN_ClearBuffer(chn_handle)
        except:
            print("Exception on ZCAN_ClearBuffer!")
            raise

    def ReadChannelErrInfo(self, chn_handle):
        try:
            ErrInfo = ZCAN_CHANNEL_ERR_INFO()
            ret = self.__dll.ZCAN_ReadChannelErrInfo(chn_handle, byref(ErrInfo))
            return ErrInfo if ret == ZCAN_STATUS_OK else None
        except:
            print("Exception on ZCAN_ReadChannelErrInfo!")
            raise

    def ReadChannelStatus(self, chn_handle):
        try:
            status = ZCAN_CHANNEL_STATUS()
            ret = self.__dll.ZCAN_ReadChannelStatus(chn_handle, byref(status))
            return status if ret == ZCAN_STATUS_OK else None
        except:
            print("Exception on ZCAN_ReadChannelStatus!")
            raise

    def GetReceiveNum(self, chn_handle, can_type = ZCAN_TYPE_CAN):
        try:
            return self.__dll.ZCAN_GetReceiveNum(chn_handle, can_type)
        except:
            print("Exception on ZCAN_GetReceiveNum!")
            raise

    def Transmit(self, chn_handle, std_msg, len):
        try:
            return self.__dll.ZCAN_Transmit(chn_handle, byref(std_msg), len)
        except:
            print("Exception on ZCAN_Transmit!")
            raise

    def Receive(self, chn_handle, rcv_num, wait_time = c_int(-1)):
        try:
            rcv_can_msgs = (ZCAN_Receive_Data * rcv_num)()
            ret = self.__dll.ZCAN_Receive(chn_handle, byref(rcv_can_msgs), rcv_num, wait_time)
            return rcv_can_msgs, ret
        except:
            print("Exception on ZCAN_Receive!")
            raise
    
    def TransmitFD(self, chn_handle, fd_msg, len):
        try:
            return self.__dll.ZCAN_TransmitFD(chn_handle, byref(fd_msg), len)
        except:
            print("Exception on ZCAN_TransmitFD!")
            raise
    
    def ReceiveFD(self, chn_handle, rcv_num, wait_time = c_int(-1)):
        try:
            rcv_canfd_msgs = (ZCAN_ReceiveFD_Data * rcv_num)()
            ret = self.__dll.ZCAN_ReceiveFD(chn_handle, byref(rcv_canfd_msgs), rcv_num, wait_time)
            return rcv_canfd_msgs, ret
        except:
            print("Exception on ZCAN_ReceiveFD!")
            raise

    def GetIProperty(self, device_handle):
        try:
            self.__dll.GetIProperty.restype = POINTER(IProperty)
            return self.__dll.GetIProperty(device_handle)
        except:
            print("Exception on ZCAN_GetIProperty!")
            raise

    def SetValue(self, iproperty, path, value):
        try:
            func = CFUNCTYPE(c_uint, c_char_p, c_char_p)(iproperty.contents.SetValue)
            return func(c_char_p(path.encode("utf-8")), c_char_p(value.encode("utf-8")))
        except:
            print("Exception on IProperty SetValue")
            raise

    def GetValue(self, iproperty, path):
        try:
            func = CFUNCTYPE(c_char_p, c_char_p)(iproperty.contents.GetValue)
            return func(c_char_p(path.encode))
        except:
            print("Exception on IProperty GetValue")
            raise

    def ReleaseIProperty(self, iproperty):
        try:
            return self.__dll.ReleaseIProperty(iproperty)
        except:
            print("Exception on ZCAN_ReleaseIProperty!")
            raise
###############################################################################


'''
USBCANFD-MINI Demo
'''

def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return format(crc, '016b')

def crc8(data: bytes) -> int:
    crc = sum(data)^ 0XFF
    return format(crc, '08b')

def encode_ready_frame():
    global G_READY_ROLL_CNT
    G_READY_ROLL_CNT = (G_READY_ROLL_CNT + 1) % 0xFFFF

    data_frame = bytearray(8)
    data_frame[0] = 0x00
    data_frame[1] = 0x01

    crc_data = crc8(data_frame[:7])

    data_frame[7] = int(crc_data,2)

    return data_frame

def encode_ffb_frame(torque):
    global G_ROLL_CNT

    data_frame = bytearray(64)

    torque = int((torque+20) * 50)

    torque_bin = format(torque, '013b')

    print(format(torque, 'x'))

    # 3.0 -3.1转角控制 0x0invalid 0x1valid 0x3reserved
    # 力矩填充到4.4-6.1位
    # 第0字节保持为子ID
    data_frame[0] = 0x20
    data_frame[1] = 0x1C
    data_frame[2] = 0x01#(0101 0000)
    data_frame[3] = int(torque_bin[-4:], 2) << 4
    data_frame[4] = int(torque_bin[1:-4], 2)

    data_frame[5] = 0x4A #(0100 1001)
    data_frame[6] = 0x40 #(0100 0000)7.4-7.6

    G_ROLL_CNT = (G_ROLL_CNT + 1) % 0xFFFF

    data_frame[60] = (G_ROLL_CNT >> 8) & 0xFF
    data_frame[61] = G_ROLL_CNT & 0xFF

    crc_data = crc16(data_frame[:62])

    data_frame[62] = int(crc_data[10:], 2)
    data_frame[63] = int(crc_data[2:10], 2)

#    format_hex(data_frame)

    return data_frame


def limit_and_encode_angle(angle: float, limit: float) -> int:
    # pitch 3.5 roll4
    limited = min(abs(angle), limit)
    signed_limited = limited if angle >= 0 else -limited
    return int((signed_limited + 1) * 90000)

def throttle_brake_response(vehicle_type, throttle, brake, speed):

    is_u7 = vehicle_type == 'U7'
    if is_u7:
        max_game_data = 0.02
    else:
        max_game_data = 0.08

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


def encode_roll_pitch_frame(vehicle_type, throttle, brake, game_roll, game_pitch, speed, steering_wheel_angle,
                            steering_wheel_angle_old, steering_wheel_rate):
    # if speed < 5:
    #     game_roll = 0
    #     game_pitch = 0

    roll_add = turning_response(vehicle_type, steering_wheel_angle, steering_wheel_angle_old, steering_wheel_rate, speed)
    pitch_add = throttle_brake_response(vehicle_type, throttle, brake, speed)

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

    # roll_total_game = game_roll + roll_add
    # pitch_total_game = game_pitch + pitch_add
    roll_total_game = game_pitch
    pitch_total_game = game_roll
    print("roll_total_game pitch_total_game", roll_total_game, pitch_total_game)
    roll_raw_real = game_roll * 0.5 * 90
    roll_add_real = roll_total_game * 90

    roll_int = limit_and_encode_angle(roll_total_game, roll_limit)
    pitch_int = limit_and_encode_angle(pitch_total_game, pitch_limit)
    print("roll_int pitch_int", roll_int, pitch_int)
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


def encode_switchVR_frame(state):
    switchVR_frame = bytearray(8)
    if state in (0x1, 0x2, 0x3):
        switchVR_frame[0] = 1
        switchVR_frame[1] = state

    return switchVR_frame


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
    steering_wheel_rate = steering_wheel_rate * 6

    if abs(steering_wheel_rate) < 50 or speed < 0.5:
        return 0.0

    speed_factor = 1
    if speed > 200:
        speed_factor = 0.6
    elif speed > 120:
        speed_factor = 0.4
    elif speed > 60:
        speed_factor = 0.2
    elif speed > 0.5:
        speed_factor = 0.1

    delta_angle = steering_wheel_angle_old - steering_wheel_angle
    direction = -1 if delta_angle > 0 else 1

    is_u7 = vehicle_type == 'U7'
    if is_u7:
        max_game_data = 0.02
    else:
        max_game_data = 0.08

    steering_wheel_rate = steering_wheel_rate * 0.0009

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
        print("Set CH%d CANFD clock failed!" %(chn))
    ret = zcanlib.SetValue(ip, str(chn) + "/canfd_standard", "0")
    if ret != ZCAN_STATUS_OK:
        print("Set CH%d CANFD standard failed!" %(chn))
    ret = zcanlib.SetValue(ip, str(chn) + "/initenal_resistance", "1")
    if ret != ZCAN_STATUS_OK:
        print("Open CH%d resistance failed!" %(chn))
    zcanlib.ReleaseIProperty(ip) 

    chn_init_cfg = ZCAN_CHANNEL_INIT_CONFIG()
    chn_init_cfg.can_type = ZCAN_TYPE_CANFD
    chn_init_cfg.config.canfd.abit_timing = 104286#101166 #1Mbps仲裁位速率
    chn_init_cfg.config.canfd.dbit_timing = 4260362#101166 #1Mbps数据位速率
    chn_init_cfg.config.canfd.mode        = 0
    chn_handle = zcanlib.InitCAN(device_handle, chn, chn_init_cfg)
    if chn_handle is None:
        return None
    zcanlib.StartCAN(chn_handle)
    return chn_handle

def send_messages(chn_handle, g112RC_api, zcanlib):
    global G_RC_RUNING_STATUS
    transmit_canfd_num = 7
    canfd_msgs = (ZCAN_TransmitFD_Data * transmit_canfd_num)()

    can_ids = [0x0C3, 0x0C3, 0x0C3, 0x29F, 0x29F, 0x29F,0x341]

    for i, can_id in enumerate(can_ids):
        msg = canfd_msgs[i]
        msg.transmit_type = 1
        msg.frame.eff = 0
        msg.frame.rtr = 0
        msg.frame.brs = 0
        msg.frame.len = 8
        msg.frame.can_id = can_id

    # 获取函数指针
    fInit = RCInit(g112RC_api.RCInit)
    fRead = RCReadFrameData(g112RC_api.RCReadFrameData)
    fEnd = RCEnd(g112RC_api.RCEnd)
    # 重试初始化
    while True:
        try:
            fInit(False)
            print("Initialization successful")
            break
        except Exception as e:
            print("RC is not ready yet")
            time.sleep(0.1)
    # 主循环
    rc_date = FrameData()
    rc_date.roll = 0.0
    rc_date.pitch = 0.0
    cnt = 0
    while True:
        try:
            rc_date_roll_old = rc_date.roll
            rc_date_pitch_old = rc_date.pitch
            fRead(byref(rc_date))
            if rc_date.running == 0:
                G_RC_RUNING_STATUS = 0
                print("No data available")
                time.sleep(1)
                cnt = 0
                continue

            if rc_date.running == 2:
                G_RC_RUNING_STATUS = 2
                fEnd()
                cnt = 0
                print("RC is finished")
                # 重新初始化
                while True:
                    try:
                        fInit(False)
                        print("Re-initialization successful")
                        break
                    except Exception as e:
                        print("RC is not ready yet")
                        time.sleep(0.1)
                continue
            delta_rc_roll = rc_date.roll - rc_date_roll_old
            delta_rc_pitch = rc_date.pitch - rc_date_pitch_old
            G_RC_RUNING_STATUS = 1
            roll_pitch_frame_data,  roll_raw_real, roll_add_real, pitch_real = (
                encode_roll_pitch_frame('U7', G_THROTTLE, G_BRAKE, delta_rc_roll, delta_rc_pitch,
                                        rc_date.speed, G_STEERING_WHEEL_ANGLE, G_STEERING_WHEEL_ANGLE_OLD,
                                        G_STEERING_RATE)
            )
            print(f"speed:{rc_date.speed}\n")
            for i in range(len(roll_pitch_frame_data)):
                for msg in canfd_msgs[:3]:
                    msg.frame.data[i] = roll_pitch_frame_data[i]

            cnt += 1
            if cnt == 5:
                cnt = 0
                switchVR_frame_data = encode_switchVR_frame(0x2)
                for i in range(len(switchVR_frame_data)):
                    for msg in canfd_msgs[3:-1]:
                        msg.frame.data[i] = switchVR_frame_data[i]

            ready_frame = encode_ready_frame()
            for i in range(len(ready_frame)):
                canfd_msgs[6].frame.data[i] = ready_frame[i]

            roll_pitch_steer_ready_ret = zcanlib.TransmitFD(chn_handle, canfd_msgs, transmit_canfd_num)

            print("roll,pitch",roll_pitch_steer_ready_ret)

            # torque = 0.3
            # ffb_frame_data = encode_ffb_frame(torque)

            # steer_canfd_msgs = (ZCAN_TransmitFD_Data)()

            # steer_canfd_msgs.transmit_type = 1
            # steer_canfd_msgs.frame.eff = 0
            # steer_canfd_msgs.frame.rtr = 0
            # steer_canfd_msgs.frame.brs = 0
            # steer_canfd_msgs.frame.len = 64
            # steer_canfd_msgs.frame.can_id = 0x57
            #
            # for i in range(len(ffb_frame_data)):
            #     steer_canfd_msgs.frame.data[i] = ffb_frame_data[i]

            # ret_steer = zcanlib.TransmitFD(chn_handle, steer_canfd_msgs, 1)

            # print("steer", ret_steer)
            # 打印数据

            time.sleep(0.1)

        except Exception as e:
            print(f"Read failed: {e}")
            return
        # finally:
        #     ac_api.AC_Close()


def receive_messages(chn_handle, g112RC_api, zcanlib):
    global G_STEERING_WHEEL_ANGLE_OLD ,G_STEERING_WHEEL_ANGLE, G_STEERING_RATE

    fWriteInput = RCSetInputData(g112RC_api.RCSetInputData)
    rc_inputdata = FrameInputData()
    try:
        while True:
            #rec_start_time = time.time()
            #rcv_num = zcanlib.GetReceiveNum(chn_handle, ZCAN_TYPE_CAN)
            rcv_canfd_num = zcanlib.GetReceiveNum(chn_handle, ZCAN_TYPE_CANFD)
            if rcv_canfd_num:
                rcv_canfd_msgs, rcv_canfd_num = zcanlib.ReceiveFD(chn_handle, rcv_canfd_num, 1000)
                for i in range(rcv_canfd_num):
                    frame = rcv_canfd_msgs[i].frame
                    can_id = frame.can_id
                    data = frame.data
                    if G_RC_RUNING_STATUS == 1 :
                        if can_id == G_STEERING_CAN_ID or can_id == G_STEERING_SBW_CAN_ID:
                            G_STEERING_WHEEL_ANGLE_OLD = G_STEERING_WHEEL_ANGLE
                            steering_angle, G_STEERING_RATE = decode_steering_wheel_angle_frame(can_id,data)
                            steering_wheel_factor = 1.6
                            G_STEERING_WHEEL_ANGLE = steering_angle * steering_wheel_factor
                            rc_inputdata.steer = steering_angle / 360
                            print("rc_inputdata.steer: ", steering_angle, rc_inputdata.steer)
                            fWriteInput(rc_inputdata)
                            # pyvjoy.VJoyDevice(1).set_axis(pyvjoy.HID_USAGE_X, int(G_STEERING_WHEEL_ANGLE / 100 * 32767))

                        if can_id == G_THROTTLE_BRAKE_CAN_ID: 
                            throttle, brake = decode_throttle_brake_frame(data)
                            rc_inputdata.throttle = throttle/ 100
                            # if brake > 25:
                             #    rc_inputdata.handbrake = 1
                            # else:
                             #    rc_inputdata.handbrake = 0
                            rc_inputdata.brake = brake/ 100
                            print("rc_inputdata.throttle: ", rc_inputdata.throttle)
                            print("rc_inputdata.brake: ", rc_inputdata.brake)
                            fWriteInput(rc_inputdata)
                            # pyvjoy.VJoyDevice(1).set_axis(pyvjoy.HID_USAGE_X, int(brake*3/ 100 * 32767))
                            # pyvjoy.VJoyDevice(1).set_axis(pyvjoy.HID_USAGE_X, int(throttle/ 100 * 32767))

            else:
                pass
                #print("wait")
                #break
    except KeyboardInterrupt:
        print("Rec interrupted by user")


if __name__ == "__main__":
    # dll_path = os.path.join(os.getcwd(), "ac_api.dll")
    # ac_api = ctypes.WinDLL(dll_path)
    # ac_api.AC_StartUpdate.restype = ctypes.c_int
    # ac_api.AC_GetSpeedKmh.restype = ctypes.c_float
    # ac_api.AC_GetSteerAngle.restype = ctypes.c_float
    #
    # if ac_api.AC_StartUpdate() != 0:
    #     print("请先打开神力科莎")
    #     exit(1)

    # dll_path = "D:/game_mb/0.0.0/WindowsNoEditor/g112/Saved/RacingRemoteController.dll"
    try:
        # g112RC_api = ctypes.WinDLL(dll_path)
        g112RC_api =windll.LoadLibrary(os.path.join(os.path.dirname(__file__), "RacingRemoteController(1).dll"))
    except Exception as e:
        print(f"Failed to load DLL: {e}")
        exit(0)

    zcanlib = ZCAN() 
    handle = zcanlib.OpenDevice(ZCAN_USBCANFD_MINI, 0,0)
    if handle == INVALID_DEVICE_HANDLE:
        print("Open Device failed!")
        exit(0)
    print("device handle:%d." %(handle))

    info = zcanlib.GetDeviceInf(handle)
    print("Device Information:\n%s" %(info))

    #Start CAN
    chn_handle = can_start(zcanlib, handle, 0)
    print("channel handle:%d." %(chn_handle))

    sender_thread = threading.Thread(target=send_messages, args=(chn_handle, g112RC_api, zcanlib))
    receiver_thread = threading.Thread(target=receive_messages, args=(chn_handle, g112RC_api, zcanlib))

    sender_thread.start()
    receiver_thread.start()

    sender_thread.join()
    receiver_thread.join()

    zcanlib.ResetCAN(chn_handle)
    zcanlib.CloseDevice(handle)
