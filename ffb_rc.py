# ffb_rc.py -主要实现巅峰极速dll加载、数据处理、数据发送

from ctypes import *
import os
import ctypes
import time
# from ffb_cal_0630 import ForceFeedbackAlgorithm
from ffb_cal_0624 import ForceFeedbackAlgorithm
from loguru import logger


# ============================
# 必要的结构体定义
# ============================

class FrameSuspensionData(Structure):
    _pack_ = 4
    _fields_ = [
        ("height", c_float),
        ("velocity", c_float),
    ]

class FrameTyreData(Structure):
    _pack_ = 4
    _fields_ = [
        ("slipRatio", c_float),
        ("slipAngle", c_float),
        ("load", c_float),
        ("rotationRate", c_float),
    ]

class FrameData(Structure):
    _pack_ = 4
    _fields_ = [
        ("clock", c_float),
        ("throttle", c_float),
        ("brake", c_float),
        ("gear", c_int),
        ("rpm", c_float),
        ("steering", c_float),
        ("speed", c_float),
        ("velocity", c_float * 3),
        ("acceleration", c_float * 3),
        ("yaw", c_float),
        ("pitch", c_float),
        ("roll", c_float),
        ("comy", c_float),
        ("tyreData", FrameTyreData * 4),
        ("suspensionData", FrameSuspensionData * 4),
        ("angular", c_float * 3),
        ("extraMessage", c_uint),
        ("running", c_uint),
    ]

class FrameInputData(Structure):
    _pack_ = 4
    _fields_ = [
        ("steer", c_float),
        ("throttle", c_float),
        ("brake", c_float),
        ("handbrake", c_float),
        ("clutch", c_float),
        ("gear", c_int),
    ]


# ============================
# 必要的函数指针定义
# ============================

RCInit = WINFUNCTYPE(None, c_bool)
RCReadFrameData = WINFUNCTYPE(None, POINTER(FrameData))
RCEnd = WINFUNCTYPE(None)
RCSetInputData = WINFUNCTYPE(None, POINTER(FrameInputData))


# ============================
# 必要的全局变量
# ============================

G_STEERING_WHEEL_ANGLE = 0.0
G_STEERING_RATE = 0.0
G_THROTTLE = 0.0
G_BRAKE = 0.0

#函数指针供外部访问
fRead = None
fEnd = None
fWriteInput= None
g112RC_api=None



def init_game_api():
    """
    使用 RacingRemoteController.dll 进行通信
    """
    global fRead, fEnd,fWriteInput,g112RC_api
    # dll_path = "RacingRemoteController(1).dll"
    try:
        # g112RC_api = WinDLL(dll_path)
        try:
            g112RC_api =windll.LoadLibrary(os.path.join(os.path.dirname(__file__), "RacingRemoteController(1).dll"))
        except Exception as e:
            #print(f"g112RC_api:{e}\n")
            logger.error(f"g112RC_api:{e}\n")

        fRead = RCReadFrameData(g112RC_api.RCReadFrameData)
        fWriteInput = RCSetInputData(g112RC_api.RCSetInputData)
        fEnd = RCEnd(g112RC_api.RCEnd)
        fInit = RCInit(g112RC_api.RCInit)
        fInit(False)
    except Exception as e:
        #print(f"Failed to load RC DLL: {e}")
        logger.error(f"Failed to load RC DLL: {e}")
        exit(0)


def get_fRead():
    """
    获取 fRead 函数引用。
    """
    if fRead is None:
        raise RuntimeError("fRead not initialized. Call init_game_api() first.")
    return fRead


def get_fEnd():
    """
    获取 fEnd 函数引用。
    """
    if fEnd is None:
        raise RuntimeError("fEnd not initialized. Call init_game_api() first.")
    return fEnd

def get_fWriteInput():
    """
    获取 fEnd 函数引用。
    """
    if fWriteInput is None:
        raise RuntimeError("fEnd not initialized. Call init_game_api() first.")
    return fWriteInput


def read_vehicle_status_from_rc():
    """
    使用 RacingRemoteController.dll 接口读取车辆状态数据
    """


    rc_date = FrameData()
    rc_date.roll = 0.0
    rc_date.pitch = 0.0

    try:
        fRead(byref(rc_date))
    except Exception as e:
        #print(f"[ERROR] Failed to read frame data from RC: {e}")
        logger.error(f"[ERROR] Failed to read frame data from RC: {e}")
        return None, None, 0.0, None, None, None, None

    if rc_date.running == 0:
        #print("No data available from RC")
        logger.error("No data available from RC")
        return None, None, 0.0, None, None, None, None
    #print(f"============speed:{rc_date.speed}\n")
    logger.debug(f"============speed:{rc_date.speed}\n")
    rc_date.speed=120

    return (
        rc_date.roll,
        rc_date.pitch,
        rc_date.speed,

    )

