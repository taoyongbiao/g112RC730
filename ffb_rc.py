# ffb_rc.py -主要实现巅峰极速dll加载、数据处理、数据发送

from ctypes import *
import os
import ctypes
import time
# from ffb_cal_0630 import ForceFeedbackAlgorithm
from ffb_cal_0624 import ForceFeedbackAlgorithm
from loguru import logger
import math


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
    # 使用 memset 清零数组部分（可选方法）
    memset(rc_date.suspensionData, 0, sizeof(FrameSuspensionData) * 4)
    memset(rc_date.tyreData, 0, sizeof(FrameTyreData) * 4)
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
    # rc_date.speed=120

    return (
        rc_date.roll,
        rc_date.pitch,
        rc_date.speed,
        rc_date.suspensionData,
        rc_date.tyreData,
    )
def calculate_suspension_force_moment(suspension_data, wheel_base=2.5, track_width=1.5):
    """
    根据悬架数据计算悬架效应对方向盘的力矩
    
    参数:
    suspension_data: FrameSuspensionData * 4 数组，包含四个车轮的悬架数据
    wheel_base: 轴距(米)
    track_width: 轮距(米)
    
    返回:
    force_moment: 方向盘力矩 (N·m)，正值表示需要向右修正，负值表示需要向左修正
    """
    
    # 获取四个车轮的悬架数据 (通常顺序为: 左前, 右前, 左后, 右后)
    left_front = suspension_data[0]
    right_front = suspension_data[1]
    left_rear = suspension_data[2]
    right_rear = suspension_data[3]
    
    # 计算前后轴的悬架高度差，反映车辆俯仰(pitch)
    front_roll_stiffness = 20000  # 前悬架刚度 (N/m)，可根据实际车辆调整
    rear_roll_stiffness = 18000   # 后悬架刚度 (N/m)，可根据实际车辆调整
    
    # 计算左右两侧悬架高度差，反映车辆侧倾(roll)
    front_suspension_diff = right_front.height - left_front.height
    rear_suspension_diff = right_rear.height - left_rear.height
    
    # 计算悬架速度差，反映车辆动态变化
    front_velocity_diff = right_front.velocity - left_front.velocity
    rear_velocity_diff = right_rear.velocity - left_rear.velocity
    
    # 计算侧倾引起的力矩 (主要影响方向盘)
    roll_moment = front_suspension_diff * front_roll_stiffness * 0.3  # 0.3为传递系数
    
    # 考虑悬架速度对力矩的影响 (增加动态反馈感)
    roll_damping_moment = front_velocity_diff * 1000  # 阻尼系数，提供动态反馈
    
    # 计算俯仰力矩 (通过轮胎载荷转移间接影响转向)
    pitch_effect = (rear_suspension_diff - front_suspension_diff) * 500  # 俯仰刚度影响
    
    # 总力矩计算
    total_moment = roll_moment + roll_damping_moment + pitch_effect
    
    return total_moment

def interpret_force_sign(meaning_only=False):
    """
    解释力矩正负的物理含义
    """
    if meaning_only:
        return
    
    meanings = {
        "正力矩": "车辆向右倾斜或右轮遇到颠簸，方向盘感受到向右的修正力",
        "负力矩": "车辆向左倾斜或左轮遇到颠簸，方向盘感受到向左的修正力",
        "力矩为0": "车辆处于水平状态，悬架两侧受力平衡",
        "力矩绝对值大": "车辆侧倾或颠簸剧烈，需要更大的手力来保持方向",
        "力矩变化快": "车辆正在快速通过不平路面，方向盘感受到快速变化的反馈"
    }
    
    return meanings
  


def calculate_lateral_force(tyre_data, friction_coefficient=1.0):
    """
    基于轮胎数据计算侧滑力（侧向力）
    
    参数:
    tyre_data: FrameTyreData 结构体实例
    friction_coefficient: 路面摩擦系数，默认为1.0
    
    返回:
    lateral_force: 侧向力 (N)
    """
    # 侧偏角转换为弧度
    slip_angle_rad = math.radians(tyre_data.slipAngle)
    
    # 考虑轮胎载荷对侧偏刚度的影响
    # 简化的侧偏刚度模型，假设基础侧偏刚度为10000 N/deg
    cornering_stiffness = 10000 * (tyre_data.load / 1000.0)  # 载荷越大，侧偏刚度越高
    
    # 侧向力计算 - 使用更精确的轮胎模型
    # 侧向力 = 侧偏刚度 × 侧偏角(弧度) × 路面摩擦系数
    lateral_force = cornering_stiffness * slip_angle_rad * friction_coefficient
    
    # 考虑轮胎滑移率对侧向力的影响（滑移率越大，侧向力越小）
    if tyre_data.slipRatio > 0:
        # 滑移率影响因子，防止侧向力过大
        slip_ratio_factor = max(0.1, 1.0 - tyre_data.slipRatio * 0.5)
        lateral_force *= slip_ratio_factor
    
    return lateral_force

def calculate_combined_tire_force(tyre_data, friction_coefficient=1.0):
    """
    综合考虑纵向力和侧向力的轮胎力计算
    
    参数:
    tyre_data: FrameTyreData 结构体实例
    friction_coefficient: 路面摩擦系数，默认为1.0
    
    返回:
    longitudinal_force: 纵向力 (N)
    lateral_force: 侧向力 (N)
    total_force: 合力 (N)
    """
    # 侧偏角转换为弧度
    slip_angle_rad = math.radians(tyre_data.slipAngle)
    
    # 简化的纵向力计算（基于滑移率）
    # 这里使用简化的魔术公式形式
    peak_slip_ratio = 0.1  # 峰值滑移率
    if tyre_data.slipRatio <= peak_slip_ratio:
        # 在峰值之前，纵向力与滑移率成正比
        longitudinal_force = tyre_data.load * friction_coefficient * (tyre_data.slipRatio / peak_slip_ratio)
    else:
        # 超过峰值后，纵向力逐渐减小
        longitudinal_force = tyre_data.load * friction_coefficient * max(0.2, 1.0 - (tyre_data.slipRatio - peak_slip_ratio) * 2)
    
    # 侧向力计算
    cornering_stiffness = 10000 * (tyre_data.load / 1000.0)
    lateral_force = cornering_stiffness * slip_angle_rad * friction_coefficient
    
    # 考虑滑移率对侧向力的影响
    if tyre_data.slipRatio > 0:
        slip_ratio_factor = max(0.1, 1.0 - tyre_data.slipRatio * 0.5)
        lateral_force *= slip_ratio_factor
    
    # 计算合力（纵向力和侧向力的矢量和）
    total_force = math.sqrt(longitudinal_force**2 + lateral_force**2)
    
    # 确保合力不超过最大摩擦力
    max_friction_force = tyre_data.load * friction_coefficient
    if total_force > max_friction_force:
        # 按比例缩放以符合摩擦力限制
        scale_factor = max_friction_force / total_force
        longitudinal_force *= scale_factor
        lateral_force *= scale_factor
        total_force = max_friction_force
    
    return longitudinal_force, lateral_force, total_force

