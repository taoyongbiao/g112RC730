import threading
# shared_state.py


"""
共享数据模块，用于解决循环引用问题
"""
import threading
from collections import defaultdict

# 共享的扭矩数据
torque_data = defaultdict(list)

# 最大数据点数
MAX_DATA_POINTS = 5000

# 全局变量
G_ROLL_CNT = 0
G_READY_ROLL_CNT = 0
G_RATE_DIR = 1
G_STEERING_RATE = 0.0
G_STEERING_WHEEL_ANGLE = 0.0
G_STEERING_WHEEL_ANGLE_OLD = 0.0
G_THROTTLE = 0.0
G_BRAKE = 0.0
G_HAND_FORCE = 0.0

# CAN ID
G_STEERING_CAN_ID = 0x8E
G_STEERING_SBW_CAN_ID = 0x8E
G_THROTTLE_BRAKE_CAN_ID = 0x342

# 运行标志事件
run_main_flag_event = threading.Event()
