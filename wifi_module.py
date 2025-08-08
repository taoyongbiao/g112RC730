

import struct
import socket
import time
import threading
import ctypes
import psutil
import msvcrt
import pyvjoy
from loguru import logger
from DCH_VR_0630bk import (
    limit_and_encode_angle,
    read_vehicle_status,
    turning_response,
    throttle_brake_response,
    record_torque_data,
    G_THROTTLE,
    G_BRAKE,    
    G_STEERING_WHEEL_ANGLE,
    G_STEERING_WHEEL_ANGLE_OLD,
    G_STEERING_RATE,
    update_g_vars,
    RC,
    fWriteInput,
    fRead
    )
# from ffb_cal_0630 import ForceFeedbackAlgorithm
from ffb_cal_0624 import ForceFeedbackAlgorithm
from ffb_rc import FrameData,FrameInputData,read_vehicle_status_from_rc

# sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

# G_DEST_IP = '192.168.46.234'
# DEST_PORT = 8888
# CONNECT_EVENT = threading.Event()


# socket value
# 包类型常量
TYPE_BROADCAST = 0x00  # 广播包
TYPE_STATUS = 0x01  # 状态包
TYPE_CTRL = 0x02    # 控制包
TYPE_HEARTBEAT = 0x03  # 心跳包
TYPE_BACK = 0x04  # 应答包
TYPE_SHOCK = 0x05 # 方向盘震动包

# UDP 配置
UDP_IP = '0.0.0.0'  # 监听所有接口
UDP_PORT_SEND = 5000  # 发送端口
UDP_PORT_RECEIVE = 8001  # 接收端口
# BROADCAST_IP = '192.168.92.255'  # 广播地址
BROADCAST_IP = '255.255.255.255'  # 广播地址

# add by kdy
G_LAST_TIME = 0.0                     # 最新心跳时间
G_LAST_TIME_LOCK = threading.Lock() # 心跳时间锁
# BROADCAST_EVENT = threading.Event() # 广播包发送事件
CONNECT_EVENT = threading.Event()   # 是否建立连接标志

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

G_DEST_IP = '192.168.77.185'    #中间件IP地址
G_HEARTBEAT_TIMEOUT = 5
DEST_PORT = 8888                # 中间件接收端口

# 全局线程停止标志
wifi_flag_event = threading.Event()
# ========== 包构造与解析 ==========
def wifi_build_packet(pkt_type, id, payload=None):
    id_bytes = struct.pack('>I', id)
    payload_bytes = payload if payload else b''
    payload_len = len(payload_bytes)
    payload_len_bytes = struct.pack('>H', payload_len)
    timestamp = int(time.time() * 1000)
    timestamp_bytes = struct.pack('>Q', timestamp)
    return struct.pack('>B', pkt_type) + id_bytes + payload_len_bytes + timestamp_bytes + payload_bytes

def wifi_parse_packet(data):
    if len(data) < 15:
        return None
    try:
        pkt_type = data[0]
        id = struct.unpack('>I', data[1:5])[0]
        payload_len = struct.unpack('>H', data[5:7])[0]
        timestamp = struct.unpack('>Q', data[7:15])[0]
        payload = data[15:15 + payload_len] if payload_len > 0 else b''
        return {
            'type': pkt_type,
            'id': id,
            'payload_len': payload_len,
            'timestamp': timestamp,
            'payload': payload
        }
    except struct.error:
        return None

# ========== 网络相关 ==========
def wifi_get_wifi_ip_address():
    interfaces = psutil.net_if_addrs()
    for interface_name, interface_addresses in interfaces.items():
        if "wireless" in interface_name.lower() or "wlan" in interface_name.lower():
            for address in interface_addresses:
                if address.family == socket.AF_INET:
                    return address.address
    return None

def wifi_get_async_key_state(vk_code):
    return bool(msvcrt.kbhit() and ord(msvcrt.getch()) == vk_code)

def wifi_send_broadcast_messages():
    global DEST_PORT, sock
    ip_str = wifi_get_wifi_ip_address()
    if ip_str:
        #print("Wi-Fi IP Address", ip_str)
        logger.info("Wi-Fi IP Address", ip_str)
    else:
        #print("无法找到Wi-Fi接口或未连接到Wi-Fi")
        logger.info("无法找到Wi-Fi接口或未连接到Wi-Fi")
        return
    packed_ip = socket.inet_aton(ip_str)
    ip_bytes = int.from_bytes(packed_ip, 'big')
    #print(f"Wi-Fi IP Address:{ip_bytes:#010x}")
    logger.debug(f"Wi-Fi IP Address:{ip_bytes:#010x}")
    while wifi_flag_event.is_set():
      
        try:
            if not CONNECT_EVENT.is_set():
                broadcast_payload = struct.pack('>IHHB', ip_bytes, UDP_PORT_SEND, UDP_PORT_RECEIVE, 0x1)
                broadcast_pkt = wifi_build_packet(TYPE_BROADCAST, 1, broadcast_payload)
                sock.sendto(broadcast_pkt, (BROADCAST_IP, DEST_PORT))
                #print(f"[BROADCAST] 广播包已发送: {broadcast_pkt.hex()}")
                logger.info(f"[BROADCAST] 广播包已发送: {broadcast_pkt.hex()}")
        except Exception as e:
            #print(f"Read failed: {e}")
            logger.info(f"Read failed: {e}")
            return
        time.sleep(1)

def wifi_send_messages(ac_api):
    global G_RC_RUNING_STATUS
    id = 1
    while wifi_flag_event.is_set():

        try:
            CONNECT_EVENT.wait()
            time.sleep(0.1)
            # 从游戏获取数据 需区分巅峰极速还是神力科莎
            if RC:
                roll, pitch, speed= read_vehicle_status_from_rc()
            else:
                roll, pitch, speed, wheel_slip, acc_g, suspension_travel, local_angular_vel = read_vehicle_status(ac_api)  


            steering_wheel_angle = G_STEERING_WHEEL_ANGLE
            steering_wheel_rate = G_STEERING_RATE
            steering_wheel_angle_old = G_STEERING_WHEEL_ANGLE_OLD
            # 计算附加项
            roll_add = turning_response('U9',steering_wheel_angle, steering_wheel_angle_old, steering_wheel_rate, speed)
            pitch_add = throttle_brake_response('U9',G_THROTTLE, G_BRAKE, speed)

            # 总量 = 原始 + 附加
            roll_total_game = roll_add + roll
            pitch_total_game = pitch + pitch_add

            # 限制并编码
            is_u7 = False  # 假设不是 U7 车型
            roll_limit = 0.044 if is_u7 else 0.12
            pitch_limit = 0.039 if is_u7 else 0.12

            roll_int = limit_and_encode_angle(pitch_total_game, roll_limit)
            pitch_int = limit_and_encode_angle(roll_total_game, pitch_limit)


            # 获取并计算力矩值
            ffb = ForceFeedbackAlgorithm()
            desired_torque = ffb.get_tanh_torque(speed, G_STEERING_WHEEL_ANGLE)
            friction, damping = ffb.get_friction(ctypes.c_float(G_STEERING_WHEEL_ANGLE), ctypes.c_float(G_STEERING_RATE))
            lateral_effect = ffb.get_lateral_effect(
                speed,
                acc_g,  # 提取数组部分
                local_angular_vel,
                wheel_slip.slip
            )
            suspension_effect=ffb.get_suspension_effect(speed, suspension_travel)
            total_torque = desired_torque - damping - friction
            scale_torque = total_torque * 0.001 * 0.05 * -1  # 调整比例

            # 记录数据
            start_time = time.time()  # 记录开始时间
            record_torque_data(start_time, desired_torque, damping, friction, total_torque, scale_torque,lateral_effect,suspension_effect)

            
            torque_data=int((scale_torque + 20) * 50)




            # 构造数据帧并通过sock发送
            shock_flag_data = 0x02
            status_payload = struct.pack('>IIH', roll_int, pitch_int, torque_data)


            status_pkt = wifi_build_packet(TYPE_STATUS, id, status_payload)
            shock_payload = struct.pack('>B', shock_flag_data)
            shock_pkt = wifi_build_packet(TYPE_SHOCK, id, shock_payload)
            id += 1
            sock.sendto(status_pkt, (G_DEST_IP, DEST_PORT)) 

            #print("send pitch、roll and torque!")
            logger.info("send pitch、roll and torque!")
            sock.sendto(shock_pkt, (G_DEST_IP, DEST_PORT))
            #print("send shock!")
            logger.info("send shock!")
            time.sleep(0.1)
        except Exception as e:
            #print(f"Read failed: {e}")
            logger.info(f"Read failed: {e}")
            # return
            continue

def wifi_send_heartbeat_messages():
    global DEST_PORT, sock
    id = 1
    try:
        while wifi_flag_event.is_set():

            CONNECT_EVENT.wait()
            status_pkt = wifi_build_packet(TYPE_HEARTBEAT, id)
            sock.sendto(status_pkt, (G_DEST_IP, DEST_PORT))
            #print("Heartbeat message sent")
            logger.info("Heartbeat message sent")
            id += 1
            time.sleep(1)
    except KeyboardInterrupt:
        #print("Send interrupted by user")
        logger.info("Send interrupted by user")

def wifi_heartbeat_monitor():
    global G_LAST_TIME, G_HEARTBEAT_TIMEOUT
    while wifi_flag_event.is_set():


        CONNECT_EVENT.wait()
        time.sleep(1.5)
        with G_LAST_TIME_LOCK:
            diff_time = time.time() - G_LAST_TIME
        if diff_time > G_HEARTBEAT_TIMEOUT:
            #print(f"Heartbeat timeout! Try reconnecting... diff_time:{diff_time}")
            logger.info(f"Heartbeat timeout! Try reconnecting... diff_time:{diff_time}")
            CONNECT_EVENT.clear()

def wifi_parse_control_data(payload_data):
    if len(payload_data) < 5:
        return None
    try:
        throttle, brake, steering_angle, steering_rate = struct.unpack('>BBhB', payload_data[:5])
        throttle_normalized = throttle / 100.0
        brake_normalized = brake / 100.0
        if steering_angle > 0x7FFF:
            steering_angle = (~steering_angle & 0xFFFF) + 1
            steering_angle_normalized = steering_angle / 7200.0
        else:
            steering_angle_normalized = -steering_angle / 7200.0
        steering_rate_normalized = steering_rate
        return throttle_normalized, brake_normalized, steering_angle_normalized, steering_rate_normalized
    except struct.error:
        return None


def process_steering_data(steering_angle_raw, steering_rate,brake,throttle):
    """
    处理方向盘角度数据并更新全局状态。
    
    参数:
        steering_angle_raw (int): 原始角度值（无符号 16 位整数）
        steering_rate (int): 转向速率
    
    返回:
        float: 处理后的角度值
        int: 转向速率
        int: 方向（1 表示左转，-1 表示右转）
    """
    # 角度转换为有符号浮点数
    if steering_angle_raw > 0x7FFF:
        steering_angle = -((~steering_angle_raw & 0xFFFF) + 1) / 10.0
    else:
        steering_angle = steering_angle_raw / 10.0

    # 方向判断
    delta = G_STEERING_WHEEL_ANGLE - steering_angle
    direction = 1 if delta > 0 else -1

    # 更新全局变量
    update_g_vars(steering_angle, steering_rate, direction)

    # 更新 pyvjoy（可选） 传输给游戏 需要区分巅峰急速还是神力科莎
    try:
        pyvjoy.VJoyDevice(1).set_axis(pyvjoy.HID_USAGE_Z,
                                      int((steering_angle + 720) / 1440 * 32767))
        pyvjoy.VJoyDevice(1).set_axis(pyvjoy.HID_USAGE_Y, int(brake * 3 / 100 * 32767))
        pyvjoy.VJoyDevice(1).set_axis(pyvjoy.HID_USAGE_X, int(throttle / 100 * 32767))
    except Exception as e:
        #print(f"[ERROR] 设置 vJoy 设备失败: {e}")
        logger.info(f"[ERROR] 设置 vJoy 设备失败: {e}")

    return steering_angle, steering_rate, direction
def wifi_receive_messages():
    global G_STEERING_WHEEL_ANGLE_OLD, G_STEERING_WHEEL_ANGLE, G_STEERING_RATE, G_CLIENT_IP, G_LAST_TIME, sock
    sock.bind((UDP_IP, UDP_PORT_RECEIVE))
    #print(f"[+] 正在监听端口 {UDP_PORT_RECEIVE}")
    logger.info(f"[+] 正在监听端口 {UDP_PORT_RECEIVE}")
    try:
        while wifi_flag_event.is_set():

            data, addr = sock.recvfrom(1024)
            pkt = wifi_parse_packet(data)
            if pkt is None:
                #print("[ERROR] 数据包解析失败")
                logger.info("[ERROR] 数据包解析失败")
                continue
            packet_type = pkt['type']
            device_id = pkt['id']
            payload_len = pkt['payload_len']
            timestamp = pkt['timestamp']
            payload_data = pkt['payload']
            #print(f"接收类型: {packet_type}")
            logger.info(f"接收类型: {packet_type}")

            if packet_type == TYPE_CTRL:
                if len(payload_data) >= 5:
                    try:
                        throttle_depth = struct.unpack('>B', payload_data[0:1])[0]
                        brake_depth = struct.unpack('>B', payload_data[1:2])[0]
                        #test
                        brake_depth=0
                        steering_angle = struct.unpack('>H', payload_data[2:4])[0]
                        steering_rotation = struct.unpack('>B', payload_data[4:5])[0]
                        #print(f"Throttle Depth: {throttle_depth}")
                        logger.debug(f"Throttle Depth: {throttle_depth}")
                        #print(f"Brake Depth: {brake_depth}")
                        logger.debug(f"Brake Depth: {brake_depth}")
                        #print(f"Steering Angle: {steering_angle}")
                        logger.debug(f"Steering Angle: {steering_angle}")
                        #print(f"Steering Rotation: {steering_rotation}")
                        logger.debug(f"Steering Rotation: {steering_rotation}")
                        # 调用封装好的函数处理方向盘数据  传给游戏
                        if RC:
                            rc_inputdata = FrameInputData()
                            rc_inputdata.steer = steering_angle / 360
                            #print("rc_inputdata.steer:", rc_inputdata.steer)
                            logger.debug("rc_inputdata.steer:", rc_inputdata.steer)    
                            rc_inputdata.throttle = throttle_depth/ 100
                            rc_inputdata.brake = brake_depth/ 100

                            #print("rc_inputdata.throttle: ", rc_inputdata.throttle)
                            logger.debug("rc_inputdata.throttle: ", rc_inputdata.throttle)
                            #print("rc_inputdata.brake: ", rc_inputdata.brake)
                            logger.debug("rc_inputdata.brake: ", rc_inputdata.brake)
                            fWriteInput(rc_inputdata)
                        else:
                            process_steering_data(steering_angle, steering_rotation,brake_depth,throttle_depth)


                    except struct.error as e:
                        #print(f"解析控制包失败: {e}")
                        logger.info(f"解析控制包失败: {e}")
                else:
                    #print("控制包数据不足")
                    logger.info("控制包数据不足")

            if packet_type == TYPE_HEARTBEAT:
                with G_LAST_TIME_LOCK:
                    G_LAST_TIME = time.time()
                #print(f"[ACTION] 收到心跳包，更新最后心跳时间: {G_LAST_TIME}")
                logger.info(f"[ACTION] 收到心跳包，更新最后心跳时间: {G_LAST_TIME}")

            if packet_type == TYPE_BACK:
                with G_LAST_TIME_LOCK:
                    G_LAST_TIME = time.time()
                    #print(f"  [ACTION] last_time: {G_LAST_TIME}")
                    logger.debug(f"  [ACTION] last_time: {G_LAST_TIME}")
                    # 解包：1字节 flag + 4字节 IP
                    funFlag, ip = struct.unpack('>BI', payload_data[:5])
                    ip_str = socket.inet_ntoa(struct.pack('!I', ip))
                    #print(f"  [ACTION] 解包：1字节 flag + 4字节 IP: {funFlag, ip_str}")
                    logger.debug(f"  [ACTION] 解包：1字节 flag + 4字节 IP: {funFlag, ip_str}")
                    global G_DEST_IP
                    G_DEST_IP = ip_str  #  更新全局目标IP地址 G_D
                # print(f"[ACTION] last_time: {G_LAST_TIME}")
                    
                CONNECT_EVENT.set()
    except KeyboardInterrupt:
        #print("Rec interrupted by user")
        logger.info("Rec interrupted by user")