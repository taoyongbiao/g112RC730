import socket
import struct
import time

# 配置目标地址和端口
DEST_IP = "127.0.0.1"  # 根据你的环境修改
DEST_PORT = 8888             # 中间件接收端口

# 包类型常量
TYPE_CTRL = 0x02  # 控制包类型

def send_test_ctrl_packet(sock, throttle=50, brake=30, steering_angle=720, steering_rotation=100):
    """
    构造并发送一个 TYPE_CTRL 类型的测试数据包。
    数据包含：
        - throttle_depth: 油门深度（0~100）
        - brake_depth: 制动深度（0~100）
        - steering_angle: 方向盘角度（无符号 16 位整数）
        - steering_rotation: 转向速率（0~255）
    """
    packet_type = TYPE_CTRL
    device_id = 1

    # 打包 payload 数据
    payload = struct.pack('>BBHBB',
                          throttle,
                          brake,
                          steering_angle,
                          steering_rotation,
                          0x00)  # 补齐字段（可选）

    print(f"[DEBUG] 发送控制包内容: throttle={throttle}, brake={brake}, angle={steering_angle}, rotation={steering_rotation}")

    # 构造完整数据包
    id_bytes = struct.pack('>I', device_id)
    payload_len_bytes = struct.pack('>H', len(payload))
    timestamp_bytes = struct.pack('>Q', int(time.time() * 1000))  # 时间戳 ms
    packet = struct.pack('>B', packet_type) + id_bytes + payload_len_bytes + timestamp_bytes + payload

    # 发送数据包
    sock.sendto(packet, (DEST_IP, DEST_PORT))

if __name__ == "__main__":
    # 创建 UDP 套接字
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:
        # 循环发送测试包
        while True:
            # 示例值：模拟方向盘左转、油门刹车变化
            throttle = 50 + int(25 * (1 + math.sin(time.time())))  # 25~75%
            brake = 30 + int(20 * (1 + math.cos(time.time())))    # 10~50%
            steering_angle = 720 + int(360 * math.sin(time.time()))  # ±360°
            steering_rotation = 100 + int(50 * math.sin(time.time()))

            send_test_ctrl_packet(sock, throttle, brake, steering_angle, steering_rotation)
            time.sleep(0.1)  # 每秒发送约 10 帧
    except KeyboardInterrupt:
        print("[INFO] 测试结束")
    finally:
        sock.close()