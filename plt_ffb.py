
import numpy as np
import matplotlib.pyplot as plt
from ffb_cal_ori import ForceFeedbackAlgorithm
import ctypes

# 移除全局变量 ffb 的定义

def plot_torque_vs_speed(ax, ffb):
    angles = [10, 60, 180, 450]
    speeds = np.linspace(0, 200, 300)

    for angle in angles:
        torques = [ffb.get_tanh_torque(speed, angle) for speed in speeds]
        ax.plot(speeds, torques, label=f"Angle = {angle}°")

    ax.set_title("Torque vs Speed (Fixed Angle)")
    ax.set_xlabel("Speed (km/h)")
    ax.set_ylabel("Torque (mNm)")
    ax.legend()
    ax.grid(True)

def plot_torque_vs_angle(ax, ffb):
    speeds = [20, 60, 120, 180]
    angles = np.linspace(-450, 450, 300)

    for speed in speeds:
        torques = [ffb.get_tanh_torque(speed, angle) for angle in angles]
        ax.plot(angles, torques, label=f"Speed = {speed} km/h")

    ax.set_title("Torque vs Steering Angle (Fixed Speed)")
    ax.set_xlabel("Steering Angle (°)")
    ax.set_ylabel("Torque (mNm)")
    ax.legend()
    ax.grid(True)

def plot_lateral_effect_vs_speed(ax, ffb):
    steer_angles = [ctypes.c_float(45), ctypes.c_float(90), ctypes.c_float(180)]
    wheel_slip = [0.0] * 4
    
    # 创建模拟的 acc_g 和 angular_vel 对象，匹配方法期望的格式
    class MockAccG:
        def __init__(self, values):
            self.accg = values
            
    class MockAngularVel:
        def __init__(self, values):
            self.VehAngVel = values

    acc_g = MockAccG([0.0, 0.0, 0.0])
    local_angular_vel = MockAngularVel([0.0, 0.0, 0.0])

    speeds = np.linspace(0, 200, 300)
    for sa in steer_angles:
        lateral_effects = [
            ffb.get_lateral_effect(
                speed,
                acc_g,
                local_angular_vel,
                (wheel_slip[0], wheel_slip[1], wheel_slip[2], wheel_slip[3])
            ) for speed in speeds
        ]
        ax.plot(speeds, lateral_effects, label=f"Steer Angle = {sa.value}°")

    ax.set_title("Lateral Effect vs Speed")
    ax.set_xlabel("Speed (km/h)")
    ax.set_ylabel("Lateral Effect")
    ax.legend()
    ax.grid(True)

def plot_friction_vs_steer_rate(ax, ffb):
    steer_angles = [ctypes.c_float(0), ctypes.c_float(90), ctypes.c_float(180)]
    steer_rates = np.linspace(0, 1080, 300)

    for sa in steer_angles:
        frictions = [
            ffb.get_friction(sa, ctypes.c_float(sr))[0] for sr in steer_rates
        ]
        ax.plot(steer_rates, frictions, label=f"Steer Angle = {sa.value}°")

    ax.set_title("Friction vs Steering Rate")
    ax.set_xlabel("Steering Rate (deg/s)")
    ax.set_ylabel("Friction")
    ax.legend()
    ax.grid(True)

def plot_total_torque_vs_speed(ax, ffb):
    speeds = np.linspace(0, 200, 300)
    angles = [ctypes.c_float(45), ctypes.c_float(90), ctypes.c_float(180)]

    for angle in angles:
        torques = [
            ffb.get_tanh_torque(speed, angle.value) -
            ffb.get_friction(angle, ctypes.c_float(0))[1] -
            ffb.get_friction(angle, ctypes.c_float(0))[0]
            for speed in speeds
        ]
        ax.plot(speeds, torques, label=f"Angle = {angle.value}°")

    ax.set_title("Total Torque vs Speed")
    ax.set_xlabel("Speed (km/h)")
    ax.set_ylabel("Total Torque (mNm)")
    ax.legend()
    ax.grid(True)

def plot_total_torque_vs_angle(ax, ffb):
    angles = np.linspace(-450, 450, 300)
    speeds = [20, 60, 120, 180]

    for speed in speeds:
        torques = [
            ffb.get_tanh_torque(speed, angle) -
            ffb.get_friction(angle, 0)[1] -
            ffb.get_friction(angle, 0)[0]
            for angle in angles
        ]
        ax.plot(angles, torques, label=f"Speed = {speed} km/h")

    ax.set_title("Total Torque vs Steering Angle")
    ax.set_xlabel("Steering Angle (°)")
    ax.set_ylabel("Total Torque (mNm)")
    ax.legend()
    ax.grid(True)

if __name__ == "__main__":
    # 初始化算法类
    ffb = ForceFeedbackAlgorithm()
    
    # 创建 2 行 3 列 的画布，总共有 6 个子图
    fig, axes = plt.subplots(2, 3, figsize=(18, 10))

    # 绘制原有两个图
    plot_torque_vs_speed(axes[0, 0], ffb)
    plot_torque_vs_angle(axes[0, 1], ffb)
    plot_lateral_effect_vs_speed(axes[0, 2], ffb)
    plot_friction_vs_steer_rate(axes[1, 0], ffb)
    plot_total_torque_vs_speed(axes[1, 1], ffb)
    plot_total_torque_vs_angle(axes[1, 2], ffb)

    # 自动调整布局并展示
    plt.tight_layout()
    plt.show()