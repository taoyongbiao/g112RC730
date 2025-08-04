import numpy as np
import matplotlib.pyplot as plt
# from ffb_cal_0630 import ForceFeedbackAlgorithm
from ffb_cal_0624 import ForceFeedbackAlgorithm
import ctypes  # 修复：添加缺失的导入语句

# 初始化算法类
ffb = ForceFeedbackAlgorithm()

# 设置绘图样式
plt.style.use('ggplot')


def plot_torque_vs_speed(ax):
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


def plot_torque_vs_angle(ax):
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


def plot_lateral_effect_vs_speed(ax):
    steer_angles = [ctypes.c_float(45), ctypes.c_float(90), ctypes.c_float(180)]
    wheel_slip = [0.0] * 4
    acc_g = [0.0] * 3
    local_angular_vel = [0.0] * 3

    speeds = np.linspace(0, 200, 300)
    for sa in steer_angles:
        lateral_effects = [
            ffb.get_lateral_effect(
                speed,
                (acc_g[0], acc_g[1], acc_g[2]),
                (local_angular_vel[0], local_angular_vel[1], local_angular_vel[2]),
                (wheel_slip[0], wheel_slip[1], wheel_slip[2], wheel_slip[3])
            ) for speed in speeds
        ]
        ax.plot(speeds, lateral_effects, label=f"Steer Angle = {sa.value}°")

    ax.set_title("Lateral Effect vs Speed")
    ax.set_xlabel("Speed (km/h)")
    ax.set_ylabel("Lateral Effect")
    ax.legend()
    ax.grid(True)


def plot_friction_vs_steer_rate(ax):
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
def plot_total_torque_vs_speed(ax):
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


def plot_total_torque_vs_angle(ax):
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
    # 创建 2 行 3 列 的画布，总共有 6 个子图
    fig, axes = plt.subplots(2, 3, figsize=(18, 10))

    # 绘制原有两个图
    plot_torque_vs_speed(axes[0, 0])
    plot_torque_vs_angle(axes[0, 1])

    # 新增两个图
    plot_lateral_effect_vs_speed(axes[0, 2])
    plot_friction_vs_steer_rate(axes[1, 0])

    # 新增 total_torque 图形
    plot_total_torque_vs_speed(axes[1, 1])
    plot_total_torque_vs_angle(axes[1, 2])

    # 自动调整布局并展示
    plt.tight_layout()
    plt.show()