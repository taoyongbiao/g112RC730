import logging
import matplotlib
import math
import numpy as np
# from scipy import signal
#
# from scipy.signal import butter, lfilter, lfilter_zi

# 悬架震动模拟#
# class VibrationTorqueGenerator:
#     def __init__(self, sample_rate=100, cutoff_freq=(5, 20)):
#         self.sample_rate = sample_rate
#         self.cutoff_freq = cutoff_freq
#         # 设计带通滤波器（提取 5~20Hz 的路感振动）
#         self.sos = signal.butter(4, cutoff_freq, 'bandpass', fs=sample_rate, output='sos')
#         self.buffer = np.zeros(100)  # 环形缓冲区存储最新悬架位移
#
#     def update_torque(self, new_suspension_travel):
#         # 1. 更新缓冲区（模拟实时数据流）
#         self.buffer = np.roll(self.buffer, -1)
#         self.buffer[-1] = new_suspension_travel[0]  # 仅用左前轮数据
#
#         # 2. 滤波提取振动分量
#         filtered = signal.sosfilt(self.sos, self.buffer)
#
#         # 3. 计算最近 10 个点的振动能量（RMS）
#         rms = np.sqrt(np.mean(filtered[-10:] ** 2))
#
#         # 4. 映射到力矩（0.3 Nm 每 mm RMS）
#         return np.clip(rms * 0.3, -2.0, 2.0)

# 修复中文乱码
matplotlib.rcParams['font.sans-serif'] = ['SimHei']  # 设置中文字体
matplotlib.rcParams['axes.unicode_minus'] = False    # 正负号显示正常

torque_table = {
    450: {0: 0, 10: 22, 30: 60, 60: 70, 100: 80, 150: 90, 210: 100},
    420: {0: 0, 10: 20, 30: 50, 60: 60, 100: 70, 150: 80, 210: 90},
    380: {0: 0, 10: 18, 30: 40, 60: 50, 100: 60, 150: 70, 210: 80},
    350: {0: 0, 10: 16, 30: 32, 60: 42, 100: 52, 150: 62, 210: 72},
    270: {0: 0, 10: 14, 30: 25, 60: 35, 100: 40, 150: 50, 210: 60},
    210: {0: 0, 10: 12, 30: 20, 60: 25, 100: 30, 150: 35, 210: 40},
    150: {0: 0, 10: 10, 30: 15, 60: 20, 100: 25, 150: 30, 210: 35},
    90: {0: 0, 10: 8, 30: 13, 60: 17, 100: 20, 150: 25, 210: 30},
    60: {0: 0, 10: 6, 30: 11, 60: 13, 100: 16, 150: 18, 210: 20},
    20: {0: 0, 10: 4, 30: 9, 60: 10, 100: 11, 150: 12, 210: 13},
    0: {0: 0, 10: 0, 30: 0, 60: 0, 100: 0, 150: 0, 210: 0},
}

class ForceFeedbackAlgorithm:
    def __init__(self):

        self.max_total_torque = 5000
        self.max_lateral_effect = 1000
        self.static_friction = 2000 #最大摩擦力 1300 mNm
        self.max_tire_effect = 1000
        self.max_road_effect = 1000
        self.max_g_effect = 1000
        self.max_desired_torque = 3000  # 最大扭矩 32767 mNm (约32.7 Nm) 最大5Nm #线控6000 #第二辆车9000 #调斜率之后更改
        self.max_damping = 5000  # 最大阻尼 500 mNm/rev/min
        self.max_spring = 2500  # 最大弹簧刚度 2500 mNm/1°
        self.limit_stiffness = 2.0
        self.max_limit_torque = 10000
        self.max_angle = 450

        # 状态变量
        self.prev_steer = 0.0
        self.filtered_torque = 0.0

        self.filtered_rate = 0.0
        self.last_sign = 0.0

        # self.vib_gen = VibrationTorqueGenerator()

    def get_torque_with_dynamic_effect(self, speed, steer_deg, steer_rate, wheel_slip, acc_g, angular_vel, suspension):
        desired_torque = 0
        tire_effect = 0
        lateral_effect = 0
        road_effect = 0
        #0626修改
        if speed <= 1:
            desired_torque = 0
            tire_effect = 0
            lateral_effect = 0
            road_effect = 0

        if speed > 1:  # 动摩擦
            desired_torque = self.get_table_torque(speed, abs(steer_deg)) * 0.01 * self.max_desired_torque

            # 轮胎滑移效应 (前轮为主)
            front_slip = (wheel_slip[0] + wheel_slip[1]) * 0.5
            tire_effect = math.tanh(front_slip) * self.max_tire_effect
            tire_effect = max(-self.max_tire_effect, min(self.max_tire_effect, tire_effect))

            # 动态效应 (横向加速度 + 横摆角速度)
            lateral_effect = (acc_g.accg[1] + angular_vel[1]) * self.max_g_effect
            lateral_effect = max(-self.max_g_effect, min(self.max_g_effect, lateral_effect))

            # 路面效应 (悬挂运动)
            road_effect = (abs(suspension[0] - suspension[1])) * self.max_road_effect
            road_effect = max(-self.max_road_effect, min(self.max_road_effect, road_effect))

            if steer_deg > 0:
                desired_torque = desired_torque * -1

        friction = self.get_friction(steer_rate)
        damping = self.max_damping * steer_rate

        desired_torque = max(-self.max_desired_torque, min(self.max_desired_torque, desired_torque))
        friction = max(-self.static_friction, min(self.static_friction, friction))
        damping = max(0, min(self.max_damping, damping))

        return tire_effect, lateral_effect, road_effect, desired_torque, friction, damping

    def get_torque(self, speed, angle, steer_rate, deadzone, mode):
        if abs(angle) < deadzone:
            return 0, 0, 0

        desired_torque = 0

        if speed > 1:  #静摩擦
            if mode == "table":
                desired_torque = -1*self.get_table_torque(speed, abs(angle)) * 0.01 * self.max_desired_torque
            elif mode == "tanh":
                desired_torque = self.get_tanh_torque(speed, abs(angle))
            if angle > 0:
                desired_torque = desired_torque * -1

        # print(steer_rate, speed, angle, desired_torque,friction)
        friction = self.get_friction(steer_rate)
        damping = self.max_damping * steer_rate/1080

        desired_torque = max(-self.max_desired_torque, min(self.max_desired_torque, desired_torque))
        friction = max(-self.static_friction, min(self.static_friction, friction))
        damping = max(-self.max_damping, min(self.max_damping, damping))

        return -1*desired_torque, friction, damping

    def get_friction(self, steer_deg, steer_rate, speed=0, mu_c=0.7, v_s=3.0, delta=2.0, sigma=0.002,
                              deadzone=4):

        steer_rate = np.where(np.abs(steer_rate) < np.abs(deadzone), 0, steer_rate)

        # if abs(steer_deg) < 0.1:
        #     friction = 0

        mu_s = 1.3  # self.static_friction * 0.001
        steer_rate_rad = steer_rate * 3.1415 / 180.0  # deg/s方向盘角速度（rad/s）
        steer_rate_rad = steer_rate_rad * np.tanh(np.abs(steer_rate_rad)) / np.deg2rad(deadzone)

        epsilon = 2.0
        if steer_rate_rad == 0.0:
            smoothed_sign = self.last_sign
        else:
            smoothed_sign = np.tanh(steer_rate_rad / epsilon)
            self.last_sign = smoothed_sign

        friction = (mu_c + (mu_s - mu_c) * np.exp(-(np.abs(steer_rate_rad) / v_s) ** delta)) + sigma * np.abs(
            steer_rate_rad)
        friction = - friction * smoothed_sign * self.static_friction / 8

        damping = - self.max_damping * steer_rate / 1080

        return friction, damping


    # 悬挂效应 通过前后轮的悬架高度差计算
    def get_suspension_effect(self, speed, suspension_travel):
        sus_front = (suspension_travel.st[0] + suspension_travel.st[1])/2
        sus_back = (suspension_travel.st[2]+suspension_travel.st[3])/2

        sus_diff = sus_front - sus_back

        road_effect = sus_diff * self.max_road_effect * 25
        # if speed > 1:  # 动摩擦
        #     # 路面效应 (悬挂运动)
        #     road_effect = self.vib_gen.update_torque(suspension_travel) * self.max_road_effect*500

        return np.clip(road_effect, -self.max_road_effect, self.max_road_effect)


    # 侧滑效应 g效应 + 轮胎滑移效进行计算，同时通过速度缩放进行缩放
    def get_lateral_effect(self, speed, acc_g, angular_vel, wheel_slip):
        speed_factor = np.tanh(speed / 400)

        g_effect_factor = 1.5
        g_effect = (acc_g.accg[1] + angular_vel.VehAngVel[1]) * g_effect_factor
        # print("==================",angular_vel[1])

        tire_effect_factor = 1.5
        front_slip = (wheel_slip[0] + wheel_slip[1])
        tire_effect = math.tanh(front_slip) * tire_effect_factor

        lateral_effect = (tire_effect + g_effect) * speed_factor * self.max_lateral_effect
        return np.clip(lateral_effect, -self.max_lateral_effect, self.max_lateral_effect)

    def get_tanh_torque(self, speed, angle):

        if abs(angle) < 0.5 or speed < 1.5:
            return 0

        angle_norm = angle / self.max_angle

        def smooth_slip_ratio(angle, speed):
            k = 3.0
            if angle < 60:
                gain = np.clip(angle/20, 0.5, 3.0)  # 在大角度时增加增益
            else:
                gain = angle/40

            if speed < 100:
                speed_norm = np.clip(speed / 30, 1.5, 2.4)
            else:
                speed_norm = np.clip(speed / 120, 0.0, 1.6) + 2

            return np.tanh(k * angle * gain * speed_norm)

        angle_ratio = smooth_slip_ratio(angle_norm, speed)

        angle_gain = 1 - np.clip(np.abs(angle_ratio), 0, 1.0)

        # angle_factor = np.clip(abs(angle) / 60,1.0,2.0)

        torque_norm = np.tanh(angle_gain * angle_ratio) + 0.7 * angle_ratio

        if 0<= speed <=90:
            speed_gain =0.75*(speed/90)
        else:
            speed_gain = 0.75+0.25*((speed-90)/30)

        speed_gain = np.clip(speed_gain,1.4,2.0)

        # torque = -self.max_desired_torque * torque_norm * angle_factor
        torque = -self.max_desired_torque * torque_norm * speed_gain
        # #
        # damping_bias = 300 / 720 * angle
        # torque = torque + damping_bias

        return torque

    def get_limit_torque(self, current_angle):
        soft_limit_start = 0.3

        angle = abs(current_angle)
        angle_ratio = angle / self.max_angle
        excess_ratio = max(0, angle_ratio - soft_limit_start) / (1 - soft_limit_start)

        base_torque = np.tanh(10 * excess_ratio ** 3)  # 使用tanh函数确保平滑

        # 限位刚度随角度变化（越接近极限越硬）
        dynamic_stiffness = self.limit_stiffness * (1.0 + 2.0 * excess_ratio ** 2)

        # 最终力矩计算
        final_torque = dynamic_stiffness * base_torque

        if angle >= self.max_angle:
            final_torque = min(final_torque, self.max_limit_torque)  # 设置最大力矩上限

        # 根据转向方向返回带符号的力矩
        return np.copysign(final_torque, -current_angle) * 1000

    def get_table_torque(self, speed, angle):
        # 获取所有角度和速度键并排序
        angles = sorted(torque_table.keys(), reverse=True)
        speeds = sorted(torque_table[angles[0]].keys())

        # 处理超出范围的角度
        angle = max(min(angle, angles[0]), angles[-1])

        # 处理超出范围的速度
        speed = max(min(speed, speeds[-1]), speeds[0])

        # 找到包围angle的两个角度
        lower_angle = angles[0]
        upper_angle = angles[-1]
        for a in angles:
            if a >= angle:
                lower_angle = a
            else:
                upper_angle = a
                break

        # 找到包围speed的两个速度
        lower_speed = speeds[0]
        upper_speed = speeds[-1]
        for s in speeds:
            if s <= speed:
                lower_speed = s
            else:
                upper_speed = s
                break

        # 如果正好匹配表中的值，直接返回
        if angle in torque_table and speed in torque_table[angle]:
            return torque_table[angle][speed]

        # 双线性插值
        if lower_angle == upper_angle and lower_speed == upper_speed:
            return torque_table[lower_angle][lower_speed]

        elif lower_angle == upper_angle:
            # 只在速度方向插值
            torque_low = torque_table[lower_angle][lower_speed]
            torque_high = torque_table[lower_angle][upper_speed]
            if upper_speed == lower_speed:
                return torque_low
            return torque_low + (torque_high - torque_low) * (speed - lower_speed) / (upper_speed - lower_speed)

        elif lower_speed == upper_speed:
            # 只在角度方向插值
            torque_low = torque_table[lower_angle][lower_speed]
            torque_high = torque_table[upper_angle][lower_speed]
            if upper_angle == lower_angle:
                return torque_low
            return torque_low + (torque_high - torque_low) * (angle - lower_angle) / (upper_angle - lower_angle)
        else:
            # 双线性插值
            # 首先在角度方向插值，得到两个速度点的扭矩
            torque_low_speed = torque_table[lower_angle][lower_speed] + \
                               (torque_table[upper_angle][lower_speed] - torque_table[lower_angle][lower_speed]) * \
                               (angle - lower_angle) / (upper_angle - lower_angle)

            torque_high_speed = torque_table[lower_angle][upper_speed] + \
                                (torque_table[upper_angle][upper_speed] - torque_table[lower_angle][upper_speed]) * \
                                (angle - lower_angle) / (upper_angle - lower_angle)

            # 然后在速度方向插值
            return torque_low_speed + (torque_high_speed - torque_low_speed) * \
                (speed - lower_speed) / (upper_speed - lower_speed)


if __name__ == "__main__":
    ffb = ForceFeedbackAlgorithm()
    ffb.plot_sine_torque_curve()
    ffb.get_friction_damping(0)