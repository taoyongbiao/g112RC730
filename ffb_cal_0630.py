import math
import numpy as np

class ForceFeedbackAlgorithm:
    def __init__(self):
        # self.max_total_torque = 5000
        # self.max_lateral_effect = 1000
        # self.static_friction = 2000 #最大摩擦力 1300 mNm
        # self.max_tire_effect = 1000
        # self.max_road_effect = 1000
        # self.max_g_effect = 1000
        # self.max_desired_torque = 3000  # 最大扭矩 32767 mNm (约32.7 Nm) 最大5Nm #线控6000 #第二辆车9000 #调斜率之后更改
        # self.max_damping = 5000  # 最大阻尼 500 mNm/rev/min
        # self.max_spring = 2500  # 最大弹簧刚度 2500 mNm/1°
        # self.limit_stiffness = 2.0
        # self.max_limit_torque = 10000
        # self.max_angle = 450
        self.last_sign = 0.0
        self.max_total_torque = 5000
        self.max_lateral_effect = 2000
        self.static_friction = 2000  # 最大摩擦力 1300 mNm
        self.max_tire_effect = 1000
        self.max_road_effect = 1000
        self.max_g_effect = 1000
        # self.max_desired_torque = 3000  # 最大扭矩 32767 mNm (约32.7 Nm) 最大5Nm #线控6000 #第二辆车9000 #调斜率之后更改
        self.max_desired_torque = 2000  # 最大扭矩 32767 mNm (约32.7 Nm) 最大5Nm #线控6000 #第二辆车9000 #调斜率之后更改
        self.max_damping = 5000  # 最大阻尼 500 mNm/rev/min
        self.max_spring = 2500  # 最大弹簧刚度 2500 mNm/1°
        self.limit_stiffness = 2.0
        self.max_limit_torque = 10000
        self.max_angle = 450


    def get_friction(self, steer_deg, steer_rate, deadzone=4):

        steer_rate = np.where(np.abs(steer_rate) < np.abs(deadzone), 0, steer_rate)

        steer_rate_rad = steer_rate * 3.1415 / 180.0
        steer_rate_rad = steer_rate_rad * np.tanh(np.abs(steer_rate_rad)) / np.deg2rad(deadzone)

        epsilon = 2.0
        if steer_rate_rad == 0.0:
            smoothed_sign = self.last_sign
        else:
            smoothed_sign = np.tanh(steer_rate_rad / epsilon)
            self.last_sign = smoothed_sign

        friction = (0.7 + 0.5 * np.exp(-(np.abs(steer_rate_rad) / 3) ** 2)) + 0.002 * np.abs(steer_rate_rad)
        friction = - friction * smoothed_sign * self.static_friction / 8

        damping = - self.max_damping * steer_rate / 1080

        return friction, damping

    def get_suspension_effect(self, speed, suspension_travel):
        sus_front = (suspension_travel[0] + suspension_travel[1])/2
        sus_back = (suspension_travel[2]+suspension_travel[3])/2
        sus_diff = sus_front - sus_back
        road_effect = sus_diff * self.max_road_effect * 25

        return np.clip(road_effect, -self.max_road_effect, self.max_road_effect)

    # 0624异常
    # def get_lateral_effect(self, speed, acc_g, angular_vel, wheel_slip):
    #     speed_factor = np.tanh(speed / 400)
    #
    #     g_effect_factor = 1.5
    #     g_effect = (acc_g[1] + angular_vel[1]) * g_effect_factor
    #
    #     tire_effect_factor = 1.5
    #     front_slip = (wheel_slip[0] + wheel_slip[1])
    #     tire_effect = math.tanh(front_slip) * tire_effect_factor
    #
    #     lateral_effect = (tire_effect + g_effect) * speed_factor * self.max_lateral_effect
    #     return np.clip(lateral_effect, -self.max_lateral_effect, self.max_lateral_effect)


    # 0630 科技展版本
    def get_lateral_effect(self, speed, acc_g, angular_vel, wheel_slip):
        speed_factor = np.tanh(speed / 400)

        g_effect_factor = 2
        g_effect = (acc_g[1] + angular_vel[1]) * g_effect_factor

        # print(g_effect)

        tire_effect_factor = 3
        front_slip = (wheel_slip[0] + wheel_slip[1])
        tire_effect = math.tanh(front_slip) * tire_effect_factor

        lateral_effect = (tire_effect + g_effect) * speed_factor * self.max_lateral_effect
        return np.clip(lateral_effect, -self.max_lateral_effect, self.max_lateral_effect)

    # # 0630 科技展版本
    # def get_tanh_torque(self, speed, angle):
    #
    #     if abs(angle) < 0.5 or speed < 1.5:
    #         return 0
    #
    #     angle_norm = angle / self.max_angle
    #
    #     def smooth_slip_ratio(angle, speed):
    #         k = 3.0
    #         if angle < 60:
    #             gain = np.clip(angle/20, 0.5, 3.0)  # 在大角度时增加增益
    #         else:
    #             gain = angle/40
    #
    #         if speed < 100:
    #             speed_norm = np.clip(speed / 30, 1.5, 2.4)
    #         else:
    #             speed_norm = np.clip(speed / 120, 0.0, 1.6) + 2
    #
    #         return np.tanh(k * angle * gain * speed_norm)
    #
    #     angle_ratio = smooth_slip_ratio(angle_norm, speed)
    #
    #     angle_gain = 1 - np.clip(np.abs(angle_ratio), 0, 1.0)
    #
    #     # angle_factor = np.clip(abs(angle) / 60,1.0,2.0)
    #
    #     torque_norm = np.tanh(angle_gain * angle_ratio) + 0.7 * angle_ratio
    #
    #     if 0 <= speed <=90:
    #         speed_gain =0.75*(speed/90)
    #     else:
    #         speed_gain = 0.75+0.25*((speed-90)/30)
    #
    #     speed_gain = np.clip(speed_gain,1.4,2.0)
    #
    #     # torque = -self.max_desired_torque * torque_norm * angle_factor
    #     torque = -self.max_desired_torque * torque_norm * speed_gain
    #     # #
    #     # damping_bias = 300 / 720 * angle
    #     # torque = torque + damping_bias
    #
    #     return torque

    # 0630 科技展版本
    def get_tanh_torque(self, speed, angle):

        if abs(angle) < 0.5 or speed < 1.5:
            return 0

        angle_norm = angle / self.max_angle

        def smooth_slip_ratio(angle, speed):
            k = 3.0
            # if angle < 60:
            #     gain = np.clip(angle/20, 0.5, 3.0)  # 在大角度时增加增益
            # else:
            #     gain = angle/40
            if angle < 60:
                gain = np.clip((angle ** 1.5) / 120, 0.5, 3.0)  # 快速上升
            else:
                gain = 3.0 + (np.log(angle - 59)) * 0.8  # 缓慢上升

            if speed < 100:
                speed_norm = np.clip(speed / 30, 1.5, 2.4)
            else:
                speed_norm = np.clip(speed / 120, 0.0, 1.6) + 2

            # # 新增：使用指数方式处理 speed_norm，强化低速响应
            # if speed < 50:
            #     speed_norm = 1.5 + 1.0 * np.exp(speed / 30)
            # elif speed < 100:
            #     speed_norm = 2.5 + 0.3 * ((speed - 50) / 50)
            # else:
            #     speed_norm = 2.8 + 0.2 * np.log(1 + (speed - 100) / 10)

            return np.tanh(k * angle * gain * speed_norm)

        angle_ratio = smooth_slip_ratio(angle_norm, speed)

        angle_gain = 1 - np.clip(np.abs(angle_ratio), 0, 1.0)

        # angle_factor = np.clip(abs(angle) / 60,1.0,2.0)

        torque_norm = np.tanh(angle_gain * angle_ratio) + 0.7 * angle_ratio

        # if 0 <= speed <=90:
        #     speed_gain =0.75*(speed/90)
        # else:
        #     speed_gain = 0.75+0.25*((speed-90)/30)
        #
        # speed_gain = np.clip(speed_gain,1.4,2.0)

        if speed <= 90:
            speed_gain = 1.4 + (0.6 * (speed / 90))  # 从1.4平滑过渡到2.0
        else:
            speed_gain = 2.0 + 0.5 * ((speed - 90) / 30)  # 速度超过90后继续增加增益

        speed_gain = np.clip(speed_gain, 1.4, 2.5)  # 扩展最大限制以适应高速度

        # if speed < 30:
        #     speed_gain = 1.4 + 1.2 * (speed / 30)  # 从1.4快速升到2.6
        # elif speed < 90:
        #     speed_gain = 2.6 + 0.4 * ((speed - 30) / 60)  # 平缓增加到3.0
        # else:
        #     speed_gain = 3.0 + 0.2 * np.log(1 + (speed - 90) / 10)  # 对数增长，避免过强反馈
        # speed_gain = np.clip(speed_gain, 1.4, 3.5)



        # torque = -self.max_desired_torque * torque_norm * angle_factor
        torque = -self.max_desired_torque * torque_norm * speed_gain
        # #
        # damping_bias = 300 / 720 * angle
        # torque = torque + damping_bias

        return torque


if __name__ == "__main__":
    ffb = ForceFeedbackAlgorithm()