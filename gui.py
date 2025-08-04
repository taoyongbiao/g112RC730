import sys
import numpy as np
import matplotlib
matplotlib.use('Qt5Agg')
from PySide6.QtWidgets import QApplication,QGroupBox, QMainWindow, QPushButton, QVBoxLayout, QWidget, QLabel, QHBoxLayout,QRadioButton, QButtonGroup
from PySide6.QtCore import QTimer
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import threading  #
import wifi_module
import time
from DCH_VR_0630 import start_main_process,torque_data
from shared_state import run_main_flag_event
from loguru import logger




class RealTimePlotWindow(QMainWindow):
    def __init__(self, config_ready_event,config=None):
        super().__init__()
        self.config = config or {}
        self.USE_WIFI = self.config.get('USE_WIFI', False)
        self.USE_REAL_CAN = self.config.get('USE_REAL_CAN', False)
        self.USE_REAL_AC = self.config.get('USE_REAL_AC', False)
        self.USE_RC = self.config.get('USE_RC', False)  # 初始化 USE_RC 属性

        self.setWindowTitle("Real-time Torque Plot")
        self.setGeometry(100, 100, 1200, 800)
        self.config_ready_event = config_ready_event

                # 接收传入的 torque_data
        self.torque_data = torque_data or {
            'time': [],
            'total_torque': [],
            'scale_torque': [],
            'desired_torque': [],
            'damping': [],
            'friction': [],
            'steering_angle': [],
            'steering_rate': [],
            'rate_dir': []
        }


        # self.thread_manager = WrappedThreadManager(self)  # 初始化线程管理器
        self.ac_api = None
        self.zcanlib = None
        self.chn_handle = None

        # 新增线程控制标志
        # self.running_threads = []
        

        # === 新增通信状态变量 ===
        self.is_connected = False  # 初始为断开状态
        

        # 初始化界面
        self._init_ui()

        #主线程
        self.main_thread = None
        ...

        # 启动配置监听
        self.start_config_monitor()

    def start_config_monitor(self):
        self.config_timer = QTimer(self)
        self.config_timer.timeout.connect(self.check_config_and_start)
        self.config_timer.start(100)  

    def check_config_and_start(self):
        if self.config_ready_event.is_set():
            #print("开始执行主流程，当前配置：")
            logger.info("开始执行主流程，当前配置：")
            #print(f"USE_WIFI: {self.config['USE_WIFI']}, USE_REAL_CAN: {self.config['USE_REAL_CAN']}, USE_REAL_AC: {self.config['USE_REAL_AC']}")
            logger.debug(f"USE_WIFI: {self.config['USE_WIFI']}, USE_REAL_CAN: {self.config['USE_REAL_CAN']}, USE_REAL_AC: {self.config['USE_REAL_AC']}")

            if not hasattr(self, 'main_thread') or self.main_thread is None or not self.main_thread.is_alive():
                self.main_thread = threading.Thread(target=start_main_process, args=(self.config,self))#把 self 传给 window
                self.main_thread.daemon = True
                self.main_thread.start()

            # 可选：停止定时器
            # self.config_timer.stop()


    # gui.py - RealTimePlotWindow 类中添加：
    def set_zcanlib(self, zcanlib):
        self.zcanlib = zcanlib
    def reset_all_states(self):
    # """ 清除所有运行时状态 """
        # print("Stopping all threads...")
        

        # for thread in self.running_threads:
        #     if thread.is_alive():
        #         thread.join(timeout=1)  # 等待线程结束，最多等待1秒

        # self.running_threads.clear()  # 清空线程列表

        # 清空图表数据
        for key in self.torque_data:
            self.torque_data[key].clear()

        

        # 重置图表
        self.line1.set_data([], [])
        self.line2.set_data([], [])
        for bar in self.bars:
            bar.set_height(0)
        for text in self.text_objects:
            text.set_text("")
        self.canvas.draw()

        #重置监听是否开启主逻辑
        self.config_ready_event.clear()  # 停止信号

        wifi_module.wifi_flag_event.clear()
        run_main_flag_event.clear()

        # # 重置CAN设备（如果支持）

        if hasattr(self, 'zcanlib') and self.zcanlib is not None:
            try:
                if hasattr(self.zcanlib, 'chn_handle'):
                    self.zcanlib.ResetCAN(self.zcanlib.chn_handle)
                if hasattr(self.zcanlib, 'device_handle'):
                    self.zcanlib.CloseDevice(self.zcanlib.device_handle)
                #print("成功重置并关闭 CAN 设备")
                logger.info("成功重置并关闭 CAN 设备")
            except Exception as e:
                #print(f"无法重置或关闭 CAN 设备: {e}")
                logger.info(f"无法重置或关闭 CAN 设备: {e}")
            finally:
                self.zcanlib = None  # 清除引用，避免后续误操作        
    def toggle_connection(self):
        self.is_connected = not self.is_connected #每次点击会在中断状态之间切换

        if self.is_connected:
            self.conn_status_label.setText("Communication: 🟢 Connected")
            self.toggle_conn_button.setText("🔴 Disconnect")
            self.config['USE_WIFI'] = self.wifi_on_radio.isChecked()
            self.config['USE_REAL_CAN'] = self.can_on_radio.isChecked()
            self.config['USE_REAL_AC'] = self.ac_on_radio.isChecked()
            self.config['USE_RC'] = self.rc_on_radio.isChecked()  # 更新 USE_RC 配置
            self.config_ready_event.set()  # 触发开始信号
            run_main_flag_event.set()
            wifi_module.wifi_flag_event.set()
            # wifi_module.stop_flag_event.set()
        else:
            self.conn_status_label.setText("Communication: ⚪ Disconnected")
            self.toggle_conn_button.setText("🟢 Connect")

            # 设置停止标志
            self.reset_all_states()

    # def set_running_threads(self, threads):
    #     self.running_threads.extend(threads)

    def reset_plot_data(self):
        """ 清空所有数据 """
        for key in self.torque_data:
            self.torque_data[key].clear()
        self.line1.set_data([], [])
        self.line2.set_data([], [])
        for bar in self.bars:
            bar.set_height(0)
        for text in self.text_objects:
            text.set_text("")
        self.canvas.draw()
    def _init_ui(self):


        main_widget = QWidget()
        self.setCentralWidget(main_widget)

        # 创建垂直布局
        layout = QVBoxLayout(main_widget)

        # === 顶部控制面板：水平布局 ===
        control_layout = QHBoxLayout()  # 水平排列控件

        # WiFi 设置
        wifi_group_box = QGroupBox("WiFi")
        wifi_layout = QHBoxLayout()
        self.wifi_on_radio = QRadioButton("ON")
        self.wifi_off_radio = QRadioButton("OFF")
        self.wifi_group = QButtonGroup()
        self.wifi_group.addButton(self.wifi_on_radio)
        self.wifi_group.addButton(self.wifi_off_radio)
        self.wifi_on_radio.setChecked(self.USE_WIFI)
        self.wifi_off_radio.setChecked(not self.USE_WIFI)
        wifi_layout.addWidget(self.wifi_on_radio)
        wifi_layout.addWidget(self.wifi_off_radio)
        wifi_group_box.setLayout(wifi_layout)

        # CAN 设置
        can_group_box = QGroupBox("CAN")
        can_layout = QHBoxLayout()
        self.can_on_radio = QRadioButton("ON")
        self.can_off_radio = QRadioButton("OFF")
        self.can_group = QButtonGroup()
        self.can_group.addButton(self.can_on_radio)
        self.can_group.addButton(self.can_off_radio)
        self.can_on_radio.setChecked(self.USE_REAL_CAN)
        self.can_off_radio.setChecked(not self.USE_REAL_CAN)
        can_layout.addWidget(self.can_on_radio)
        can_layout.addWidget(self.can_off_radio)
        can_group_box.setLayout(can_layout)

        # AC 设置
        ac_group_box = QGroupBox("AC")
        ac_layout = QHBoxLayout()
        self.ac_on_radio = QRadioButton("ON")
        self.ac_off_radio = QRadioButton("OFF")
        self.ac_group = QButtonGroup()
        self.ac_group.addButton(self.ac_on_radio)
        self.ac_group.addButton(self.ac_off_radio)
        self.ac_on_radio.setChecked(self.USE_REAL_AC)
        self.ac_off_radio.setChecked(not self.USE_REAL_AC)
        ac_layout.addWidget(self.ac_on_radio)
        ac_layout.addWidget(self.ac_off_radio)
        ac_group_box.setLayout(ac_layout)


        # RC 控制
        rc_group_box = QGroupBox("RC")
        rc_layout = QHBoxLayout()
        self.rc_on_radio = QRadioButton("ON")
        self.rc_off_radio = QRadioButton("OFF")
        self.rc_group = QButtonGroup()
        self.rc_group.addButton(self.rc_on_radio)
        self.rc_group.addButton(self.rc_off_radio)
        self.rc_on_radio.setChecked(self.USE_RC)  # 假设 USE_RC 是 RealTimePlotWindow 的属性
        self.rc_off_radio.setChecked(not self.USE_RC)
        rc_layout.addWidget(self.rc_on_radio)
        rc_layout.addWidget(self.rc_off_radio)
        rc_group_box.setLayout(rc_layout)

        # # 创建确认按钮
        # self.confirm_button = QPushButton("Confirm & Start")
        # self.confirm_button.clicked.connect(self.on_confirm)

        # # 将三个 QGroupBox 添加进 control_layout 中以显示出来
        # control_layout.addWidget(wifi_group_box)  # 新增：添加 WiFi 分组框
        # control_layout.addWidget(can_group_box)   # 新增：添加 CAN 分组框
        # control_layout.addWidget(ac_group_box)    # 新增：添加 AC 分组框


        # control_layout.addWidget(self.confirm_button)  # ← 将按钮放在这里
        # control_layout.addStretch()  # 可选：右侧留白防止控件太挤

        # # 将顶部控制面板添加到主布局
        # layout.addLayout(control_layout)



        # === 新增：通信状态标签 ===
        self.conn_status_label = QLabel("Communication: ⚪ Disconnected")
        self.conn_status_label.setStyleSheet("font-size: 14px;")

        # === 修改：将确认按钮改为通信切换按钮 ===
        self.toggle_conn_button = QPushButton("🟢 Connect")
        self.toggle_conn_button.setCheckable(True)
        self.toggle_conn_button.setChecked(False)
        self.toggle_conn_button.clicked.connect(self.toggle_connection)

        # 将控件加入控制面板布局
        control_layout.addWidget(wifi_group_box)
        control_layout.addWidget(can_group_box)
        control_layout.addWidget(ac_group_box)
        control_layout.addWidget(rc_group_box)    # 新增：添加 RC 分组框
        control_layout.addWidget(self.conn_status_label)  # 新增状态标签
        control_layout.addWidget(self.toggle_conn_button)  # 替换原来的 confirm_button

        control_layout.addStretch()
        layout.addLayout(control_layout)



        #======图表显示区域======
        self.figure = Figure()
        self.ax1 = self.figure.add_subplot(211)
        self.ax2 = self.figure.add_subplot(212)
        self.canvas = FigureCanvas(self.figure)
        layout.addWidget(self.canvas)



        # self.canvas.mpl_connect('button_press_event', self.onclick)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(500)

        # 初始化图表
        self.line1, = self.ax1.plot([], [], 'b-', label='Total Torque')
        self.line2, = self.ax1.plot([], [], 'r--', label='Scale Torque')
        self.ax1.legend()
        self.ax1.grid(True)

        bar_labels = ['Desired', 'Damping', 'Friction', 'Steer Angle', 'Steer Rate', 'Rate Dir']
        self.bars = self.ax2.bar(bar_labels, [0]*len(bar_labels), color=[
            'blue', 'green', 'orange', 'purple', 'red', 'brown'
        ])
        self.text_objects = [self.ax2.text(0, 0, "", ha='center', va='bottom') for _ in self.bars]

    def on_confirm(self):
        self.config['USE_WIFI'] = self.wifi_on_radio.isChecked()
        self.config['USE_REAL_CAN'] = self.can_on_radio.isChecked()
        self.config['USE_REAL_AC'] = self.ac_on_radio.isChecked()
        #print("Configuration confirmed.")
        logger.info("Configuration confirmed.")
        self.config_ready_event.set()
        # 主动调用一次 update_plot 更新图表
        self.update_plot()

        # self.hide()

    # def onclick(self, event):
    #     x = event.x
    #     width = self.width()
    #     if x < 0.3 * width:
    #         self.USE_WIFI = not self.USE_WIFI
    #         self.wifi_label.setText(f"USE_WIFI: {'ON' if self.USE_WIFI else 'OFF'}")
    #     elif x < 0.6 * width:
    #         self.USE_REAL_CAN = not self.USE_REAL_CAN
    #         self.can_label.setText(f"USE_REAL_CAN: {'ON' if self.USE_REAL_CAN else 'OFF'}")
    #     else:
    #         self.USE_REAL_AC = not self.USE_REAL_AC
    #         self.ac_label.setText(f"USE_REAL_AC: {'ON' if self.USE_REAL_AC else 'OFF'}")

    def update_plot(self):
        t = self.torque_data['time']
        total = self.torque_data['total_torque']
        scale = self.torque_data['scale_torque']

        if len(t) > 0:
            self.line1.set_data(t, total)
            self.line2.set_data(t, scale)
            self.ax1.set_xlim(max(0, t[-1] - 10), t[-1] + 1)
            current_min = min(min(total), min(scale)) - 100 if len(total) > 0 else -2000
            current_max = max(max(total), max(scale)) + 100 if len(total) > 0 else 2000
            self.ax1.set_ylim(current_min, current_max)

        keys = ['desired_torque', 'damping', 'friction', 'steering_angle', 'steering_rate', 'rate_dir']
        values = [np.mean(self.torque_data[key]) if self.torque_data[key] else 0 for key in keys]
        max_val = max(values) if values else 1
        offset = max(0.02 * max_val, 20)

        for i, val in enumerate(values):
            self.bars[i].set_height(val)
            self.text_objects[i].set_position((self.bars[i].get_x() + self.bars[i].get_width()/2., val + offset))
            self.text_objects[i].set_text(f'{val:.1f}')

        self.ax2.set_ylim(top=max_val * 1.2 if max_val > 0 else 1)
        self.canvas.draw()