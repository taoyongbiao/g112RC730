import sys
import numpy as np
import matplotlib
matplotlib.use('Qt5Agg')
from PySide6.QtWidgets import (QApplication,QGroupBox, QMainWindow, QPushButton, QVBoxLayout, QWidget, 
                                QLabel, QHBoxLayout,QRadioButton, QButtonGroup,QTextEdit, QTabWidget,
                                QMenuBar, QMenu, QFileDialog,QDialog)
from PySide6.QtCore import QTimer, Qt,Signal, QObject  # 添加 Qt

import matplotlib.pyplot as plt
import ctypes 

from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import threading  #
import wifi_module
import time
from DCH_VR_0630 import start_main_process,torque_data
from shared_state import run_main_flag_event
from loguru import logger

import io
import sys
from contextlib import redirect_stdout

# 在 gui.py 文件开头添加导入
import os
import json
from pathlib import Path

# 调用原 plt_ffb.py 中的绘图函数
from ffb_cal_ori import ForceFeedbackAlgorithm

ffb = ForceFeedbackAlgorithm()

# 在 gui.py 中修改 FFBPlotPage 类

# 首先添加导入
from plt_ffb import (plot_torque_vs_speed, plot_torque_vs_angle, 
                     plot_lateral_effect_vs_speed, plot_friction_vs_steer_rate,
                     plot_total_torque_vs_speed, plot_total_torque_vs_angle,
                      plot_lateral_effect_vs_angular_vel)

import os
import json
from pathlib import Path


class LogPage(QDialog):
    """日志显示弹窗"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("日志配置")
        self.setGeometry(200, 200, 800, 600)
        self.setModal(False)  # 设置为非模态窗口，允许与其他窗口交互
        self.init_ui()
        
    def init_ui(self):
        layout = QVBoxLayout(self)
        
        # 添加日志显示区域
        self.log_text_edit = QTextEdit()
        self.log_text_edit.setReadOnly(True)
        
        # 添加按钮布局
        button_layout = QHBoxLayout()
        
        # 清除日志按钮
        self.clear_log_button = QPushButton("清除日志")
        self.clear_log_button.clicked.connect(self.clear_log)
        
        # 保存日志按钮
        self.save_log_button = QPushButton("保存日志")
        self.save_log_button.clicked.connect(self.save_log)
        
        # 关闭按钮
        self.close_button = QPushButton("关闭")
        self.close_button.clicked.connect(self.close)
        
        button_layout.addWidget(self.clear_log_button)
        button_layout.addWidget(self.save_log_button)
        button_layout.addWidget(self.close_button)
        button_layout.addStretch()
        
        layout.addLayout(button_layout)
        layout.addWidget(self.log_text_edit)
        

        
    def append_log(self, message):
        """添加日志信息"""
        self.log_text_edit.append(message)
        
    def clear_log(self):
        """清除日志"""
        self.log_text_edit.clear()
        
    def save_log(self):
        """保存日志到文件"""
        filename, _ = QFileDialog.getSaveFileName(self, "Save Log", "", "Text Files (*.txt);;All Files (*)")
        if filename:
            try:
                with open(filename, 'w', encoding='utf-8') as f:
                    f.write(self.log_text_edit.toPlainText())
            except Exception as e:
                # 可以添加错误提示
                pass






class FFBPlotPage(QWidget):
    def __init__(self):
        super().__init__()
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout(self)

        # 创建 Matplotlib Figure
        self.figure = plt.figure(figsize=(10, 8))
        axes = self.figure.subplots(2, 3)

        # 使用全局的 ffb 实例
        plot_torque_vs_speed(axes[0, 0], ffb)
        plot_total_torque_vs_speed(axes[0, 1], ffb)
        plot_friction_vs_steer_rate(axes[0, 2], ffb)
        plot_torque_vs_angle(axes[1, 0], ffb)
        plot_total_torque_vs_angle(axes[1, 1], ffb) #总矩Torque vs 角度注释
        plot_lateral_effect_vs_angular_vel(axes[1, 2], ffb)  # 新增的图

        plt.tight_layout()

        # 创建 Canvas 并加入布局
        self.canvas = FigureCanvas(self.figure)
        layout.addWidget(self.canvas)




class CanMessagePage(QWidget):
    "can信号页面"
    # 添加信号用于线程安全的消息更新
    message_received = Signal(str)
    def __init__(self, main_window=None):
        super().__init__()
        self.parent_window = main_window  # 保存主窗口引用
        self.init_ui()
        self.filter_id = None  # 当前筛选的 CAN ID
        # 连接信号到槽函数
        self.message_received.connect(self._append_message_thread_safe)

    def init_ui(self):
        layout = QVBoxLayout()
        # self.filter_input = QLineEdit()
        # self.filter_input.setPlaceholderText("Filter by CAN ID (e.g. 0x8E)")
        # self.filter_input.textChanged.connect(self.apply_filter)

        self.text_edit = QTextEdit()  
        self.text_edit.setReadOnly(True)

        # layout.addWidget(self.filter_input)
        layout.addWidget(self.text_edit)
        self.setLayout(layout)
    def _append_message_thread_safe(self, message: str):
        """在GUI线程中实际添加消息"""
        self.text_edit.append(message)
        self.refresh_display()  # 可选自动刷新
    def apply_filter(self):
        text = self.filter_input.text().strip()
        if text:
            try:
                self.filter_id = int(text, 16)  # 支持输入 0x8E 形式
            except ValueError:
                self.filter_id = None
        else:
            self.filter_id = None
        self.refresh_display()

    def append_message(self, message: str):
        """线程安全地添加消息"""
        self.message_received.emit(message)

    def refresh_display(self):
        """根据 filter_id 刷新显示"""

        # 使用传递进来的主窗口引用
        if not self.parent_window or not hasattr(self.parent_window, 'can_data'):
            logger.error("Main window with can_data not found") 
            return
        
        self.text_edit.clear()
        for message in self.parent_window.can_data:  # 假设传入了 parent_window
            if self.filter_id is None or message.id == self.filter_id:
                self.text_edit.append(str(message))


class RealTimePlotWindow(QMainWindow):
    def __init__(self, config_ready_event,config=None):
        super().__init__()
        self.log_page = None  # 添加日志页面属性

        self.can_reader_thread = None  # 添加线程引用
        self.can_reader_stop_flag = False  # 添加线程停止标志


        # 获取项目根目录路径
        self.project_dir = Path(__file__).parent
        self.config_file = self.project_dir / "app_config.json"
        
        # 加载保存的设置
        saved_config = self.load_config()
        
        # 从设置中读取上次保存的配置，如果没有则使用默认值
        default_config = {
            'USE_WIFI': False,
            'USE_REAL_CAN': False,
            'USE_REAL_AC': False,
            'USE_RC': False,
        }
        
        # 合并默认配置、保存的配置和传入的配置
        self.config = {**default_config,  **(config or {}),**saved_config,}
        
        self.USE_WIFI = self.config.get('USE_WIFI', False)
        self.USE_REAL_CAN = self.config.get('USE_REAL_CAN', False)
        self.USE_REAL_AC = self.config.get('USE_REAL_AC', False)
        self.USE_RC = self.config.get('USE_RC', False)

        self.is_plot_visible = True  # 用于判断是否需要刷新图表


        # RealTimePlotWindow.__init__ 中添加：
        self.can_data = []  # 存储所有接收到的 CAN 帧




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
            'rate_dir': [],
            'lateral_effect':[],
            'suspension_effect':[]
        }


    
        self.ac_api = None
        self.zcanlib = None
        self.chn_handle = None

        # 新增线程控制标志
        # self.running_threads = []
        

        # === 新增通信状态变量 ===
        self.is_connected = False  # 初始为断开状态
        

        # 初始化界面
        self._init_ui()

        # 添加菜单栏
        self._create_menu_bar()

        #主线程
        self.main_thread = None
        ...

        # 启动配置监听
        self.start_config_monitor()

    def load_config(self):
        """从项目目录加载配置"""
        try:
            if self.config_file.exists():
                with open(self.config_file, 'r', encoding='utf-8') as f:
                    return json.load(f)
            else:
                return {}
        except Exception as e:
            logger.error(f"加载配置文件失败: {e}")
            return {}

    def save_config(self):
        """保存配置到项目目录"""
        try:
            # 确保配置目录存在
            self.config_file.parent.mkdir(parents=True, exist_ok=True)
            
            with open(self.config_file, 'w', encoding='utf-8') as f:
                json.dump(self.config, f, indent=4, ensure_ascii=False)
        except Exception as e:
            logger.error(f"保存配置文件失败: {e}")

    def _create_menu_bar(self):
        """创建菜单栏"""
        menubar = self.menuBar()
        
        # 创建"视图"菜单
        view_menu = menubar.addMenu('设置')
        
        # 添加"日志"动作
        log_action = view_menu.addAction('日志配置')
        log_action.triggered.connect(self.show_log_page)
        
    def show_log_page(self):
        """显示日志页面"""
        if self.log_page is None:
            self.log_page = LogPage(self)# 传入 parent 以保持窗口层级关系
            # self.tabs.addTab(self.log_page, "Log")
            
            # 捕获 logger 的输出
            def log_sink(message):
                if self.log_page and hasattr(self.log_page, 'log_text_edit'):
                    # 格式化日志消息
                    formatted_message = f"{message.time:YYYY-MM-DD HH:mm:ss} | {message.level.name: <8} | {message.message}"
                    self.log_page.append_log(formatted_message)
                    
            # 添加日志处理器
            logger.add(log_sink, level="INFO", format="{time:YYYY-MM-DD HH:mm:ss} | {level: <8} | {message}")
        
        # 显示日志窗口
        self.log_page.show()
        self.log_page.raise_()  # 将窗口提到最前
        self.log_page.activateWindow()  # 激活窗口
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

    #can数据读取线程
    def start_can_reader(self):
        self.can_reader_stop_flag = False  # 重置停止标志
        def can_reader():
            while not self.can_reader_stop_flag and self.is_connected:
                if self.zcanlib and self.chn_handle:
                    try:
                        # 统一使用 ReceiveFD 方法处理CAN数据
                        if hasattr(self.zcanlib, 'ReceiveFD') and hasattr(self.zcanlib, 'GetReceiveNum'):
                            # 获取可用帧数量
                            rcv_num = self.zcanlib.GetReceiveNum(self.chn_handle, 1)  # 1 for CANFD
                            if rcv_num > 0:
                                # 接收CAN FD帧
                                rcv_canfd_msgs, actual_num = self.zcanlib.ReceiveFD(self.chn_handle, rcv_num, 1000)
                                for i in range(actual_num):
                                    frame = rcv_canfd_msgs[i].frame
                                    self.can_data.append(frame)
                                    if hasattr(self, 'can_message_page') and self.can_message_page:
                                        self.can_message_page.append_message(str(frame))
                    except Exception as e:
                        logger.debug(f"CAN reading error: {e}")
                        pass
                time.sleep(0.01)  # 避免 CPU 占用过高

        self.can_reader_thread = threading.Thread(target=can_reader, daemon=True)
        self.can_reader_thread.start()

    
    def reset_all_states(self):
        # 设置停止标志来终止线程
        self.can_reader_stop_flag = True

        wifi_module.wifi_flag_event.clear()
        run_main_flag_event.clear()
        wifi_module.CONNECT_EVENT.clear()  # 如果使用WiFi也需要清除
        
        #重置监听是否开启主逻辑
        self.config_ready_event.clear()  # 停止信号

        # 等待CAN读取线程结束（设置超时避免无限等待）
        if self.can_reader_thread and self.can_reader_thread.is_alive():
            self.can_reader_thread.join(timeout=1.0)  # 最多等待1秒

        # 清空图表数据
        for key in self.torque_data:
            self.torque_data[key].clear()

        

        # 重置图表
        self.line1.set_data([], [])
        
        for bar in self.bars:
            bar.set_height(0)
        for text in self.text_objects:
            text.set_text("")
        self.canvas.draw()




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

            # 保存配置到本地文件
            self.save_config()
            self.config_ready_event.set()  # 触发开始信号
            run_main_flag_event.set()
            wifi_module.wifi_flag_event.set()
            # wifi_module.stop_flag_event.set()
            if self.config['USE_WIFI']:
                pass
            else:
                self.start_can_reader() #can数据读取线程
        

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
        layout.addLayout(control_layout)  #将control_layout添加到layout中


        # ====== 图表区域和 CAN 报文区域使用 Tab 控件 ======
        self.tabs = QTabWidget()







        #======图表显示区域======
        plot_widget = QWidget()
        plot_layout = QVBoxLayout(plot_widget)




        self.figure = Figure()
        self.ax1 = self.figure.add_subplot(211)
        self.ax2 = self.figure.add_subplot(212)
        self.canvas = FigureCanvas(self.figure)
        # layout.addWidget(self.canvas)#将绘图添加到窗口布局中
        plot_layout.addWidget(self.canvas)

        self.tabs.addTab(plot_widget, "Torque Plot")#将绘图区域添加进标签页



        # self.canvas.mpl_connect('button_press_event', self.onclick)
        # 定时器，定时刷新数据
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(500)

        # 初始化图表
        self.line1, = self.ax1.plot([], [], 'b-', label='Total Torque')
        # self.line2, = self.ax1.plot([], [], 'r--', label='Scale Torque')

        self.ax1.set_xlabel('Steering Angle (degrees)')
        self.ax1.set_ylabel('Torque')

        self.ax1.legend()
        self.ax1.grid(True)

        bar_labels = ['Desired', 'Damping', 'Friction',  'Lateral','suspension','Steer Angle', 'Steer Rate', 'Rate Dir']
        self.bars = self.ax2.bar(bar_labels, [0]*len(bar_labels), color=[
            'blue', 'green', 'orange', 'purple', 'red', 'brown', 'cyan','yellow'
        ])
        self.text_objects = [self.ax2.text(0, 0, "", ha='center', va='bottom') for _ in self.bars]


        # CAN 报文页
        # self.can_message_page = CanMessagePage()
        # 创建对象时传递参数
        self.can_message_page = CanMessagePage(self)
        self.tabs.addTab(self.can_message_page, "CAN Messages")#将CAN报文页添加到标签页

        # 添加 Tab 控件到主布局
        layout.addWidget(self.tabs)


        # FFB 分析图表页
        ffb_plot_page = FFBPlotPage()
        self.tabs.addTab(ffb_plot_page, "FFB Analysis")
        self.tabs.currentChanged.connect(self.on_tab_changed)  # 监听 Tab 切换



    # 切换标签页时触发 判断是不是绘图页
    def on_tab_changed(self, index):
        current_tab = self.tabs.tabText(index)
        self.is_plot_visible = (current_tab == "Torque Plot")
    def on_confirm(self):
        self.config['USE_WIFI'] = self.wifi_on_radio.isChecked()
        self.config['USE_REAL_CAN'] = self.can_on_radio.isChecked()
        self.config['USE_REAL_AC'] = self.ac_on_radio.isChecked()
        self.config['USE_RC ']= self.rc_on_radio.isChecked()

        # 保存配置到本地文件
        self.save_config()
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
        if not getattr(self, 'is_plot_visible', True):
            return  # 非当前 Tab 不刷新


        # t = self.torque_data['time']
        # 使用转向角数据替代时间数据作为 X 轴
        steering_angles = self.torque_data['steering_angle']
        total_torques = self.torque_data['total_torque']
        # scale_torques = self.torque_data['scale_torque']


        # if len(t) > 0:
        #     self.line1.set_data(t, total)
        #     self.line2.set_data(t, scale)
        #     self.ax1.set_xlim(max(0, t[-1] - 10), t[-1] + 1)
        #     current_min = min(min(total), min(scale)) - 100 if len(total) > 0 else -2000
        #     current_max = max(max(total), max(scale)) + 100 if len(total) > 0 else 2000
        #     self.ax1.set_ylim(current_min, current_max)

        if len(steering_angles) > 0 and len(total_torques) > 0:
        # 确保两个数组长度一致
        # 直接使用所有数据点，不截取最新数据
            # x_data = steering_angles
            # total_data = total_torques
            # scale_data = scale_torques

                    # 确保x轴和y轴数据长度一致
            min_length = min(len(steering_angles), len(total_torques))
            if min_length > 0:
                # 取最后min_length个数据点，确保数据长度一致
                x_data = steering_angles[-min_length:]
                total_data = total_torques[-min_length:]
                # scale_data = scale_torques[-min_length:]

            self.line1.set_data(x_data, total_data)
            # self.line2.set_data(x_data, scale_data)
            # 设置坐标轴范围为固定值
            self.ax1.set_xlim(-540, 540)
            self.ax1.set_ylim(-5000, 5000)        



        

        keys = ['desired_torque', 'damping', 'friction',  'lateral_effect','suspension_effect','steering_angle', 'steering_rate', 'rate_dir']
        values = [np.mean(self.torque_data[key]) if self.torque_data[key] else 0 for key in keys]
        max_val = max(values) if values else 1
        offset = max(0.02 * max_val, 20)

        for i, val in enumerate(values):
            self.bars[i].set_height(val)
            self.text_objects[i].set_position((self.bars[i].get_x() + self.bars[i].get_width()/2., val + offset))
            self.text_objects[i].set_text(f'{val:.1f}')

        self.ax2.set_ylim(top=max_val * 1.2 if max_val > 0 else 1)
        self.canvas.draw()