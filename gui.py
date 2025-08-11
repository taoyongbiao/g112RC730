import sys
import numpy as np
import matplotlib
matplotlib.use('Qt5Agg')
# from PySide6.QtWidgets import (QApplication,QGroupBox, QMainWindow, QPushButton, QVBoxLayout, QWidget, 
# QLabel, QHBoxLayout,QRadioButton, QButtonGroup,QTextEdit, QTabWidget,
# QMenuBar, QMenu, QFileDialog,QDialog)

from PyQt5.QtWidgets import (QApplication, QGroupBox,          
        QMainWindow,QPushButton, QVBoxLayout, QWidget, 
        QLabel, QHBoxLayout, QRadioButton, QButtonGroup, QTextEdit, QTabWidget,
        QMenuBar, QMenu, QFileDialog, QDialog)
from PyQt5.QtCore import QTimer, Qt, pyqtSignal as Signal, QObject
# from PySide6.QtCore import QTimer, Qt,Signal, QObject

import ctypes 

import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import threading
import wifi_module
import time
from shared_state import run_main_flag_event
from loguru import logger
import io
import sys
from contextlib import redirect_stdout
import os
import json
from pathlib import Path
from ffb_cal_ori import ForceFeedbackAlgorithm
from plt_ffb import (plot_torque_vs_speed, plot_torque_vs_angle, 
                     plot_lateral_effect_vs_speed, plot_friction_vs_steer_rate,
                     plot_total_torque_vs_speed, plot_total_torque_vs_angle,
                      plot_lateral_effect_vs_angular_vel)

# 从共享数据模块导入
from shared_state import torque_data, MAX_DATA_POINTS

ffb = ForceFeedbackAlgorithm()

class LogPage(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("日志配置")
        self.setGeometry(200, 200, 800, 600)
        self.setModal(False)
        self.init_ui()
        
    def init_ui(self):
        layout = QVBoxLayout(self)
        self.log_text_edit = QTextEdit()
        self.log_text_edit.setReadOnly(True)
        
        button_layout = QHBoxLayout()
        self.clear_log_button = QPushButton("清除日志")
        self.clear_log_button.clicked.connect(self.clear_log)
        self.save_log_button = QPushButton("保存日志")
        self.save_log_button.clicked.connect(self.save_log)
        self.close_button = QPushButton("关闭")
        self.close_button.clicked.connect(self.close)
        
        button_layout.addWidget(self.clear_log_button)
        button_layout.addWidget(self.save_log_button)
        button_layout.addWidget(self.close_button)
        button_layout.addStretch()
        
        layout.addLayout(button_layout)
        layout.addWidget(self.log_text_edit)
        
    def append_log(self, message):
        self.log_text_edit.append(message)
        
    def clear_log(self):
        self.log_text_edit.clear()
        
    def save_log(self):
        filename, _ = QFileDialog.getSaveFileName(self, "Save Log", "", "Text Files (*.txt);;All Files (*)")
        if filename:
            try:
                with open(filename, 'w', encoding='utf-8') as f:
                    f.write(self.log_text_edit.toPlainText())
            except Exception as e:
                pass

class FFBPlotPage(QWidget):
    def __init__(self):
        super().__init__()
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout(self)
        self.figure = plt.figure(figsize=(10, 8))
        axes = self.figure.subplots(2, 3)

        plot_torque_vs_speed(axes[0, 0], ffb)
        plot_total_torque_vs_speed(axes[0, 1], ffb)
        plot_friction_vs_steer_rate(axes[0, 2], ffb)
        plot_torque_vs_angle(axes[1, 0], ffb)
        plot_total_torque_vs_angle(axes[1, 1], ffb)
        plot_lateral_effect_vs_angular_vel(axes[1, 2], ffb)

        plt.tight_layout()
        self.canvas = FigureCanvas(self.figure)
        layout.addWidget(self.canvas)

class CanMessagePage(QWidget):
    message_received = Signal(str)
    
    def __init__(self, main_window=None):
        super().__init__()
        self.parent_window = main_window
        self.init_ui()
        self.filter_id = None
        self.message_received.connect(self._append_message_thread_safe)

    def init_ui(self):
        layout = QVBoxLayout()
        self.text_edit = QTextEdit()  
        self.text_edit.setReadOnly(True)
        layout.addWidget(self.text_edit)
        self.setLayout(layout)
        
    def _append_message_thread_safe(self, message: str):
        self.text_edit.append(message)
        self.refresh_display()
        
    def apply_filter(self):
        text = self.filter_input.text().strip()
        if text:
            try:
                self.filter_id = int(text, 16)
            except ValueError:
                self.filter_id = None
        else:
            self.filter_id = None
        self.refresh_display()

    def append_message(self, message: str):
        self.message_received.emit(message)

    def refresh_display(self):
        if not self.parent_window or not hasattr(self.parent_window, 'can_data'):
            logger.error("Main window with can_data not found") 
            return
        
        self.text_edit.clear()
        for message in self.parent_window.can_data:
            if self.filter_id is None or message.id == self.filter_id:
                self.text_edit.append(str(message))

class RealTimePlotWindow(QMainWindow):
    def __init__(self, config=None):
        super().__init__()
        self.log_page = None
        self.can_reader_thread = None
        self.can_reader_stop_flag = False

        self.project_dir = Path(__file__).parent
        self.config_file = self.project_dir / "app_config.json"
        
        saved_config = self.load_config()
        
        default_config = {
            'USE_WIFI': False,
            'USE_REAL_CAN': False,
            'USE_REAL_AC': False,
            'USE_RC': False,
        }
        
        self.config = {**default_config,  **(config or {}),**saved_config,}
        
        self.USE_WIFI = self.config.get('USE_WIFI', False)
        self.USE_REAL_CAN = self.config.get('USE_REAL_CAN', False)
        self.USE_REAL_AC = self.config.get('USE_REAL_AC', False)
        self.USE_RC = self.config.get('USE_RC', False)

        self.is_plot_visible = True
        self.can_data = []

        self.setWindowTitle("Real-time Torque Plot")
        self.setGeometry(100, 100, 1200, 800)

        self.ac_api = None
        self.zcanlib = None
        self.chn_handle = None
        self.is_connected = False

        self._init_ui()
        self._create_menu_bar()
        self.main_thread = None

    def load_config(self):
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
        try:
            self.config_file.parent.mkdir(parents=True, exist_ok=True)
            with open(self.config_file, 'w', encoding='utf-8') as f:
                json.dump(self.config, f, indent=4, ensure_ascii=False)
        except Exception as e:
            logger.error(f"保存配置文件失败: {e}")

    def _create_menu_bar(self):
        menubar = self.menuBar()
        view_menu = menubar.addMenu('设置')
        log_action = view_menu.addAction('日志配置')
        log_action.triggered.connect(self.show_log_page)
        
    def show_log_page(self):
        if self.log_page is None:
            self.log_page = LogPage(self)
            
            def log_sink(message):
                if self.log_page and hasattr(self.log_page, 'log_text_edit'):
                    formatted_message = f"{message.time:YYYY-MM-DD HH:mm:ss} | {message.level.name: <8} | {message.message}"
                    self.log_page.append_log(formatted_message)
                    
            logger.add(log_sink, level="INFO", format="{time:YYYY-MM-DD HH:mm:ss} | {level: <8} | {message}")
        
        self.log_page.show()
        self.log_page.raise_()
        self.log_page.activateWindow()
        
    def set_zcanlib(self, zcanlib):
        self.zcanlib = zcanlib

    def start_can_reader(self):
        self.can_reader_stop_flag = False
        def can_reader():
            while not self.can_reader_stop_flag and self.is_connected:
                if self.zcanlib and self.chn_handle:
                    try:
                        if hasattr(self.zcanlib, 'ReceiveFD') and hasattr(self.zcanlib, 'GetReceiveNum'):
                            rcv_num = self.zcanlib.GetReceiveNum(self.chn_handle, 1)
                            if rcv_num > 0:
                                rcv_canfd_msgs, actual_num = self.zcanlib.ReceiveFD(self.chn_handle, rcv_num, 1000)
                                for i in range(actual_num):
                                    frame = rcv_canfd_msgs[i].frame
                                    self.can_data.append(frame)
                                    if hasattr(self, 'can_message_page') and self.can_message_page:
                                        self.can_message_page.append_message(str(frame))
                    except Exception as e:
                        logger.debug(f"CAN reading error: {e}")
                        pass
                time.sleep(0.01)

        self.can_reader_thread = threading.Thread(target=can_reader, daemon=True)
        self.can_reader_thread.start()

    def reset_all_states(self):
        self.can_reader_stop_flag = True

        wifi_module.wifi_flag_event.clear()
        run_main_flag_event.clear()
        wifi_module.CONNECT_EVENT.clear()
        
        if self.can_reader_thread and self.can_reader_thread.is_alive():
            self.can_reader_thread.join(timeout=1.0)

        for key in torque_data:
            torque_data[key].clear()

        self.line1.set_data([], [])
        
        for bar in self.bars:
            bar.set_height(0)
        for text in self.text_objects:
            text.set_text("")
        self.canvas.draw()

        if hasattr(self, 'zcanlib') and self.zcanlib is not None:
            try:
                if hasattr(self.zcanlib, 'chn_handle'):
                    self.zcanlib.ResetCAN(self.zcanlib.chn_handle)
                if hasattr(self.zcanlib, 'device_handle'):
                    self.zcanlib.CloseDevice(self.zcanlib.device_handle)
                logger.info("成功重置并关闭 CAN 设备")
            except Exception as e:
                logger.info(f"无法重置或关闭 CAN 设备: {e}")
            finally:
                self.zcanlib = None

    def toggle_connection(self):
        self.is_connected = not self.is_connected

        if self.is_connected:
            self.conn_status_label.setText("Communication: 🟢 Connected")
            self.toggle_conn_button.setText("🔴 Disconnect")
            self.config['USE_WIFI'] = self.wifi_on_radio.isChecked()
            self.config['USE_REAL_CAN'] = self.can_on_radio.isChecked()
            self.config['USE_REAL_AC'] = self.ac_on_radio.isChecked()
            self.config['USE_RC'] = self.rc_on_radio.isChecked()

            self.save_config()
            
            # 启动主进程
            from DCH_VR_0630 import start_main_process
            if not hasattr(self, 'main_thread') or self.main_thread is None or not self.main_thread.is_alive():
                self.main_thread = threading.Thread(target=start_main_process, args=(self.config,self))
                self.main_thread.daemon = True
                self.main_thread.start()

            run_main_flag_event.set()
            wifi_module.wifi_flag_event.set()
            
            if self.config['USE_WIFI']:
                pass
            else:
                self.start_can_reader()
        else:
            self.conn_status_label.setText("Communication: ⚪ Disconnected")
            self.toggle_conn_button.setText("🟢 Connect")
            self.reset_all_states()

    def reset_plot_data(self):
        for key in torque_data:
            torque_data[key].clear()
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
        layout = QVBoxLayout(main_widget)

        control_layout = QHBoxLayout()

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

        rc_group_box = QGroupBox("RC")
        rc_layout = QHBoxLayout()
        self.rc_on_radio = QRadioButton("ON")
        self.rc_off_radio = QRadioButton("OFF")
        self.rc_group = QButtonGroup()
        self.rc_group.addButton(self.rc_on_radio)
        self.rc_group.addButton(self.rc_off_radio)
        self.rc_on_radio.setChecked(self.USE_RC)
        self.rc_off_radio.setChecked(not self.USE_RC)
        rc_layout.addWidget(self.rc_on_radio)
        rc_layout.addWidget(self.rc_off_radio)
        rc_group_box.setLayout(rc_layout)

        self.conn_status_label = QLabel("Communication: ⚪ Disconnected")
        self.conn_status_label.setStyleSheet("font-size: 14px;")

        self.toggle_conn_button = QPushButton("🟢 Connect")
        self.toggle_conn_button.setCheckable(True)
        self.toggle_conn_button.setChecked(False)
        self.toggle_conn_button.clicked.connect(self.toggle_connection)

        control_layout.addWidget(wifi_group_box)
        control_layout.addWidget(can_group_box)
        control_layout.addWidget(ac_group_box)
        control_layout.addWidget(rc_group_box)
        control_layout.addWidget(self.conn_status_label)
        control_layout.addWidget(self.toggle_conn_button)

        control_layout.addStretch()
        layout.addLayout(control_layout)

        self.tabs = QTabWidget()

        plot_widget = QWidget()
        plot_layout = QVBoxLayout(plot_widget)

        self.figure = Figure()
        self.ax1 = self.figure.add_subplot(211)
        self.ax2 = self.figure.add_subplot(212)
        self.canvas = FigureCanvas(self.figure)
        plot_layout.addWidget(self.canvas)

        self.tabs.addTab(plot_widget, "Torque Plot")

        self.ax1.set_xlim(-540, 540)
        self.ax1.set_ylim(-5000, 5000)
        self.ax1.set_xlabel('Steering Angle (degrees)')
        self.ax1.set_ylabel('Torque')
        self.line1, = self.ax1.plot([], [], 'b-', label='Total Torque')
        self.ax1.legend()
        self.ax1.grid(True)
        
        self.ax2.set_ylim(-5000, 5000)
        self.ax2.set_ylabel('Values')
        self.ax2.grid(True, axis='y')

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(500)

        self.line1, = self.ax1.plot([], [], 'b-', label='Total Torque')
        self.ax1.set_xlabel('Steering Angle (degrees)')
        self.ax1.set_ylabel('Torque')
        self.ax1.legend()
        self.ax1.grid(True)

        bar_labels = []
        self.text_objects = []

        self.can_message_page = CanMessagePage(self)
        self.tabs.addTab(self.can_message_page, "CAN Messages")

        layout.addWidget(self.tabs)

        ffb_plot_page = FFBPlotPage()
        self.tabs.addTab(ffb_plot_page, "FFB Analysis")
        self.tabs.currentChanged.connect(self.on_tab_changed)

    def on_tab_changed(self, index):
        current_tab = self.tabs.tabText(index)
        self.is_plot_visible = (current_tab == "Torque Plot")

    def update_plot(self):
        if not getattr(self, 'is_plot_visible', True):
            return

        steering_angles = torque_data.get('steering_angle', [])
        total_torques = torque_data.get('total_torque', [])

        if len(steering_angles) > 0 and len(total_torques) > 0:
            min_length = min(len(steering_angles), len(total_torques))
            if min_length > 0:
                x_data = steering_angles[-min_length:]
                total_data = total_torques[-min_length:]

            self.line1.set_data(x_data, total_data)
            self.ax1.set_xlim(-540, 540)
            self.ax1.set_ylim(-5000, 5000)        

        def format_label(key):
            if len(key) > 10 and '_' in key:
                truncated_key = key.split('_')[0]
                return truncated_key.capitalize()
            else:
                return key.capitalize()

        display_fields = [key for key in torque_data.keys() 
                        if len(torque_data[key]) > 0]
        
        if not display_fields:
            return
        
        # if not hasattr(self, 'bars') or len(display_fields) != len(self.bars):
        #     self.ax2.clear()

        need_recreate_bars = (not hasattr(self, 'bars') or 
                    len(display_fields) != len(self.bars) or
                    not hasattr(self, 'bar_labels') or
                    display_fields != getattr(self, 'current_display_fields', []))

        if need_recreate_bars:
            self.ax2.clear()
            
            colors = ['blue', 'green', 'orange', 'purple', 'red', 'brown', 'cyan', 'yellow', 'pink', 'gray']
            bar_colors = [colors[i % len(colors)] for i in range(len(display_fields))]
            
            bar_labels = [format_label(field) for field in display_fields]
            
            self.bars = self.ax2.bar(bar_labels, [0]*len(display_fields), color=bar_colors)
            self.text_objects = [self.ax2.text(0, 0, "", ha='center', va='bottom') for _ in self.bars]
            
            self.ax2.set_ylabel('Values')
            self.ax2.grid(True, axis='y')
        
        values = []
        for field in display_fields:
            if torque_data[field]:
                data_points = torque_data[field][-100:] if len(torque_data[field]) >= 100 else torque_data[field]
                values.append(np.mean(data_points))
            else:
                values.append(0)
        
        for i, (bar, val) in enumerate(zip(self.bars, values)):
            bar.set_height(val)
            offset = abs(val) * 0.02 + 1
            text_y_pos = val + offset if val >= 0 else val - offset
            self.text_objects[i].set_position((bar.get_x() + bar.get_width()/2., text_y_pos))
            self.text_objects[i].set_text(f'{val:.3f}')

        self.ax2.set_ylim(-5000, 5000)
        self.canvas.draw()