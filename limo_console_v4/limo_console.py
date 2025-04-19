#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LIMO控制台主程序
集成状态监控和功能模块控制
"""

import sys
import os
import time
import threading
import subprocess
from datetime import datetime
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
    QLabel, QProgressBar, QTextEdit, QGroupBox, QRadioButton, 
    QPushButton, QTabWidget, QStackedWidget, QToolBar, QStatusBar,
    QSplitter, QFrame, QButtonGroup, QGridLayout, QScrollArea,
    QSizePolicy, QSpacerItem, QComboBox
)
from PyQt5.QtCore import Qt, QProcess, QTimer, pyqtSignal, QSize
from PyQt5.QtGui import QIcon, QPixmap, QFont, QColor, QPalette, QTextCursor, QTextCharFormat

# 导入自定义模块
from status_monitor import StatusMonitor
from script_executor import ScriptExecutor

class MatplotlibCanvas(FigureCanvas):
    """Matplotlib画布类，用于在Qt界面中嵌入matplotlib图表"""
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = self.fig.add_subplot(111)
        super(MatplotlibCanvas, self).__init__(self.fig)
        self.setParent(parent)
        self.fig.tight_layout()

class ModuleButton(QGroupBox):
    """模块按钮类，用于创建统一样式的功能模块按钮"""
    def __init__(self, title, options=None, parent=None):
        super(ModuleButton, self).__init__(title, parent)
        
        # 设置大小策略，使其能够在网格布局中自适应大小
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        
        # 设置样式
        self.setStyleSheet("""
            QGroupBox {
                border: 2px solid #3498db;
                border-radius: 8px;
                margin-top: 15px;
                font-weight: bold;
                background-color: #ecf0f1;
                min-height: 120px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top center;
                padding: 0 8px;
                background-color: #3498db;
                color: white;
                border-radius: 4px;
                font-size: 14px;
            }
        """)
        
        # 创建布局
        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(15, 25, 15, 15)
        self.layout.setSpacing(3)
        
        # 添加选项（如果有）
        self.option_combos = {}
        if options:
            for option in options:
                option_layout = QHBoxLayout()
                option_label = QLabel(option["name"] + ":")
                option_label.setStyleSheet("font-weight: normal;")
                option_combo = QComboBox()
                option_combo.addItems(option["values"])
                option_combo.setStyleSheet("""
                    QComboBox {
                        padding: 5px;
                        border: 1px solid #bdc3c7;
                        border-radius: 4px;
                        background-color: rgba(255, 255, 255, 0.1);
                        color: rgba(0, 0, 0, 0.8);  /* 设置文字颜色和透明度 */
                    }
                    QComboBox::drop-down {
                        border: none;
                        width: 20px;
                    }
                """)
                self.option_combos[option["name"]] = {
                    "combo": option_combo,
                    "param_values": option["param_values"]
                }
                option_layout.addWidget(option_label)
                option_layout.addWidget(option_combo)
                self.layout.addLayout(option_layout)
        
        # 添加按钮
        self.buttons_layout = QHBoxLayout()
        
        # 开始按钮
        self.start_button = QPushButton("开始")
        self.start_button.setStyleSheet("""
            QPushButton {
                background-color: #2ecc71;
                color: white;
                border-radius: 4px;
                padding: 8px;
                font-weight: bold;
                font-size: 13px;
            }
            QPushButton:hover {
                background-color: #27ae60;
            }
        """)
        self.buttons_layout.addWidget(self.start_button)
        
        # 结束按钮
        self.stop_button = QPushButton("结束")
        self.stop_button.setStyleSheet("""
            QPushButton {
                background-color: #e74c3c;
                color: white;
                border-radius: 4px;
                padding: 8px;
                font-weight: bold;
                font-size: 13px;
            }
            QPushButton:hover {
                background-color: #c0392b;
            }
        """)
        self.buttons_layout.addWidget(self.stop_button)
        
        self.layout.addLayout(self.buttons_layout)
    
    def get_options(self):
        """获取当前选项值"""
        options = {}
        for name, option_data in self.option_combos.items():
            combo = option_data["combo"]
            param_values = option_data["param_values"]
            selected_index = combo.currentIndex()
            options[name] = param_values[selected_index]
        return options

class LimoConsole(QMainWindow):
    """LIMO控制台主窗口类"""
    def __init__(self):
        super(LimoConsole, self).__init__()
        
        # 设置窗口属性
        self.setWindowTitle("LIMO控制台")
        self.setMinimumSize(1200, 800)
        
        # 初始化状态监控器
        self.status_monitor = StatusMonitor(self)
        self.status_monitor.status_updated.connect(self.update_status_display)
        
        # 初始化脚本执行器
        self.script_executor = ScriptExecutor(self)
        self.script_executor.output_received.connect(self.append_output)
        self.script_executor.error_received.connect(self.append_error)
        
        # 创建界面
        self._create_ui()
        
        # 创建定时器，用于更新图表
        self.chart_timer = QTimer(self)
        self.chart_timer.timeout.connect(self.update_charts)
        self.chart_timer.start(1000)  # 每秒更新一次图表
        
        # 创建定时器，用于检测roscore状态
        self.roscore_timer = QTimer(self)
        self.roscore_timer.timeout.connect(self.check_roscore_status)
        self.roscore_timer.start(5000)  # 每5秒检测一次roscore状态
        
        # 显示状态栏消息
        self.statusBar().showMessage("LIMO控制台已启动")
    
    def _create_ui(self):
        """创建用户界面"""
        # 创建中央部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 创建主布局
        main_layout = QVBoxLayout(central_widget)
        
        # 创建工具栏
        self._create_toolbar()
        
        # 创建主分割器
        main_splitter = QSplitter(Qt.Horizontal)
        main_layout.addWidget(main_splitter)
        
        # 创建左侧功能选择区
        left_widget = self._create_left_panel()
        main_splitter.addWidget(left_widget)
        
        # 创建右侧内容区
        right_splitter = QSplitter(Qt.Vertical)
        main_splitter.addWidget(right_splitter)
        
        # 创建右上状态显示区
        right_top_widget = self._create_status_panel()
        right_splitter.addWidget(right_top_widget)
        
        # 创建右下命令行状态区
        right_bottom_widget = self._create_command_panel()
        right_splitter.addWidget(right_bottom_widget)
        
        # 设置分割器比例
        main_splitter.setSizes([600, 600])
        right_splitter.setSizes([600, 400])
        
        # 创建状态栏
        self.statusBar()
    
    def _create_toolbar(self):
        """创建工具栏"""
        toolbar = QToolBar("主工具栏")
        toolbar.setMovable(False)
        toolbar.setIconSize(QSize(32, 32))
        self.addToolBar(toolbar)
        
        # 添加标题标签
        title_label = QLabel("                                                                                                    LIMO控制台")
        title_label.setFont(QFont("Arial", 16, QFont.Bold))
        toolbar.addWidget(title_label)
        
        # 添加伸缩器
        spacer = QWidget()
        spacer.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        toolbar.addWidget(spacer)
        
        # 添加公司图标（占位，实际使用时替换为真实图标）
        logo_label = QLabel()
        logo_pixmap = QPixmap("111.png")  # 替换为实际图片的路径
        if not logo_pixmap.isNull():  # 检查图片是否成功加载
            logo_label.setPixmap(logo_pixmap.scaled(505/3, 164/3, Qt.KeepAspectRatio))  # 调整图片大小并保持纵横比
        else:
            print("Error: Logo image not found!")  # 如果图片未找到，打印错误信息
        toolbar.addWidget(logo_label)
    
    def _create_left_panel(self):
        """创建左侧功能选择面板"""
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        
        # 创建滚动区域
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll_area.setStyleSheet("border: none;")
        
        # 创建滚动区域内容
        scroll_content = QWidget()
        # 使用网格布局替代垂直布局，实现一行两个模块
        scroll_layout = QGridLayout(scroll_content)
        scroll_layout.setSpacing(20)
        
        # 定义功能模块
        self.modules = [
            {
                "name": "Gmapping建图", 
                "module_id": 1,
                "options": None
            },
            {
                "name": "Cartographer建图", 
                "module_id": 2,
                "options": None
            },
            {
                "name": "检查雷达数据", 
                "module_id": 3,
                "options": None
            },
            {
                "name": "雷达导航", 
                "module_id": 4,
                "options": [
                    {"name": "底盘模式", "values": ["差速", "阿克曼", "麦轮"], "param_values": ["diff", "ackerman", "mec"]}
                ]
            },
            {
                "name": "控制方式", 
                "module_id": 5,
                "options": [
                    {"name": "控制类型", "values": ["键盘", "鼠标"], "param_values": ["keyboard", "mouse"]}
                ]
            },
            {
                "name": "集成控制", 
                "module_id": 6,
                "options": None
            },
            {
                "name": "基础节点", 
                "module_id": 7,
                "options": None
            },
            {
                "name": "路径巡检", 
                "module_id": 8,
                "options": [
                    {"name": "底盘模式", "values": ["差速", "阿克曼"], "param_values": ["diff", "ackerman"]}
                ]
            },
            {
                "name": "摄像头控制", 
                "module_id": 9,
                "options": [
                    {"name": "相机类型", "values": ["大白相机", "Realsense相机"], "param_values": ["dabai", "realsense"]}
                ]
            },
            {
                "name": "摄像头建图", 
                "module_id": 10,
                "options": [
                    {"name": "相机类型", "values": ["大白相机", "Realsense相机"], "param_values": ["dabai", "realsense"]}
                ]
            },
            {
                "name": "摄像头导航", 
                "module_id": 11,
                "options": [
                    {"name": "相机类型", "values": ["大白相机", "Realsense相机"], "param_values": ["dabai", "realsense"]},
                    {"name": "底盘模式", "values": ["差速", "阿克曼"], "param_values": ["diff", "ackerman"]}
                ]
            },
            {
                "name": "文字识别", 
                "module_id": 12,
                "options": None
            },
            {
                "name": "红绿灯识别", 
                "module_id": 13,
                "options": [
                    {"name": "相机类型", "values": ["大白相机", "Realsense相机"], "param_values": ["dabai", "realsense"]}
                ]
            },
            {
                "name": "语音转文字", 
                "module_id": 14,
                "options": None
            },
            {
                "name": "语音控制", 
                "module_id": 15,
                "options": None
            },
            {
                "name": "RViz", 
                "module_id": 16,
                "options": None
            }
        ]
        
        # 创建模块按钮，使用网格布局，一行两个模块
        self.module_buttons = []
        for i, module in enumerate(self.modules):
            module_button = ModuleButton(module["name"], module["options"])
            module_button.start_button.clicked.connect(
                lambda checked, m=module: self.start_module(m)
            )
            module_button.stop_button.clicked.connect(
                lambda checked, m=module: self.stop_module(m)
            )
            
            # 计算行和列位置
            row = i // 2  # 每行两个模块
            col = i % 2   # 列索引为0或1
            
            scroll_layout.addWidget(module_button, row, col)
            self.module_buttons.append(module_button)
        
        # 设置滚动区域内容
        scroll_area.setWidget(scroll_content)
        
        # 添加到左侧布局
        left_layout.addWidget(scroll_area)
        
        return left_widget
    
    def _create_status_panel(self):
        """创建右上状态显示面板"""
        status_widget = QWidget()
        status_layout = QVBoxLayout(status_widget)
        
        # 创建状态显示组
        status_group = QGroupBox("LIMO状态")
        status_group.setStyleSheet("""
            QGroupBox {
                border: 2px solid #f39c12;
                border-radius: 8px;
                margin-top: 10px;
                font-weight: bold;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top center;
                padding: 0 5px;
                background-color: #f39c12;
                color: white;
                border-radius: 3px;
            }
        """)
        
        status_inner_layout = QVBoxLayout(status_group)
        
        # 创建roscore状态指示器
        roscore_layout = QHBoxLayout()
        roscore_label = QLabel("ROS核心状态:")
        self.roscore_status_label = QLabel("未运行")
        self.roscore_status_label.setStyleSheet("color: red; font-weight: bold;")
        roscore_layout.addWidget(roscore_label)
        roscore_layout.addWidget(self.roscore_status_label)
        roscore_layout.addStretch()
        status_inner_layout.addLayout(roscore_layout)
        
        # 创建数据源指示器
        data_source_layout = QHBoxLayout()
        data_source_label = QLabel("数据来源:")
        self.data_source_label = QLabel("pylimo库")
        self.data_source_label.setStyleSheet("color: blue; font-weight: bold;")
        data_source_layout.addWidget(data_source_label)
        data_source_layout.addWidget(self.data_source_label)
        data_source_layout.addStretch()
        status_inner_layout.addLayout(data_source_layout)
        
        # 创建状态数据网格
        status_grid = QGridLayout()
        status_grid.setSpacing(10)
        
        # 添加状态项
        status_items = [
            ("电池电压:", "battery_voltage", "V"),
            ("运动模式:", "motion_mode", ""),
            ("控制模式:", "control_mode", ""),
            ("线速度:", "linear_velocity", "m/s"),
            ("角速度:", "angular_velocity", "rad/s"),
            ("横向速度:", "lateral_velocity", "m/s"),
            ("转向角度:", "steering_angle", "°"),
            ("错误代码:", "error_code", "")
        ]
        
        self.status_labels = {}
        for i, (label_text, key, unit) in enumerate(status_items):
            row = i // 2
            col = (i % 2) * 3
            
            label = QLabel(label_text)
            value_label = QLabel("--")
            unit_label = QLabel(unit)
            
            self.status_labels[key] = value_label
            
            status_grid.addWidget(label, row, col)
            status_grid.addWidget(value_label, row, col + 1)
            status_grid.addWidget(unit_label, row, col + 2)
        
        status_inner_layout.addLayout(status_grid)
        
        # 创建图表区域
        charts_layout = QHBoxLayout()
        
        # 电池电压图表
        battery_group = QGroupBox("电池电压")
        battery_layout = QVBoxLayout(battery_group)
        self.battery_canvas = MatplotlibCanvas(width=4, height=2, dpi=100)
        battery_layout.addWidget(self.battery_canvas)
        charts_layout.addWidget(battery_group)
        
        # 速度图表
        velocity_group = QGroupBox("速度")
        velocity_layout = QVBoxLayout(velocity_group)
        self.velocity_canvas = MatplotlibCanvas(width=4, height=2, dpi=100)
        velocity_layout.addWidget(self.velocity_canvas)
        charts_layout.addWidget(velocity_group)
        
        status_inner_layout.addLayout(charts_layout)
        
        # 添加到状态布局
        status_layout.addWidget(status_group)
        
        return status_widget
    
    def _create_command_panel(self):
        """创建右下命令行状态面板"""
        command_widget = QWidget()
        command_layout = QVBoxLayout(command_widget)
        
        # 创建命令输出组
        command_group = QGroupBox("命令输出")
        command_group.setStyleSheet("""
            QGroupBox {
                border: 2px solid #3498db;
                border-radius: 8px;
                margin-top: 10px;
                font-weight: bold;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top center;
                padding: 0 5px;
                background-color: #3498db;
                color: white;
                border-radius: 3px;
            }
        """)
        
        command_inner_layout = QVBoxLayout(command_group)
        
        # 创建输出文本框
        self.output_text = QTextEdit()
        self.output_text.setReadOnly(True)
        self.output_text.setStyleSheet("""
            QTextEdit {
                background-color: #1e272e;
                color: #ecf0f1;
                border-radius: 4px;
                font-family: Consolas, Monaco, monospace;
                font-size: 13px;
                padding: 10px;
            }
        """)
        command_inner_layout.addWidget(self.output_text)
        
        # 创建按钮布局
        buttons_layout = QHBoxLayout()
        
        # 清空按钮
        clear_button = QPushButton("清空输出")
        clear_button.clicked.connect(self.clear_output)
        clear_button.setStyleSheet("""
            QPushButton {
                background-color: #7f8c8d;
                color: white;
                border-radius: 4px;
                padding: 8px;
                font-weight: bold;
                font-size: 13px;
            }
            QPushButton:hover {
                background-color: #95a5a6;
            }
        """)
        buttons_layout.addWidget(clear_button)
        
        # 复位按钮
        reset_button = QPushButton("复位所有")
        reset_button.clicked.connect(self.reset_all)
        reset_button.setStyleSheet("""
            QPushButton {
                background-color: #e74c3c;
                color: white;
                border-radius: 4px;
                padding: 8px;
                font-weight: bold;
                font-size: 13px;
            }
            QPushButton:hover {
                background-color: #c0392b;
            }
        """)
        buttons_layout.addWidget(reset_button)
        
        command_inner_layout.addLayout(buttons_layout)
        
        # 添加到命令布局
        command_layout.addWidget(command_group)
        
        return command_widget
    
    def start_module(self, module):
        """启动指定的功能模块"""
        module_id = module["module_id"]
        module_name = module["name"]
        
        # 获取选项参数
        params = []
        if module["options"]:
            button_index = self.modules.index(module)
            module_button = self.module_buttons[button_index]
            options = module_button.get_options()
            params = list(options.values())
        
        # 启动模块
        self.script_executor.execute_script(module_id, "start", *params)
        
        # 更新状态栏
        self.statusBar().showMessage(f"正在运行: {module_name}")
    
    def stop_module(self, module):
        """停止指定的功能模块"""
        module_id = module["module_id"]
        module_name = module["name"]
        
        # 停止模块
        self.script_executor.execute_script(module_id, "stop")
        
        # 更新状态栏
        self.statusBar().showMessage(f"已停止: {module_name}")
    
    def update_status_display(self):
        """更新状态显示"""
        # 更新状态标签
        self.status_labels["battery_voltage"].setText(f"{self.status_monitor.battery_voltage:.2f}")
        self.status_labels["motion_mode"].setText(self.status_monitor.motion_mode_dict.get(self.status_monitor.motion_mode, "未知"))
        self.status_labels["control_mode"].setText(self.status_monitor.control_mode_dict.get(self.status_monitor.control_mode, "未知"))
        self.status_labels["linear_velocity"].setText(f"{self.status_monitor.linear_velocity:.2f}")
        self.status_labels["angular_velocity"].setText(f"{self.status_monitor.angular_velocity:.2f}")
        self.status_labels["lateral_velocity"].setText(f"{self.status_monitor.lateral_velocity:.2f}")
        self.status_labels["steering_angle"].setText(f"{self.status_monitor.steering_angle:.2f}")
        
        # 错误代码处理
        error_text = "无"
        if self.status_monitor.error_code > 0:
            error_codes = []
            for code, desc in self.status_monitor.error_code_dict.items():
                if self.status_monitor.error_code & code:
                    error_codes.append(desc)
            if error_codes:
                error_text = ", ".join(error_codes)
        self.status_labels["error_code"].setText(error_text)
    
    def update_charts(self):
        """更新图表"""
        # 获取数据锁
        with self.status_monitor.data_lock:
            timestamps = self.status_monitor.timestamps.copy()
            battery_history = self.status_monitor.battery_history.copy()
            velocity_history = self.status_monitor.velocity_history.copy()
        
        # 如果没有数据，则不更新
        if not timestamps:
            return
        
        # 转换时间戳为相对时间（秒）
        relative_times = [t - timestamps[0] for t in timestamps]
        
        # 更新电池电压图表
        self.battery_canvas.axes.clear()
        self.battery_canvas.axes.plot(relative_times, battery_history, 'b-')
        self.battery_canvas.axes.set_ylabel('电压 (V)')
        self.battery_canvas.axes.set_xlabel('时间 (s)')
        self.battery_canvas.axes.grid(True)
        self.battery_canvas.draw()
        
        # 更新速度图表
        self.velocity_canvas.axes.clear()
        self.velocity_canvas.axes.plot(relative_times, velocity_history, 'g-')
        self.velocity_canvas.axes.set_ylabel('线速度 (m/s)')
        self.velocity_canvas.axes.set_xlabel('时间 (s)')
        self.velocity_canvas.axes.grid(True)
        self.velocity_canvas.draw()
    
    def check_roscore_status(self):
        """检查roscore状态"""
        try:
            # 使用rostopic list命令检查roscore是否运行
            result = subprocess.run(
                ["rostopic", "list"], 
                stdout=subprocess.PIPE, 
                stderr=subprocess.PIPE, 
                timeout=1
            )
            
            # 根据返回码判断roscore状态
            if result.returncode == 0:
                # roscore正在运行
                self.roscore_status_label.setText("运行中")
                self.roscore_status_label.setStyleSheet("color: green; font-weight: bold;")
                
                # 如果之前不是ROS话题订阅模式，则切换
                if self.data_source_label.text() != "ROS话题":
                    self.switch_to_ros_topics()
            else:
                # roscore未运行
                self.roscore_status_label.setText("未运行")
                self.roscore_status_label.setStyleSheet("color: red; font-weight: bold;")
                
                # 如果之前不是pylimo库模式，则切换
                if self.data_source_label.text() != "pylimo库":
                    self.switch_to_pylimo()
        except Exception as e:
            # 发生异常，认为roscore未运行
            self.roscore_status_label.setText("未运行")
            self.roscore_status_label.setStyleSheet("color: red; font-weight: bold;")
            
            # 如果之前不是pylimo库模式，则切换
            if self.data_source_label.text() != "pylimo库":
                self.switch_to_pylimo()
    
    def switch_to_ros_topics(self):
        """切换到ROS话题订阅模式"""
        self.append_output("检测到ROS核心运行中，切换到ROS话题订阅模式")
        self.data_source_label.setText("ROS话题")
        self.data_source_label.setStyleSheet("color: green; font-weight: bold;")
        
        # 停止pylimo库数据读取
        self.status_monitor.stop_pylimo()
        
        # 启动ROS话题订阅
        self.status_monitor.start_ros_subscriber()
    
    def switch_to_pylimo(self):
        """切换到pylimo库模式"""
        self.append_output("未检测到ROS核心，切换到pylimo库读取模式")
        self.data_source_label.setText("pylimo库")
        self.data_source_label.setStyleSheet("color: blue; font-weight: bold;")
        
        # 停止ROS话题订阅
        self.status_monitor.stop_ros_subscriber()
        
        # 启动pylimo库数据读取
        self.status_monitor.start_pylimo()
    
    def append_output(self, text):
        """添加输出文本"""
        cursor = self.output_text.textCursor()
        format = QTextCharFormat()
        format.setForeground(QColor("#54a0ff"))  # 蓝色
        cursor.movePosition(QTextCursor.End)
        cursor.insertText(text + "\n", format)
        self.output_text.setTextCursor(cursor)
        self.output_text.ensureCursorVisible()
    
    def append_error(self, text):
        """添加错误文本"""
        cursor = self.output_text.textCursor()
        format = QTextCharFormat()
        format.setForeground(QColor("#ff6b6b"))  # 红色
        cursor.movePosition(QTextCursor.End)
        cursor.insertText("错误: " + text + "\n", format)
        self.output_text.setTextCursor(cursor)
        self.output_text.ensureCursorVisible()
    
    def clear_output(self):
        """清空输出区域"""
        self.output_text.clear()
    
    def reset_all(self):
        """复位所有功能"""
        # 终止所有进程
        self.script_executor.terminate_all()
        
        # 清空输出
        self.clear_output()
        
        # 更新状态栏
        self.statusBar().showMessage("已复位所有功能")
        
        # 添加输出信息
        self.append_output("已复位所有功能")

    # 重写调整大小事件，确保窗口自适应大小
    def resizeEvent(self, event):
        super().resizeEvent(event)
        # 可以在这里添加额外的自适应大小逻辑
        # 例如，根据窗口大小调整某些组件的大小

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = LimoConsole()
    window.show()
    sys.exit(app.exec_())