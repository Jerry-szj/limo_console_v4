#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
状态监控模块
用于监控LIMO小车的状态信息
支持两种数据读取方式：
1. pylimo库直接读取（当roscore未运行时）
2. ROS话题订阅（当roscore运行时）
"""

import os
import sys
import time
import threading
import csv
import subprocess
from datetime import datetime
import numpy as np
from PyQt5.QtCore import QObject, pyqtSignal

# 导入pylimo库
sys.path.append('/home/ubuntu/upload')
try:
    from pylimo.limo import LIMO
    import pylimo.limomsg as limomsg
    LIMO_AVAILABLE = True
except ImportError:
    print("警告: pylimo库未找到，直接读取功能将不可用")
    LIMO_AVAILABLE = False

# 导入ROS相关库
try:
    import rospy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Float32, Int32, String
    ROS_AVAILABLE = True
except ImportError:
    print("警告: ROS库未找到，话题订阅功能将不可用")
    ROS_AVAILABLE = False

class StatusMonitor(QObject):
    """LIMO状态监控类，用于获取和显示LIMO状态信息"""
    # 定义信号
    status_updated = pyqtSignal()
    
    def __init__(self, parent=None, log_data=True):
        """初始化状态监控器
        
        Args:
            parent: 父对象
            log_data: 是否记录数据到CSV文件
        """
        super(StatusMonitor, self).__init__(parent)
        
        # 状态数据
        self.vehicle_state = 0
        self.control_mode = 0
        self.battery_voltage = 0.0
        self.error_code = 0
        self.motion_mode = 0
        
        # 运动状态数据
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.lateral_velocity = 0.0
        self.steering_angle = 0.0
        
        # 数据记录
        self.log_data = log_data
        self.csv_file = None
        self.csv_writer = None
        self.data_lock = threading.Lock()
        
        # 历史数据
        self.timestamps = []
        self.battery_history = []
        self.velocity_history = []
        self.error_history = []
        
        # 状态解析字典
        self.vehicle_state_dict = {
            0: "正常",
            1: "紧急停止",
            2: "故障"
        }
        
        self.control_mode_dict = {
            0: "遥控模式",
            1: "命令模式",
            2: "未知模式"
        }
        
        self.motion_mode_dict = {
            0: "差速模式",
            1: "阿克曼模式",
            2: "麦克纳姆轮模式",
            3: "未知模式"
        }
        
        self.error_code_dict = {
            0x0001: "电池电量低",
            0x0002: "电池电量低",
            0x0004: "遥控器连接丢失",
            0x0008: "电机驱动器1错误",
            0x0010: "电机驱动器2错误",
            0x0020: "电机驱动器3错误",
            0x0040: "电机驱动器4错误",
            0x0100: "驱动状态错误"
        }
        
        # pylimo相关
        self.limo = None
        self.limo_connected = False
        self.pylimo_running = False
        self.pylimo_thread = None
        
        # ROS相关
        self.ros_initialized = False
        self.ros_running = False
        self.ros_thread = None
        self.ros_subscribers = []
        
        # 初始化数据记录
        if self.log_data:
            self._init_data_logging()
        
        # 默认启动pylimo库数据读取
        self.start_pylimo()
    
    def _init_data_logging(self):
        """初始化数据记录"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_dir = "status_logs"
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        
        filename = f"{log_dir}/status_{timestamp}.csv"
        self.csv_file = open(filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'timestamp', 'vehicle_state', 'control_mode', 'battery_voltage', 
            'error_code', 'motion_mode', 'linear_velocity', 'angular_velocity',
            'lateral_velocity', 'steering_angle'
        ])
        print(f"数据记录已启动，文件：{filename}")
    
    def start_pylimo(self):
        """启动pylimo库数据读取"""
        if not LIMO_AVAILABLE:
            print("警告: pylimo库未找到，无法启动直接读取")
            return False
        
        if self.pylimo_running:
            print("pylimo库数据读取已经在运行")
            return True
        
        try:
            # 初始化LIMO设备
            self.limo = LIMO()
            self.limo.EnableCommand()
            print("命令模式已启用")
            self.limo_connected = True
            
            # 启动数据更新线程
            self.pylimo_running = True
            self.pylimo_thread = threading.Thread(target=self._pylimo_update_loop)
            self.pylimo_thread.daemon = True
            self.pylimo_thread.start()
            
            print("pylimo库数据读取已启动")
            return True
        except Exception as e:
            print(f"启动pylimo库数据读取失败: {e}")
            self.limo_connected = False
            self.pylimo_running = False
            return False
    
    def stop_pylimo(self):
        """停止pylimo库数据读取"""
        if not self.pylimo_running:
            return
        
        self.pylimo_running = False
        
        if hasattr(self, 'pylimo_thread') and self.pylimo_thread and self.pylimo_thread.is_alive():
            self.pylimo_thread.join(timeout=1.0)
        
        if self.limo_connected and self.limo:
            try:
                self.limo.DisableCommand()
                print("命令模式已禁用")
            except:
                pass
            self.limo_connected = False
        
        print("pylimo库数据读取已停止")
    
    def _pylimo_update_status(self):
        """使用pylimo库更新状态信息"""
        if not self.limo_connected or not self.limo:
            return False
            
        try:
            # 获取车辆状态
            self.vehicle_state = limomsg.GetVehicleState()
            
            # 获取控制模式
            self.control_mode = limomsg.GetControlMode()
            
            # 获取电池电压
            self.battery_voltage = limomsg.GetBatteryVoltage()
            
            # 获取错误代码
            self.error_code = limomsg.GetErrorCode()
            
            # 获取运动模式
            self.motion_mode = limomsg.GetMotionMode()
            
            # 获取运动状态
            self.linear_velocity = limomsg.GetLinearVelocity()
            self.angular_velocity = limomsg.GetAngularVelocity()
            self.lateral_velocity = limomsg.GetLateralVelocity()
            self.steering_angle = limomsg.GetSteeringAngle()
            
            # 记录历史数据
            self._record_history_data()
            
            # 发出信号通知界面更新
            self.status_updated.emit()
            
            return True
        except Exception as e:
            print(f"使用pylimo库更新状态信息时出错: {e}")
            return False
    
    def _pylimo_update_loop(self):
        """pylimo库数据更新循环"""
        while self.pylimo_running:
            if self.limo_connected:
                self._pylimo_update_status()
            time.sleep(0.1)  # 更新频率10Hz
    
    def start_ros_subscriber(self):
        """启动ROS话题订阅"""
        if not ROS_AVAILABLE:
            print("警告: ROS库未找到，无法启动话题订阅")
            return False
        
        if self.ros_running:
            print("ROS话题订阅已经在运行")
            return True
        
        try:
            # 初始化ROS节点
            if not self.ros_initialized:
                rospy.init_node('limo_status_monitor', anonymous=True, disable_signals=True)
                self.ros_initialized = True
            
            # 创建订阅者
            self.ros_subscribers = []
            
            # 订阅车辆状态话题
            self.ros_subscribers.append(
                rospy.Subscriber('/limo_status/vehicle_state', Int32, self._ros_vehicle_state_callback)
            )
            
            # 订阅控制模式话题
            self.ros_subscribers.append(
                rospy.Subscriber('/limo_status/control_mode', Int32, self._ros_control_mode_callback)
            )
            
            # 订阅电池电压话题
            self.ros_subscribers.append(
                rospy.Subscriber('/limo_status/battery_voltage', Float32, self._ros_battery_voltage_callback)
            )
            
            # 订阅错误代码话题
            self.ros_subscribers.append(
                rospy.Subscriber('/limo_status/error_code', Int32, self._ros_error_code_callback)
            )
            
            # 订阅运动模式话题
            self.ros_subscribers.append(
                rospy.Subscriber('/limo_status/motion_mode', Int32, self._ros_motion_mode_callback)
            )
            
            # 订阅速度话题
            self.ros_subscribers.append(
                rospy.Subscriber('/cmd_vel', Twist, self._ros_velocity_callback)
            )
            
            # 订阅转向角度话题
            self.ros_subscribers.append(
                rospy.Subscriber('/limo_status/steering_angle', Float32, self._ros_steering_angle_callback)
            )
            
            # 启动ROS更新线程
            self.ros_running = True
            self.ros_thread = threading.Thread(target=self._ros_update_loop)
            self.ros_thread.daemon = True
            self.ros_thread.start()
            
            print("ROS话题订阅已启动")
            return True
        except Exception as e:
            print(f"启动ROS话题订阅失败: {e}")
            self.ros_running = False
            return False
    
    def stop_ros_subscriber(self):
        """停止ROS话题订阅"""
        if not self.ros_running:
            return
        
        self.ros_running = False
        
        # 取消所有订阅
        for subscriber in self.ros_subscribers:
            try:
                subscriber.unregister()
            except:
                pass
        self.ros_subscribers = []
        
        if hasattr(self, 'ros_thread') and self.ros_thread and self.ros_thread.is_alive():
            self.ros_thread.join(timeout=1.0)
        
        print("ROS话题订阅已停止")
    
    def _ros_vehicle_state_callback(self, msg):
        """ROS车辆状态回调"""
        self.vehicle_state = msg.data
    
    def _ros_control_mode_callback(self, msg):
        """ROS控制模式回调"""
        self.control_mode = msg.data
    
    def _ros_battery_voltage_callback(self, msg):
        """ROS电池电压回调"""
        self.battery_voltage = msg.data
    
    def _ros_error_code_callback(self, msg):
        """ROS错误代码回调"""
        self.error_code = msg.data
    
    def _ros_motion_mode_callback(self, msg):
        """ROS运动模式回调"""
        self.motion_mode = msg.data
    
    def _ros_velocity_callback(self, msg):
        """ROS速度回调"""
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z
        self.lateral_velocity = msg.linear.y
    
    def _ros_steering_angle_callback(self, msg):
        """ROS转向角度回调"""
        self.steering_angle = msg.data
    
    def _ros_update_loop(self):
        """ROS数据更新循环"""
        rate = rospy.Rate(10)  # 10Hz
        while self.ros_running and not rospy.is_shutdown():
            # 记录历史数据
            self._record_history_data()
            
            # 发出信号通知界面更新
            self.status_updated.emit()
            
            try:
                rate.sleep()
            except:
                break
    
    def _record_history_data(self):
        """记录历史数据"""
        with self.data_lock:
            current_time = time.time()
            self.timestamps.append(current_time)
            self.battery_history.append(self.battery_voltage)
            self.velocity_history.append(self.linear_velocity)
            self.error_history.append(self.error_code)
            
            # 限制历史数据长度
            max_history = 100
            if len(self.timestamps) > max_history:
                self.timestamps = self.timestamps[-max_history:]
                self.battery_history = self.battery_history[-max_history:]
                self.velocity_history = self.velocity_history[-max_history:]
                self.error_history = self.error_history[-max_history:]
        
        # 记录数据到CSV文件
        if self.log_data and self.csv_writer:
            self.csv_writer.writerow([
                time.time(), self.vehicle_state, self.control_mode, 
                self.battery_voltage, self.error_code, self.motion_mode,
                self.linear_velocity, self.angular_velocity,
                self.lateral_velocity, self.steering_angle
            ])
    
    def stop(self):
        """停止状态监控器"""
        # 停止pylimo库数据读取
        self.stop_pylimo()
        
        # 停止ROS话题订阅
        self.stop_ros_subscriber()
        
        # 关闭数据记录
        if hasattr(self, 'csv_file') and self.csv_file:
            self.csv_file.close()
            print("数据记录已停止")
        
        print("状态监控器已停止")
