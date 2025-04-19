#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
脚本执行框架
用于从PyQt界面调用功能模块脚本
"""

import os
import sys
import subprocess
import threading
import time
import signal
import re
from PyQt5.QtCore import QObject, pyqtSignal, QProcess, QTimer
from PyQt5.QtGui import QTextCursor, QColor, QTextCharFormat

class ScriptExecutor(QObject):
    """脚本执行器类，用于执行功能模块脚本"""
    
    # 定义信号
    output_received = pyqtSignal(str)
    error_received = pyqtSignal(str)
    execution_finished = pyqtSignal(int, str)
    
    def __init__(self, parent=None):
        """初始化脚本执行器
        
        Args:
            parent: 父对象
        """
        super(ScriptExecutor, self).__init__(parent)
        self.processes = {}  # 存储所有进程
        self.script_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'scripts')
        
        # 确保脚本目录存在
        if not os.path.exists(self.script_dir):
            os.makedirs(self.script_dir)
    
    def execute_script(self, module_id, action, *args):
        """执行脚本
        
        Args:
            module_id: 模块ID（1-16）
            action: 操作（start, stop等）
            args: 附加参数
            
        Returns:
            进程ID
        """
        # 构建脚本路径
        script_path = os.path.join(self.script_dir, f"module{module_id}_{self._get_script_name(module_id)}.py")
        
        # 检查脚本是否存在
        if not os.path.exists(script_path):
            self.error_received.emit(f"错误: 脚本 {script_path} 不存在")
            return None
        
        # 构建命令
        cmd = [sys.executable, script_path, action]
        cmd.extend(args)
        
        # 创建进程
        process = QProcess(self)
        
        # 连接信号
        process.readyReadStandardOutput.connect(
            lambda: self._handle_output(process)
        )
        process.readyReadStandardError.connect(
            lambda: self._handle_error(process)
        )
        process.finished.connect(
            lambda exit_code, exit_status: self._handle_finished(process.processId(), exit_code, exit_status)
        )
        
        # 设置工作目录
        process.setWorkingDirectory(os.path.expanduser("~"))
        
        # 启动进程
        process.start(cmd[0], cmd[1:])
        
        # 存储进程
        process_id = process.processId()
        self.processes[process_id] = process
        
        # 输出启动信息
        self.output_received.emit(f"启动脚本: {' '.join(cmd)}")
        
        return process_id
    
    def _get_script_name(self, module_id):
        """根据模块ID获取脚本名称"""
        script_names = {
            1: "gmapping",
            2: "cartographer",
            3: "check_lidar",
            4: "navigation",
            5: "control",
            6: "integrated_control",
            7: "base_node",
            8: "path_patrol",
            9: "camera",
            10: "camera_mapping",
            11: "camera_navigation",
            12: "text_recognition",
            13: "traffic_light",
            14: "voice_to_text",
            15: "voice_control",
            16: "rviz"
        }
        return script_names.get(module_id, f"unknown_{module_id}")
    
    def _handle_output(self, process):
        """处理标准输出"""
        data = process.readAllStandardOutput().data().decode('utf-8')
        
        # 移除ANSI颜色代码
        clean_data = re.sub(r'\033\[\d+m', '', data)
        
        self.output_received.emit(clean_data)
    
    def _handle_error(self, process):
        """处理标准错误"""
        data = process.readAllStandardError().data().decode('utf-8')
        
        # 移除ANSI颜色代码
        clean_data = re.sub(r'\033\[\d+m', '', data)
        
        self.error_received.emit(clean_data)
    
    def _handle_finished(self, process_id, exit_code, exit_status):
        """处理进程结束"""
        status_text = "正常结束" if exit_status == QProcess.NormalExit else "异常结束"
        self.execution_finished.emit(exit_code, status_text)
        
        # 从进程字典中移除
        if process_id in self.processes:
            del self.processes[process_id]
    
    def terminate_all(self):
        """终止所有进程"""
        for process_id, process in list(self.processes.items()):
            if process.state() != QProcess.NotRunning:
                # 发送Ctrl+C信号
                try:
                    process.write(b'\x03')
                    time.sleep(0.1)
                except:
                    pass
                
                # 如果进程仍在运行，则终止它
                if process.state() != QProcess.NotRunning:
                    process.terminate()
                    process.waitForFinished(1000)
                    
                # 如果进程仍在运行，则强制终止
                if process.state() != QProcess.NotRunning:
                    process.kill()
        
        # 清空进程字典
        self.processes.clear()
        
        # 使用系统命令确保所有相关进程都被终止
        try:
            # 终止所有ROS相关进程
            subprocess.run("pkill -f 'ros'", shell=True)
            # 等待一段时间
            time.sleep(1)
            # 关闭所有终端窗口
            subprocess.run("pkill -f xterm", shell=True)
        except:
            pass
        
        self.output_received.emit("所有进程已终止")

class OutputFormatter:
    """输出格式化器，用于格式化输出文本"""
    
    @staticmethod
    def format_output(text, text_edit):
        """格式化输出文本
        
        Args:
            text: 输出文本
            text_edit: QTextEdit对象
        """
        cursor = text_edit.textCursor()
        format = QTextCharFormat()
        
        # 根据文本内容设置颜色
        if "错误" in text or "Error" in text or "error" in text:
            format.setForeground(QColor("#ff6b6b"))  # 红色
        elif "警告" in text or "Warning" in text or "warning" in text:
            format.setForeground(QColor("#feca57"))  # 黄色
        elif "信息" in text or "Info" in text or "info" in text:
            format.setForeground(QColor("#54a0ff"))  # 蓝色
        else:
            format.setForeground(QColor("#f5f6fa"))  # 白色
        
        cursor.movePosition(QTextCursor.End)
        cursor.insertText(text, format)
        text_edit.setTextCursor(cursor)
        text_edit.ensureCursorVisible()
