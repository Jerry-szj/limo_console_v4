#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LIMO控制台启动脚本
"""

import os
import sys
import subprocess
import time

def check_dependencies():
    """检查依赖项"""
    print("检查依赖项...")
    
    # 检查PyQt5
    try:
        import PyQt5
        print("PyQt5已安装")
    except ImportError:
        print("正在安装PyQt5...")
        subprocess.run([sys.executable, "-m", "pip", "install", "PyQt5"], check=True)
    
    # 检查matplotlib
    try:
        import matplotlib
        print("matplotlib已安装")
    except ImportError:
        print("正在安装matplotlib...")
        subprocess.run([sys.executable, "-m", "pip", "install", "matplotlib"], check=True)
    
    # 检查numpy
    try:
        import numpy
        print("numpy已安装")
    except ImportError:
        print("正在安装numpy...")
        subprocess.run([sys.executable, "-m", "pip", "install", "numpy"], check=True)
    
    # 检查pylimo库
    sys.path.append('/home/ubuntu/upload')
    try:
        import pylimo
        print("pylimo库已找到")
    except ImportError:
        print("警告: pylimo库未找到，请确保pylimo库已正确安装")
    
    # 检查ROS环境
    try:
        import rospy
        print("ROS环境已配置")
    except ImportError:
        print("警告: ROS环境未配置，部分功能可能不可用")
    
    print("依赖项检查完成")

def create_scripts_dir():
    """创建脚本目录"""
    script_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'scripts')
    if not os.path.exists(script_dir):
        os.makedirs(script_dir)
        print(f"已创建脚本目录: {script_dir}")
    else:
        print(f"脚本目录已存在: {script_dir}")

def main():
    """主函数"""
    print("启动LIMO控制台...")
    
    # 检查依赖项
    check_dependencies()
    
    # 创建脚本目录
    create_scripts_dir()
    
    # 启动控制台
    try:
        from limo_console import LimoConsole
        from PyQt5.QtWidgets import QApplication
        
        app = QApplication(sys.argv)
        window = LimoConsole()
        window.show()
        sys.exit(app.exec_())
    except Exception as e:
        print(f"启动控制台时出错: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
