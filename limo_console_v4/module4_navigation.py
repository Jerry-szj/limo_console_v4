#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模块4：雷达导航
支持不同底盘模式的导航功能
"""

import os
import sys
import subprocess
import time
import signal
import argparse

def start_navigation(chassis_mode="diff"):
    """开始雷达导航
    
    Args:
        chassis_mode: 底盘模式，可选值：diff（差速）、ackerman（阿克曼）、mec（麦轮）
    """
    print(f"\033[34m[信息] 开始雷达导航，底盘模式: {chassis_mode}...\033[0m")
    
    processes = []
    
    try:
        # 设置底盘模式
        if chassis_mode == "diff":
            print("\033[34m[信息] 设置差速模式...\033[0m")
            subprocess.run(["rosservice", "call", "/set_motion_mode", "0"], check=True)
        elif chassis_mode == "ackerman":
            print("\033[34m[信息] 设置阿克曼模式...\033[0m")
            subprocess.run(["rosservice", "call", "/set_motion_mode", "1"], check=True)
        elif chassis_mode == "mec":
            print("\033[34m[信息] 设置麦轮模式...\033[0m")
            subprocess.run(["rosservice", "call", "/set_motion_mode", "2"], check=True)
        else:
            print(f"\033[33m[警告] 未知底盘模式: {chassis_mode}，使用默认差速模式\033[0m")
            subprocess.run(["rosservice", "call", "/set_motion_mode", "0"], check=True)
        
        # 启动limo_bringup
        print("\033[34m[信息] 启动limo_bringup...\033[0m")
        limo_process = subprocess.Popen(
            ["roslaunch", "limo_bringup", "limo_start.launch"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        processes.append(limo_process)
        time.sleep(3)
        
        # 启动激光雷达
        print("\033[34m[信息] 启动激光雷达...\033[0m")
        lidar_process = subprocess.Popen(
            ["roslaunch", "limo_bringup", "lidar.launch"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        processes.append(lidar_process)
        time.sleep(3)
        
        # 启动导航
        print("\033[34m[信息] 启动导航...\033[0m")
        nav_process = subprocess.Popen(
            ["roslaunch", "limo_bringup", "limo_navigation.launch"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        processes.append(nav_process)
        time.sleep(3)
        
        # 启动rviz
        print("\033[34m[信息] 启动rviz...\033[0m")
        rviz_process = subprocess.Popen(
            ["roslaunch", "limo_bringup", "limo_navigation_rviz.launch"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        processes.append(rviz_process)
        
        print("\033[34m[信息] 雷达导航已启动，请在RViz中设置目标点进行导航\033[0m")
        print("\033[34m[信息] 按Ctrl+C结束导航\033[0m")
        
        # 等待任意进程结束
        while all(process.poll() is None for process in processes):
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\033[34m[信息] 收到中断信号，正在停止...\033[0m")
    except Exception as e:
        print(f"\033[31m[错误] 启动雷达导航时出错: {e}\033[0m")
    finally:
        # 终止所有进程
        for process in processes:
            if process.poll() is None:
                try:
                    process.terminate()
                    process.wait(timeout=5)
                except:
                    process.kill()
        
        print("\033[34m[信息] 雷达导航已停止\033[0m")

def stop_all():
    """结束所有进程"""
    print("\033[34m[信息] 正在停止所有进程...\033[0m")
    
    try:
        # 查找相关进程
        ros_processes = subprocess.run(
            "ps aux | grep -E 'roslaunch|rosrun' | grep -v grep | awk '{print $2}'",
            shell=True,
            stdout=subprocess.PIPE,
            text=True
        ).stdout.strip().split('\n')
        
        # 发送SIGINT信号 (Ctrl+C)
        for pid in ros_processes:
            if pid:
                print(f"\033[34m[信息] 发送终止信号到进程 {pid}\033[0m")
                try:
                    os.kill(int(pid), signal.SIGINT)
                except:
                    pass
        
        # 等待5秒
        time.sleep(5)
        
        # 检查是否有进程仍在运行，如果有则强制终止
        ros_processes = subprocess.run(
            "ps aux | grep -E 'roslaunch|rosrun' | grep -v grep | awk '{print $2}'",
            shell=True,
            stdout=subprocess.PIPE,
            text=True
        ).stdout.strip().split('\n')
        
        if ros_processes and ros_processes[0]:
            print("\033[33m[警告] 一些进程仍在运行，强制终止...\033[0m")
            for pid in ros_processes:
                if pid:
                    try:
                        os.kill(int(pid), signal.SIGKILL)
                    except:
                        pass
        
        # 关闭所有终端窗口
        subprocess.run("pkill -f xterm", shell=True)
        
        print("\033[34m[信息] 所有进程已停止\033[0m")
        
    except Exception as e:
        print(f"\033[31m[错误] 停止进程时出错: {e}\033[0m")

def main():
    """主函数"""
    parser = argparse.ArgumentParser(description="雷达导航模块")
    parser.add_argument("action", choices=["start", "stop"], help="操作类型")
    parser.add_argument("chassis_mode", nargs="?", default="diff", 
                        choices=["diff", "ackerman", "mec"], 
                        help="底盘模式（仅在start操作时有效）")
    
    args = parser.parse_args()
    
    if args.action == "start":
        start_navigation(args.chassis_mode)
    elif args.action == "stop":
        stop_all()

if __name__ == "__main__":
    main()
