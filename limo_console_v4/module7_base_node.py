#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模块7：基础节点
启动LIMO基础节点
"""

import os
import sys
import subprocess
import time
import signal
import argparse

def start_base_node():
    """启动基础节点"""
    print("\033[34m[信息] 启动LIMO基础节点...\033[0m")
    
    processes = []
    
    try:
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
        
        print("\033[34m[信息] LIMO基础节点已启动\033[0m")
        print("\033[34m[信息] 按Ctrl+C结束\033[0m")
        
        # 等待任意进程结束
        while all(process.poll() is None for process in processes):
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\033[34m[信息] 收到中断信号，正在停止...\033[0m")
    except Exception as e:
        print(f"\033[31m[错误] 启动基础节点时出错: {e}\033[0m")
    finally:
        # 终止所有进程
        for process in processes:
            if process.poll() is None:
                try:
                    process.terminate()
                    process.wait(timeout=5)
                except:
                    process.kill()
        
        print("\033[34m[信息] 基础节点已停止\033[0m")

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
    parser = argparse.ArgumentParser(description="LIMO基础节点模块")
    parser.add_argument("action", choices=["start", "stop"], help="操作类型")
    
    args = parser.parse_args()
    
    if args.action == "start":
        start_base_node()
    elif args.action == "stop":
        stop_all()

if __name__ == "__main__":
    main()
