#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模块16：RViz
启动RViz可视化工具
"""

import os
import sys
import subprocess
import time
import signal
import argparse

def start_rviz():
    """启动RViz"""
    print("\033[34m[信息] 启动RViz...\033[0m")
    
    processes = []
    
    try:
        # 启动RViz
        print("\033[34m[信息] 启动RViz可视化工具...\033[0m")
        rviz_process = subprocess.Popen(
            ["roslaunch", "limo_bringup", "limo_rviz.launch"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        processes.append(rviz_process)
        
        print("\033[34m[信息] RViz已启动\033[0m")
        print("\033[34m[信息] 按Ctrl+C结束\033[0m")
        
        # 等待任意进程结束
        while all(process.poll() is None for process in processes):
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\033[34m[信息] 收到中断信号，正在停止...\033[0m")
    except Exception as e:
        print(f"\033[31m[错误] 启动RViz时出错: {e}\033[0m")
    finally:
        # 终止所有进程
        for process in processes:
            if process.poll() is None:
                try:
                    process.terminate()
                    process.wait(timeout=5)
                except:
                    process.kill()
        
        print("\033[34m[信息] RViz已停止\033[0m")

def stop_all():
    """结束所有进程"""
    print("\033[34m[信息] 正在停止所有进程...\033[0m")
    
    try:
        # 查找相关进程
        ros_processes = subprocess.run(
            "ps aux | grep -E 'roslaunch|rosrun|rviz' | grep -v grep | awk '{print $2}'",
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
            "ps aux | grep -E 'roslaunch|rosrun|rviz' | grep -v grep | awk '{print $2}'",
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
    parser = argparse.ArgumentParser(description="RViz模块")
    parser.add_argument("action", choices=["start", "stop"], help="操作类型")
    
    args = parser.parse_args()
    
    if args.action == "start":
        start_rviz()
    elif args.action == "stop":
        stop_all()

if __name__ == "__main__":
    main()
