#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模块1：Gmapping建图
支持开始建图、保存地图和结束操作
"""

import os
import sys
import subprocess
import time
import signal
import argparse

def start_gmapping():
    """开始Gmapping建图"""
    print("\033[34m[信息] 开始Gmapping建图...\033[0m")
    
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
        
        # 启动gmapping
        print("\033[34m[信息] 启动gmapping...\033[0m")
        gmapping_process = subprocess.Popen(
            ["roslaunch", "limo_bringup", "limo_gmapping.launch"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        processes.append(gmapping_process)
        time.sleep(3)
        
        # 启动rviz
        print("\033[34m[信息] 启动rviz...\033[0m")
        rviz_process = subprocess.Popen(
            ["roslaunch", "limo_bringup", "limo_gmapping_rviz.launch"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        processes.append(rviz_process)
        
        print("\033[34m[信息] Gmapping建图已启动，请使用键盘或手柄控制LIMO移动进行建图\033[0m")
        print("\033[34m[信息] 按Ctrl+C结束建图\033[0m")
        
        # 等待任意进程结束
        while all(process.poll() is None for process in processes):
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\033[34m[信息] 收到中断信号，正在停止...\033[0m")
    except Exception as e:
        print(f"\033[31m[错误] 启动Gmapping建图时出错: {e}\033[0m")
    finally:
        # 终止所有进程
        for process in processes:
            if process.poll() is None:
                try:
                    process.terminate()
                    process.wait(timeout=5)
                except:
                    process.kill()
        
        print("\033[34m[信息] Gmapping建图已停止\033[0m")

def save_gmapping_map():
    """保存Gmapping地图"""
    print("\033[34m[信息] 保存Gmapping地图...\033[0m")
    
    try:
        # 创建地图保存目录
        maps_dir = os.path.expanduser("~/maps")
        os.makedirs(maps_dir, exist_ok=True)
        
        # 生成带时间戳的地图名称
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        map_name = f"gmapping_map_{timestamp}"
        
        # 保存地图
        result = subprocess.run(
            ["rosrun", "map_server", "map_saver", "-f", f"{maps_dir}/{map_name}"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            check=True
        )
        
        print(f"\033[34m[信息] 地图已保存到: {maps_dir}/{map_name}.pgm 和 {maps_dir}/{map_name}.yaml\033[0m")
        
    except subprocess.CalledProcessError as e:
        print(f"\033[31m[错误] 保存地图失败: {e.stderr}\033[0m")
    except Exception as e:
        print(f"\033[31m[错误] 保存地图时出错: {e}\033[0m")

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
    parser = argparse.ArgumentParser(description="Gmapping建图模块")
    parser.add_argument("action", choices=["start", "save", "stop"], help="操作类型")
    
    args = parser.parse_args()
    
    if args.action == "start":
        start_gmapping()
    elif args.action == "save":
        save_gmapping_map()
    elif args.action == "stop":
        stop_all()

if __name__ == "__main__":
    main()
