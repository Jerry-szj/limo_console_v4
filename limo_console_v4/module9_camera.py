#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模块9：摄像头控制
支持大白相机和Realsense相机
"""

import os
import sys
import subprocess
import time
import signal
import argparse

def start_camera(camera_type="dabai"):
    """启动摄像头
    
    Args:
        camera_type: 相机类型，可选值：dabai（大白相机）、realsense（Realsense相机）
    """
    print(f"\033[34m[信息] 启动{camera_type}摄像头...\033[0m")
    
    processes = []
    
    try:
        # 根据相机类型启动不同的节点
        if camera_type == "dabai":
            print("\033[34m[信息] 启动大白相机...\033[0m")
            camera_process = subprocess.Popen(
                ["roslaunch", "astra_camera", "dabai_u3.launch"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            processes.append(camera_process)
            
            # 启动图像查看器
            print("\033[34m[信息] 启动图像查看器...\033[0m")
            viewer_process = subprocess.Popen(
                ["rosrun", "rqt_image_view", "rqt_image_view"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            processes.append(viewer_process)
            
        elif camera_type == "realsense":
            print("\033[34m[信息] 启动Realsense相机...\033[0m")
            camera_process = subprocess.Popen(
                ["roslaunch", "realsense2_camera", "rs_camera.launch"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            processes.append(camera_process)
            
            # 启动图像查看器
            print("\033[34m[信息] 启动图像查看器...\033[0m")
            viewer_process = subprocess.Popen(
                ["rosrun", "rqt_image_view", "rqt_image_view"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            processes.append(viewer_process)
            
        else:
            print(f"\033[33m[警告] 未知相机类型: {camera_type}，使用默认大白相机\033[0m")
            camera_process = subprocess.Popen(
                ["roslaunch", "astra_camera", "dabai_u3.launch"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            processes.append(camera_process)
            
            # 启动图像查看器
            viewer_process = subprocess.Popen(
                ["rosrun", "rqt_image_view", "rqt_image_view"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            processes.append(viewer_process)
        
        print("\033[34m[信息] 摄像头已启动\033[0m")
        print("\033[34m[信息] 按Ctrl+C结束\033[0m")
        
        # 等待任意进程结束
        while all(process.poll() is None for process in processes):
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\033[34m[信息] 收到中断信号，正在停止...\033[0m")
    except Exception as e:
        print(f"\033[31m[错误] 启动摄像头时出错: {e}\033[0m")
    finally:
        # 终止所有进程
        for process in processes:
            if process.poll() is None:
                try:
                    process.terminate()
                    process.wait(timeout=5)
                except:
                    process.kill()
        
        print("\033[34m[信息] 摄像头已停止\033[0m")

def stop_all():
    """结束所有进程"""
    print("\033[34m[信息] 正在停止所有进程...\033[0m")
    
    try:
        # 查找相关进程
        ros_processes = subprocess.run(
            "ps aux | grep -E 'roslaunch|rosrun|rqt_image_view' | grep -v grep | awk '{print $2}'",
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
            "ps aux | grep -E 'roslaunch|rosrun|rqt_image_view' | grep -v grep | awk '{print $2}'",
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
    parser = argparse.ArgumentParser(description="摄像头控制模块")
    parser.add_argument("action", choices=["start", "stop"], help="操作类型")
    parser.add_argument("camera_type", nargs="?", default="dabai", 
                        choices=["dabai", "realsense"], 
                        help="相机类型（仅在start操作时有效）")
    
    args = parser.parse_args()
    
    if args.action == "start":
        start_camera(args.camera_type)
    elif args.action == "stop":
        stop_all()

if __name__ == "__main__":
    main()
