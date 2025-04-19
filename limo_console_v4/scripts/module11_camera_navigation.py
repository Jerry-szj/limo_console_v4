#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
功能模块11：摄像头导航
支持大白相机或realsense相机选择
支持差速底盘或阿克曼底盘选择
"""

import os
import sys
import time
import signal
import subprocess

def start_camera_navigation(camera_type, chassis_mode):
    """
    启动摄像头导航
    
    Args:
        camera_type: 相机类型，可选值为"dabai"(大白相机)、"realsense"(Realsense相机)
        chassis_mode: 底盘模式，可选值为"diff"(差速)、"ackerman"(阿克曼)
    """
    processes = []
    
    try:
        # 切换到工作目录
        os.chdir(os.path.expanduser("~/agilex_ws"))
        
        # 设置环境变量
        env = os.environ.copy()
        env["ROS_MASTER_URI"] = "http://master:11311"
        
        # 运行source命令
        subprocess.run("source devel/setup.bash", shell=True, executable="/bin/bash")
        
        # 根据相机类型启动相应的相机和rtabmap
        if camera_type == "dabai":
            # 启动大白相机
            cmd1 = "roslaunch astra_camera dabai_u3.launch"
            p1 = subprocess.Popen(cmd1, shell=True, env=env, preexec_fn=os.setsid)
            processes.append(p1)
            print(f"已启动: {cmd1}")
            
            # 等待3秒
            time.sleep(3)
            
            # 启动rtabmap定位
            cmd2 = "roslaunch limo_bringup limo_rtabmap_orbbec.launch localization:=true"
            p2 = subprocess.Popen(cmd2, shell=True, env=env, preexec_fn=os.setsid)
            processes.append(p2)
            print(f"已启动: {cmd2}")
            
        elif camera_type == "realsense":
            # 启动Realsense相机
            cmd1 = "roslaunch realsense2_camera rs_camera.launch align_depth:=true"
            p1 = subprocess.Popen(cmd1, shell=True, env=env, preexec_fn=os.setsid)
            processes.append(p1)
            print(f"已启动: {cmd1}")
            
            # 等待3秒
            time.sleep(3)
            
            # 启动rtabmap定位
            cmd2 = "roslaunch limo_bringup limo_rtabmap.launch localization:=true"
            p2 = subprocess.Popen(cmd2, shell=True, env=env, preexec_fn=os.setsid)
            processes.append(p2)
            print(f"已启动: {cmd2}")
            
        else:
            raise ValueError(f"未知的相机类型: {camera_type}")
        
        # 等待3秒
        time.sleep(3)
        
        # 启动limo_start
        cmd3 = "roslaunch limo_bringup limo_start.launch pub_odom_tf:=true"
        p3 = subprocess.Popen(cmd3, shell=True, env=env, preexec_fn=os.setsid)
        processes.append(p3)
        print(f"已启动: {cmd3}")
        
        # 等待3秒
        time.sleep(3)
        
        # 根据底盘模式启动相应的导航launch文件
        if chassis_mode == "diff":
            cmd4 = "roslaunch limo_bringup limo_navigation_rtabmap.launch"
        elif chassis_mode == "ackerman":
            cmd4 = "roslaunch limo_bringup limo_navigation_rtabmap_ackerman.launch"
        else:
            raise ValueError(f"未知的底盘模式: {chassis_mode}")
        
        p4 = subprocess.Popen(cmd4, shell=True, env=env, preexec_fn=os.setsid)
        processes.append(p4)
        print(f"已启动: {cmd4}")
        
        # 等待3秒
        time.sleep(3)
        
        # 启动rviz
        cmd5 = "roslaunch limo_bringup rtabmap_rviz.launch"
        p5 = subprocess.Popen(cmd5, shell=True, env=env, preexec_fn=os.setsid)
        processes.append(p5)
        print(f"已启动: {cmd5}")
        
        # 等待进程结束或被终止
        while all(p.poll() is None for p in processes):
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("接收到终止信号，正在停止所有进程...")
    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        # 终止所有进程
        for p in processes:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGINT)
                time.sleep(1)
                if p.poll() is None:
                    os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            except:
                pass
        
        print("所有进程已停止")

def stop_all():
    """
    停止所有相关进程
    """
    try:
        # 查找相关进程
        cmd = "ps aux | grep -E 'roslaunch|rosrun' | grep -v grep"
        output = subprocess.check_output(cmd, shell=True).decode('utf-8')
        
        # 提取PID
        for line in output.strip().split('\n'):
            if not line:
                continue
            
            parts = line.split()
            if len(parts) < 2:
                continue
                
            pid = parts[1]
            try:
                # 发送SIGINT信号 (Ctrl+C)
                os.kill(int(pid), signal.SIGINT)
                print(f"已发送终止信号到进程 {pid}")
            except:
                pass
        
        # 等待5秒
        time.sleep(5)
        
        # 关闭所有终端窗口
        subprocess.run("pkill -f xterm", shell=True)
        
    except Exception as e:
        print(f"停止进程时出错: {e}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("用法: python3 module11_camera_navigation.py [start|stop] [camera_type] [chassis_mode]")
        print("camera_type可选值: dabai(大白相机), realsense(Realsense相机)")
        print("chassis_mode可选值: diff(差速), ackerman(阿克曼)")
        sys.exit(1)
        
    action = sys.argv[1]
    
    if action == "start":
        if len(sys.argv) < 4:
            print("启动摄像头导航时必须指定相机类型和底盘模式")
            print("用法: python3 module11_camera_navigation.py start [camera_type] [chassis_mode]")
            print("camera_type可选值: dabai(大白相机), realsense(Realsense相机)")
            print("chassis_mode可选值: diff(差速), ackerman(阿克曼)")
            sys.exit(1)
        
        camera_type = sys.argv[2]
        chassis_mode = sys.argv[3]
        start_camera_navigation(camera_type, chassis_mode)
    elif action == "stop":
        stop_all()
    else:
        print(f"未知操作: {action}")
        print("用法: python3 module11_camera_navigation.py [start|stop] [camera_type] [chassis_mode]")
        print("camera_type可选值: dabai(大白相机), realsense(Realsense相机)")
        print("chassis_mode可选值: diff(差速), ackerman(阿克曼)")
        sys.exit(1)
