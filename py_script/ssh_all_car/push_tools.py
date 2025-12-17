#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import subprocess
import json
import common
import argparse
import os

def push_folder_to_car(car_id, local_folder):
    """将本地文件夹推送到远程车辆的 ~/ms 目录"""
    try:
        if not os.path.exists(local_folder):
            common.print_red(f"错误: 本地文件夹 '{local_folder}' 不存在")
            return False
        
        if not os.path.isdir(local_folder):
            common.print_red(f"错误: '{local_folder}' 不是文件夹")
            return False

        with open('./car_config.json', 'r', encoding='utf-8') as file:
            data = json.load(file)
        
        if car_id not in data:
            common.print_red(f"错误: 未找到车辆ID '{car_id}'")
            return False
        
        ip_address = data[car_id]
        common.print_blue(f"目标车辆: {car_id} ({ip_address})")

        check_ping = subprocess.run(
            f"ping -c 3 -W 2 {ip_address}",
            shell=True, 
            capture_output=True, 
            text=True,
            timeout=5
        )
        
        if check_ping.returncode != 0:
            common.print_red(f"连接失败: {ip_address} 无法ping通，请联系师傅开启工控机")
            return False

        common.print_green(f"网络连接正常，开始推送文件夹...")

        rsync_command = [
            "rsync", "-avz", "--progress",
            "-e", "ssh",
            f"{local_folder}/",
            f"qcraft@{ip_address}:~/ms/"
        ]
        
        common.print_blue(f"执行命令: {' '.join(rsync_command)}")
        
        result = subprocess.run(rsync_command)
        
        if result.returncode == 0:
            common.print_green(f"✅ 文件夹推送成功: {local_folder} -> {car_id}:~/ms/")
            return True
        else:
            common.print_red(f"❌ 文件夹推送失败")
            return False
            
    except FileNotFoundError:
        common.print_red("错误: 找不到 car_config.json 文件")
        return False
    except json.JSONDecodeError:
        common.print_red("错误: JSON 文件格式不正确")
        return False
    except Exception as e:
        common.print_red(f"错误: {e}")
        return False

def connect_to_car(car_id):
    """SSH连接到车辆（保留原有功能）"""
    try:
        with open('./car_config.json', 'r', encoding='utf-8') as file:
            data = json.load(file)
        
        if car_id not in data:
            common.print_red(f"错误: 未找到车辆ID '{car_id}'")
            return False
        else:
            ip_address = data[car_id]

        check_ping = subprocess.run(
            f"ping -c 3 -W 2 {ip_address}",
            shell=True, 
            capture_output=True, 
            text=True,
            timeout=5
        )
        
        if check_ping.returncode == 0:
            ssh_command = f"ssh qcraft@{ip_address}"
            common.print_blue(f"连接到: {car_id} ({ip_address})")
            subprocess.run(ssh_command, shell=True)
            return True
        else:
            common.print_red(f"连接失败: {ip_address} 无法ping通，请联系师傅开启工控机")
            return False
            
    except Exception as e:
        common.print_red(f"错误: {e}")
        return False

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='车辆管理工具')
    parser.add_argument('-n', '--name', type=str, required=True, help='车辆ID (例如: q3701)')
    
    parser.add_argument('-p', '--push', type=str, metavar='FOLDER', 
                       help='推送本地文件夹到远程车辆的 ~/ms 目录')
    
    parser.add_argument('-c', '--connect', action='store_true', 
                       help='SSH连接到车辆（默认行为）')
    
    args = parser.parse_args()
    
    if args.push:
        # 推送文件夹模式
        success = push_folder_to_car(args.name, args.push)
        if success:
            common.print_green(f"操作完成: {args.push} -> {args.name}")
        else:
            common.print_red("操作失败")
            exit(1)
    elif args.connect:
        connect_to_car(args.name)
    else:
        connect_to_car(args.name)