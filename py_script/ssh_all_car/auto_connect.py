#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import subprocess
import json
import common
import argparse

def connect_to_car(car_id):
    try:
        with open('./car_config.json', 'r', encoding='utf-8') as file:
            data = json.load(file)
        
        if car_id not in data:
            common.print_red(f"错误: 未找到车辆ID '{car_id}'")
            return
        else:
            ip_address = data[car_id]
            # common.print_blue(ip_address)

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
        else:
            common.print_red(f"连接失败: {ip_address} 无法ping通，请联系师傅开启工控机")
            print(ip_address)
            exit(1)
            
    except FileNotFoundError:
        common.print_red("错误: 找不到 car_config.json 文件")
    except json.JSONDecodeError:
        common.print_red("错误: JSON 文件格式不正确")
    except Exception as e:
        common.print_red(f"错误: {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='SSH连接到指定车辆')
    parser.add_argument('-n', '--name', type=str, required=True, help='车辆ID (例如: q3701)')
    args = parser.parse_args()
    
    if connect_to_car(args.name):
        print_green(car_id)