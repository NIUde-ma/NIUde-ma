#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import subprocess
import json
import common
import argparse
import os
import threading
import time
from concurrent.futures import ThreadPoolExecutor, as_completed

def deploy_to_car(car_id, ip_address, local_path="./qiandao_check_webs", remote_path="~/ms", password=None):

    local_path = "qiandao_check_webs"
    try:
        if not os.path.exists(local_path):
            common.print_red(f"错误: 本地文件 {local_path} 不存在")
            return {"car_id": car_id, "status": "error", "message": "本地文件不存在"}
        
        # 检查目标IP是否可达
        check_ping = subprocess.run(
            f"ping -c 1 -W 2 {ip_address}",
            shell=True, 
            capture_output=True, 
            text=True,
            timeout=3
        )
        
        if check_ping.returncode != 0:
            return {"car_id": car_id, "status": "error", "message": "无法ping通"}
        
        password = "qingzhou"
        if password:
            mkdir_p = f"sshpass -p '{password}' ssh qcraft@{ip_address} 'mkdir -p {remote_path}' "
            scp_command = f"sshpass -p '{password}' scp -r  {local_path} qcraft@{ip_address}:{remote_path}"
            print(scp_command)
            time.sleep(2)
            chmod_command = f"sshpass -p '{password}' ssh qcraft@{ip_address} 'chmod +x {remote_path}/qiandao_check_webs'"
            time.sleep(2)
            check_start_status = f"sshpass -p '{password}' ssh qcraft@{ip_address} 'sudo bash {remote_path}/qiandao_check_webs/init_.sh'"
        else:
            scp_command = f"scp -o ConnectTimeout=5 -o StrictHostKeyChecking=no {local_path} qcraft@{ip_address}:{remote_path}/"
            chmod_command = f"ssh -o ConnectTimeout=5 -o StrictHostKeyChecking=no qcraft@{ip_address} 'chmod +x {remote_path}/qiandao_check_webs'"
        
        result = subprocess.run(
            scp_command,
            shell=True,
            capture_output=True,
            text=True,
            timeout=10
        )

        if result.returncode == 0:
            subprocess.run(chmod_command, shell=True, capture_output=True, timeout=5)
            common.print_green(f"✓ {car_id} ({ip_address}) 部署成功")
            
            result2 = subprocess.run(
                check_start_status,
                shell=True,
                stderr=subprocess.PIPE,
                stdout=subprocess.PIPE,
                timeout=30
            )

            if result2.returncode == 0:
                if result2.stdout:
                    common.print_green(f"✓ {car_id} ({ip_address}) + {result2.stdout.decode('utf-8').strip()} 自启动服务部署成功")
            else:
                common.print_red(f"X {car_id} ({ip_address}) + {result2.stderr.decode('utf-8').strip()}自启动服务部署失败")

            return {"car_id": car_id, "status": "success", "message": "部署成功"}
        else:
            common.print_red(f"✗ {car_id} ({ip_address}) 部署失败: {result.stderr.strip()}")
            return {"car_id": car_id, "status": "error", "message": result.stderr.strip()}
            
    except Exception as e:
        common.print_red(f"✗ {car_id} ({ip_address}) 部署异常: {str(e)}")
        return {"car_id": car_id, "status": "error", "message": str(e)}


def batch_deploy_all(local_path="./qiandao_check_webs", remote_path="~/ms", max_workers=10):
    """
    批量部署到所有车辆
    """
    try:
        with open('./qiandao.json', 'r', encoding='utf-8') as file:
            data = json.load(file)
        
        total_cars = len(data)
        common.print_blue(f"开始批量部署到 {total_cars} 台车辆...")
        
        results = []
        success_count = 0
        error_count = 0
        
        # 使用线程池并发部署
        with ThreadPoolExecutor(max_workers=max_workers) as executor:
            # 提交所有任务
            future_to_car = {
                executor.submit(deploy_to_car, car_id, ip_address, local_path, remote_path): car_id 
                for car_id, ip_address in data.items()
            }
            
            # 收集结果
            for future in as_completed(future_to_car):
                result = future.result()
                results.append(result)
                if result["status"] == "success":
                    success_count += 1
                else:
                    error_count += 1
        
        # 输出统计信息
        common.print_blue("\n" + "="*50)
        common.print_green(f"部署完成! 成功: {success_count}/{total_cars}")
        if error_count > 0:
            common.print_red(f"失败: {error_count}/{total_cars}")
        
        # 输出失败详情
        failed_cars = [r for r in results if r["status"] == "error"]
        if failed_cars:
            common.print_red("\n失败的车辆:")
            for failed in failed_cars:
                common.print_red(f"  {failed['car_id']}: {failed['message']}")
        
        return results
        
    except FileNotFoundError:
        common.print_red("错误: 找不到 car_config.json 文件")
    except json.JSONDecodeError:
        common.print_red("错误: JSON 文件格式不正确")
    except Exception as e:
        common.print_red(f"错误: {e}")

def batch_deploy_selected(car_ids, local_path="./qiandao_check_webs", remote_path="~/ms", max_workers=5):
    """
    批量部署到指定车辆
    """
    try:
        with open('./car_config.json', 'r', encoding='utf-8') as file:
            data = json.load(file)
        
        selected_data = {}
        for car_id in car_ids:
            if car_id in data:
                selected_data[car_id] = data[car_id]
            else:
                common.print_red(f"警告: 车辆ID '{car_id}' 不存在，已跳过")
        
        if not selected_data:
            common.print_red("错误: 没有有效的车辆ID")
            return
        
        total_cars = len(selected_data)
        common.print_blue(f"开始批量部署到 {total_cars} 台指定车辆...")
        
        results = []
        success_count = 0
        
        with ThreadPoolExecutor(max_workers=max_workers) as executor:
            future_to_car = {
                executor.submit(deploy_to_car, car_id, ip_address, local_path, remote_path): car_id 
                for car_id, ip_address in selected_data.items()
            }
            
            for future in as_completed(future_to_car):
                result = future.result()
                results.append(result)
                if result["status"] == "success":
                    success_count += 1
        
        common.print_blue("\n" + "="*50)
        common.print_green(f"部署完成! 成功: {success_count}/{total_cars}")
        
        return results
        
    except Exception as e:
        common.print_red(f"错误: {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='批量部署文件到车辆')
    parser.add_argument('--all', action='store_true', help='部署到所有车辆')
    parser.add_argument('--cars', nargs='+', type=str, help='指定车辆ID列表 (例如: q3701 q3702 q3703)')
    parser.add_argument('--local', type=str, default="./check", help='本地文件路径 (默认: ./check)')
    parser.add_argument('--remote', type=str, default="~/ms", help='远程部署路径 (默认: ~/ms)')
    parser.add_argument('--threads', type=int, default=10, help='并发线程数 (默认: 10)')
    
    args = parser.parse_args()
    
    if args.all:
        batch_deploy_all(args.local, args.remote, args.threads)
    elif args.cars:
        batch_deploy_selected(args.cars, args.local, args.remote, args.threads)
    else:
        common.print_red("请指定部署范围: 使用 --all 部署到所有车辆，或使用 --cars 指定车辆列表")
        parser.print_help()
