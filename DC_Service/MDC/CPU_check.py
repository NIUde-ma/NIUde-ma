#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import paramiko
import logging
import time

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

def ssh_connect(host, username, password):
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        ssh.connect(host, username=username, password=password, allow_agent=False, look_for_keys=False)
        logging.info(f"成功连接到 {host}")
        
        # 在远程服务器上执行命令获取进程信息
        command = """python3 -c "import psutil; import json; 
        processes = [];
        for proc in psutil.process_iter(['pid', 'name', 'cpu_percent', 'memory_percent']):
            try:
                processes.append(proc.as_dict(attrs=['pid', 'name', 'cpu_percent', 'memory_percent']));
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                pass;
        print(json.dumps(processes))" """
        
        stdin, stdout, stderr = ssh.exec_command(command)
        output = stdout.read().decode('utf-8')
        error = stderr.read().decode('utf-8')
        
        if error:
            logging.error(f"执行命令时出错: {error}")
        else:
            processes = json.loads(output)
            logging.info(f"-----------------------------进程信息-------------------------------------")
            for proc in processes:
                logging.info(f"进程名: {proc['name']:<20} PID: {proc['pid']:<10} CPU利用率: {proc['cpu_percent']:<10} 内存利用率: {proc['memory_percent']:<10}")
        
    except paramiko.AuthenticationException:
        logging.error("认证失败，请验证您的凭据")
    except paramiko.SSHException as sshException:
        logging.error(f"无法建立SSH连接: {sshException}")
    except paramiko.BadHostKeyException as badHostKeyException:
        logging.error(f"无法验证服务器的 host key: {badHostKeyException}")
    finally:
        ssh.close()

if __name__ == '__main__':
    ssh_connect('192.168.1.70', 'root', 'Huawei12#$')