#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import paramiko
import subprocess
import psutil
import datetime
import time

def check_version():  ###Get Version
    try:
        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        client.connect('192.168.1.10', username='root', password='', allow_agent=False, look_for_keys=False)

        stdin, stdout, stderr = client.exec_command('cat /asw/control/VERSION')
        control_version = stdout.read().decode('utf-8')

        stdin, stdout, stderr = client.exec_command('cat /asw/planning/VERSION')
        planning_version = stdout.read().decode('utf-8')

        stdin, stdout, stderr = client.exec_command('head -n 1 /asw/CHANGELOG.MD')
        changelog_version = stdout.read().decode('utf-8')

        print(f"Control Version: {control_version}")
        print(f"Planning Version: {planning_version}")
        print(f"Changelog Version: {changelog_version}")

        client.close()
    except Exception as e:
        print(f"Error: {str(e)}")
        
def Cpu_check(): ###cpu and memory check
    try:
        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        client.connect('192.168.1.10', username='root', password='', allow_agent=False, look_for_keys=False)
        
        remote_path = '/asw/'
        local_path = './Cpu_check.py'
        
        sftp = client.open_sftp()
        sftp.put(local_path, remote_path)
        sftp.close()
        
        stdin, stdout, stderr = client.exec_command('python3 /asw/Cpu_check.py')
        cpu_status = stdout.read().decode('utf-8')
        print(f"{cpu_status}")
        client.close()
        
    except Exception as e:
        print(f"Error: {str(e)}")
    
    # now_time = time.strftime('%Y-%m-%d-%H:%M:%S', time.localtime(time.time()))
    # print(now_time)
    # print(u"物理CPU个数: %s" % psutil.cpu_count(logical=False))

    # cpu = (str(psutil.cpu_percent(1))) + '%'
    # print(u"cup使用率: %s" % cpu)

    # free = str(round(psutil.virtual_memory().free / (1024.0 * 1024.0 * 1024.0), 2))
    # total = str(round(psutil.virtual_memory().total / (1024.0 * 1024.0 * 1024.0), 2))
    # memory = int(psutil.virtual_memory().total - psutil.virtual_memory().free) / float(psutil.virtual_memory().total)
    # print(u"物理内存： %s G" % total)
    # print(u"剩余物理内存： %s G" % free)
    # print(u"物理内存使用率： %s %%" % int(memory * 100))

    # print(u"系统启动时间: %s" % datetime.datetime.fromtimestamp(psutil.boot_time()).strftime("%Y-%m-%d %H:%M:%S"))


    # users_count = len(psutil.users())

    # users_list = ",".join([u.name for u in psutil.users()])
    # print(u"当前有%s个用户，分别是 %s" % (users_count, users_list))

    # net = psutil.net_io_counters()
    # bytes_sent = '{0:.2f} Mb'.format(net.bytes_recv / 1024 / 1024)
    # bytes_rcvd = '{0:.2f} Mb'.format(net.bytes_sent / 1024 / 1024)
    # print(u"网卡接收流量 %s 网卡发送流量 %s" % (bytes_rcvd, bytes_sent))

    # disk = psutil.disk_partitions()
    # # print(f"niude{disk}")
    # print(f'\n-----------------------------磁盘信息---------------------------------------')

    # print("系统磁盘信息：" + str(disk))
    # for i in range(1):
    #     o = psutil.disk_usage('/')
    #     print(f"\n / 的总容量：" + str(int(o.total / (1024.0 * 1024.0 * 1024.0))) + "G")
    #     print(f"/ 已用容量：" + str(int(o.used / (1024.0 * 1024.0 * 1024.0))) + "G")
    #     print(f"/ 可用容量：" + str(int(o.free / (1024.0 * 1024.0 * 1024.0))) + "G")

    # print(f'-----------------------------进程信息-------------------------------------')
    # # 查看系统全部进程
    # for pnum in psutil.pids():
    #     p = psutil.Process(pnum)
    #     print(u"进程名: %-20s  内存利用率: %-18s   CPU利用率: %-18s 进程状态: %-10s 创建时间: %-10s " \
    #     % (p.name(), p.memory_percent(),  p.cpu_percent() , p.status(), p.create_time()))
        
if __name__ == '__main__':
    check_version()
    #Cpu_check()