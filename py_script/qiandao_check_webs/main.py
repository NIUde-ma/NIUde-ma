#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import common
import command
import Get_Gps_time
import math , sys
import os, subprocess
import argparse , json
import json , re
from http.server import HTTPServer, BaseHTTPRequestHandler


main_host = command.host_env()

def diff_hostname():
    master_host = main_host.get_orin_host()
    diff_output = main_host.diff_car_host_envhost()
    IPC_host = main_host.get_env_hosts()
    
    # common.print_yellow(master_host[1] + diff_output[1] + IPC_host[1])
    # print (int(diff_output[0]))

    if int(master_host[0]) == 0 and int(IPC_host[0]) == 0:
        if master_host[1] == IPC_host[1]:
            common.print_green(f"Orin_host == {master_host[1].strip()} && Ipc_host == {IPC_host[1].strip()}")
            if int(diff_output[0]) == 0:
                common.print_green(f"env_car：{diff_output[1].strip()} == git config Car_id")
            elif int(diff_output[0]) == 2:
                # print(diff_output)
                common.print_yellow(f"env_car：{diff_output[1][0].strip()} in v2_path")

            else:
                common.print_red(f"env_car：{diff_output[1].strip()} != git config Car_id")
                common.print_red(f"严重警告：车辆实际host与配置文件不符合，请速度联系FAE同学查验！")
                sys.exit(1)
        else:
            common.print_red(f"Orin_host:{master_host[1].strip()} != Ipc_host:{IPC_host[1].strip()}")
            common.print_red(f"严重警告：域控host与ipc的host不符合，请速度联系FAE同学查验！")
            sys.exit(1)

    else:
        common.print_red(f"Orin host get Errors && Ipc host get Errors")
    # if int(diff_output[0]) == 0:

def orin():
    if main_host.orin_() == 0:
        common.print_green(f"True：ping orin-y ok")
    else:
        common.print_red(f"严重警告：ping orin-y is failed ，请及时联系对应FAE的同学！")
        sys.exit(1)

def network():
    if main_host.get_network() == 0:
        common.print_green(f"True：网络正常 ...")
        common.print_newline()
        diff_times = main_host.get_network_time()
        if diff_times[0] == 0:
            common.print_green(f"差异时间为：{diff_times[1]}")
        else:
            common.print_yellow(f"差异时间为：{diff_times[1]}")
    else:
        common.print_red(f"Errors：网络异常，如长时间不恢复，重新插拔一下SIM卡")

def mount_status_check():
    docker_mount = main_host.docker_data_status()
    if docker_mount[0] == 0:
        common.print_green(f"容器内挂载正常{docker_mount[1]}")
    else:
        common.print_yellow(f"{docker_mount[1]}")
    
    ipc_mount_and_docker_mount = main_host.diff_docker_mount_ipc_mount()
    if ipc_mount_and_docker_mount[0] == 0:
        common.print_green(f"{ipc_mount_and_docker_mount[1]}")
    else:
        common.print_red(f"{ipc_mount_and_docker_mount[1]}")

def version_check():
    version_str = main_host.check_version()
    # print((version_str[1])[2:4])

    if int((version_str)[0]) == 1:
        common.print_red(version_str[1])
    else:
        if int(((version_str)[1])[2:4]) < 14:
            common.print_yellow(f"提示：您的版本{version_str[1]},低于rc14")
            return 1
        else:
            common.print_green(f"您当前的版本为：{version_str[1]}")
            if version_str[1] == "rc14":
                return 0
            else:
                return 1

def disk_check(path="/"):
    import shutil

    st = os.statvfs(path)
    free_space = (st.f_bavail * st.f_frsize) / (1024 * 1024 *1024)

    if round(free_space) < 50:
        common.print_red(f"Errors：Ipc space < 50GB")
        # return 1
    else:
        common.print_green(f"True：IPC端根目录剩余容量为: {round(free_space)} GB")

    orin_size = main_host.get_size_log()
    ipc_mount = main_host.get_disk_status_for_ipc()
    # print(ipc_mount.)
    common.print_newline
    new_mount = ipc_mount[1].strip()
    common.print_blue(new_mount)
    # print(f"{orin_size} dadad")
    if orin_size == 0:
        common.print_green(f"True：域控容量")
    
    common.print_newline()

    if ipc_mount is not None:
        if ipc_mount[0] == 0:
            common.print_green(f"True：Ipc mount")
        else:
            common.print_red(f"Errors：Ipc mount")
    else:
        common.print_red(f"Errors：Ipc mount {ipc_mount.strip()}")

    
def beiyun_check():
    # print(main_host.beiyun())
    if main_host.beiyun() == 0:
        common.print_green(f"True：ping beiyun ok")
        common.print_newline()
        # print((main_host.shm_check()))
        by_str = main_host.shm_check().decode('utf-8')
        if "false" in by_str:
            common.print_green("node shm is off")
        else:
            common.print_yellow(f"提示：node 共享内存-shm is open，建议关闭！")
            common.print_newline()
    else:
        common.print_red(f"Errors：北云ping不通，请及时排查，或者提醒师傅重新上电后搜星！")

    gnss_status = main_host.beiyun_gnss()
    if gnss_status == 0:
        common.print_newline()
        common.print_green(f"True：Gnss固定解，状态OK")
    else:
        common.print_newline()
        common.print_red(f"警告：检查状态，当前状态不是固定解 \n {gnss_status[1]}")

def diff_orin_ipc_time():
    diff_time_all = Get_Gps_time.diff_main()
    # print(diff_time_all)
    if diff_time_all is not None:
        if diff_time_all[0] == 0:
            common.print_green(f"域控内部时间与GPS时间匹配到日{diff_time_all[1]}")
        else:
            common.print_red(f"域控时间与GPS时间不匹配{diff_time_all[1]}")
    else:
        common.print_red("Get_Gps code is errors")

def oriny_check_local():
    if version_check() == 0:
        common.print_green("当前版本已经是最新，无需rc.local检查")
    
    # print(startup_sh)
    else:
        startup_sh = main_host.rc_local_check()
        if startup_sh == 0:
            common.print_green(f"rc.local中没有额外的自启动服务")
        elif int(startup_sh[0]) == 1:
            common.print_green(f"{startup_sh[1]}")
        # elif int(startup_sh[0]) == 2:
        #     common.print_yellow(startup_sh[1])
        elif int(startup_sh[0]) == 3:
            common.print_red(f"{startup_sh[1]}")

        clean_log = main_host.clean_log_diff()

        if clean_log == 0:
            common.print_newline()
            common.print_green("rc.local中有删除log的服务")
        elif clean_log[0] == 1:
            common.print_newline()
            common.print_yellow(f"{clean_log[1]}")
        # else:


def oriny_process_check():
    status = main_host.get_process_status()
    if status:
        if any(key.startswith('Q-') for key in status.keys()):
            common.print_blue("Found running processes:")
            for process, pid in status.items():
                if "Process not found" in pid:
                    common.print_red(f"Not found {process}: PID {pid}")
                else:
                    common.print_green(f"found {process}: PID {pid}")
        else:
            common.print_red(f"No processes found: {process}")
    else:
        common.print_red("No status returned from process check")

def oriny_isp_version():
    isp = main_host.get_isp()
    if isp != 1:
        isp_output = isp.decode('utf-8')
        common.print_blue(f"isp-version = \n{isp_output}")
        # common.print_newline()
    else:
        common.print_red(f"get isp version failed")

def Orin_Y_logs():
    log_list = main_host.get_orin_logs_()
    
    if log_list[0] == 0:
        common.print_newline()
        common.print_blue(f"正在获取Orin-Y内部日志.....")
        common.print_red(f"{log_list[1]}")
    else:
        common.print_newline()
        common.print_red(f"get log failed {log_list[1]}")

def IPC_pose_check():
    pose_status = main_host.get_pose_status()
    
    if pose_status[0] != 1:
        common.print_green(f"{pose_status[1]}")
    else:
        common.print_red(f"{pose_status[1]}")


def run_server(port=8080):
    server = HTTPServer(('172.20.19.173', port), orin)
    print(f"服务器运行在 http://172.20.19.173:{port}")
    server.serve_forever()

if __name__ == "__main__":
    common.print_newline()
    diff_orin_ipc_time()
    common.print_newline()
    orin()
    common.print_newline()
    diff_hostname()
    common.print_newline()
    network()
    common.print_newline()
    version_check()
    common.print_newline()
    mount_status_check()
    common.print_newline()
    oriny_check_local()
    common.print_newline()
    disk_check()
    common.print_newline()
    beiyun_check()
    common.print_newline()
    oriny_process_check()
    common.print_newline()
    oriny_isp_version()
    # common.print_newline()
    Orin_Y_logs()
    # run_server(port=8080)
    common.print_newline()
    IPC_pose_check()