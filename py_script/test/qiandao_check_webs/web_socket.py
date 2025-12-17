#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import common
import command
import math
import os, subprocess
import argparse , json
import json , re ,sys
from http.server import HTTPServer, BaseHTTPRequestHandler

main_host = command.host_env()

def get_car_hosts():
    need_hosts = main_host.get_env_hosts()
    if need_hosts[0] == 0:
        car_hosts = need_hosts[1]
        return car_hosts

class qcraft_check(BaseHTTPRequestHandler):
    def do_GET(self):
        """处理GET请求"""
        if self.path == '/':
            self.handle_root()
        elif self.path == '/status':
            self.handle_status()
        elif self.path == '/orin':
            self.handle_orin()
        elif self.path == '/mount':
            self.handle_mount()
        elif self.path == '/lidar_check':
            self.handle_lidar()
        elif self.path == '/lidar_pose_check':
            self.handle_lidar_pose()
        elif self.path == '/Host':
            self.handle_host()
        elif self.path == '/network':
            self.handle_network()
        elif self.path == '/version':
            self.handle_version()
        elif self.path == '/disk':
            self.handle_disk()
        elif self.path == '/beiyun':
            self.handle_beiyun()
        elif self.path == '/process':
            self.handle_process()
        elif self.path == '/isp':
            self.handle_isp()
        # elif self.path == 'Reboot_Orin-Y':
        #     self.handle_reboot()
        elif self.path == '/full-check':
            self.handle_full_check()
        else:
            self.send_error(404, "Endpoint not found")
    
    def handle_root(self):
        """根路径，显示所有可用接口"""
        self.send_response(200)
        self.send_header('Content-type', 'text/html; charset=utf-8')
        self.end_headers()
        ids = get_car_hosts()

        html = """
        <html>
            <head>
                <title>Q-%s设备监控服务-Version:ND_1.0</title>
                <meta charset="UTF-8">
                <style>
                    body { font-family: Arial, sans-serif; margin: 40px; }
                    .endpoint { background: #f5f5f5; padding: 15px; margin: 10px 0; border-radius: 5px; }
                    .endpoint h3 { margin-top: 0; }
                    a { color: #007bff; text-decoration: none; }
                    a:hover { text-decoration: underline; }
                    .success { color: green; }
                    .warning { color: orange; }
                    .error { color: red; }
                </style>
            </head>
            <body>
                <h1>Q-%s设备监控服务--Version:ND_1.0s</h1>
                <p>版本：1.0</p>
                
                <div class="endpoint">
                    <h3><a href="/full-check">完整检查</a></h3>
                    <p>执行所有检查项目：/full-check</p>
                </div>
                
                <div class="endpoint">
                    <h3>单项检查</h3>
                    <ul>
                        <li><a href="/lidar_check">lidar检查</a> - /lidar</li>
                        <li><a href="/lidar_pose_check">lidar_pose检查</a> - /lidar_pose_check</li>
                        <li><a href="/network">网络检查</a> - /network</li>
                        <li><a href="/status">状态汇总</a> - /status</li>
                    </ul>
                </div>
            </body>
        </html>
        """% (ids, ids)
        self.wfile.write(html.encode('utf-8'))


    def handle_status(self):
        """状态汇总"""
        self.send_response(200)
        self.send_header('Content-type', 'application/json; charset=utf-8')
        self.end_headers()
        
        status = {
            # "orin": self.check_orin(),
            # "hostname": self.check_host(),
            # "network": self.check_network(),
            # "version": self.check_version(),
            # "disk": self.check_disk(),
            "lidar_check": self.check_lidar(),
            "lidar_pose_check": self.check_lidar_pose(),
            # "mount": self.check_mount(),
            # "beiyun": self.check_beiyun(),
            # "process": self.check_process(),
            # "isp": self.check_isp(),
            "timestamp": self.get_timestamp()
        }
        
        formatted_json = json.dumps(status, ensure_ascii=False, indent=2, sort_keys=True)
        self.wfile.write(formatted_json.encode('utf-8'))
    
    # def handle_isp(self):
    #     result = self.check_isp()
    #     self.send_json_response(result)
    
    def handle_lidar(self):
        result = self.check_lidar()
        self.send_json_response(result)

    def handle_host(self):
        result = self.check_host()
        self.send_json_response(result)

    # def handle_orin(self):
    #     """Orin检查"""
    #     result = self.check_orin()
    #     self.send_json_response(result)
    
    def handle_lidar_pose(self):
        result = self.check_lidar_pose()
        self.send_json_response(result)

    def handle_network(self):
        """网络检查"""
        result = self.check_network()
        self.send_json_response(result)
    
    # def handle_version(self):
    #     """版本检查"""
    #     result = self.check_version()
    #     self.send_json_response(result)
    
    # def handle_mount(self):
    #     """挂载检查"""
    #     result = self.check_mount()
    #     self.send_json_response(result)

    # def handle_disk(self):
    #     """磁盘检查"""
    #     result = self.check_disk()
    #     self.send_json_response(result)
    
    def handle_beiyun(self):
        """北云检查"""
        result = self.check_beiyun()
        self.send_json_response(result)
    
    # def handle_process(self):
    #     """进程检查"""
    #     result = self.check_process()
    #     self.send_json_response(result)
    
    def handle_full_check(self):
        """完整检查"""
        self.send_response(200)
        self.send_header('Content-type', 'application/json; charset=utf-8')
        self.end_headers()
        
        results = {
            # "network": self.check_network(),
            # "version": self.check_version(),  
            # "mount": self.check_mount(),
            # "disk": self.check_disk(),
            "lidar": self.check_lidar(),
            # "beiyun": self.check_beiyun(),
            "timestamp": self.get_timestamp()
        }

        self.wfile.write(json.dumps(results, ensure_ascii=False, indent=2).encode('utf-8'))
    
    def send_json_response(self, data):
        self.send_response(200)
        self.send_header('Content-type', 'application/json; charset=utf-8')
        self.end_headers()
        
        formatted_json = json.dumps(data, ensure_ascii=False, indent=2, sort_keys=True)
        self.wfile.write(formatted_json.encode('utf-8'))
    
    def get_timestamp(self):
        """获取时间戳"""
        import time
        from datetime import datetime
        local_timestamp = time.time()
        dt = datetime.fromtimestamp(local_timestamp)
        
        year = dt.year        
        month = dt.month      
        day = dt.day          
        
        hour = dt.hour
        minute = dt.minute
        second = dt.second

        return f"{year}-{month}-{day}-{hour}-{minute}-{second}"
    
    def check_lidar(self):
        try:
            lidar_list = main_host.config_lidar_status()
            if lidar_list is not None:
                return lidar_list
            else:
                return lidar_list

        except Exception as e:
            return {"status": "error", "message": f"lidar检查异常: {str(e)}"}

    def check_lidar_pose(self):
        try:
            lidar_poses = main_host.get_pose_status()
            if lidar_poses is not None:
                print(lidar_poses)
                return lidar_poses
            else:
                return lidar_poses
        except Exception as e:
            return {"status": "error", "message": f"lidar_pose检查异常: {str(e)}"}

    def check_host(self):
        try:
            master_host = main_host.get_orin_host()
            diff_output = main_host.diff_car_host_envhost()
            IPC_host = main_host.get_env_hosts()
    
            # common.print_yellow(master_host[1] + diff_output[1] + IPC_host[1])
            # print (int(diff_output[0]))

            if int(master_host[0]) == 0 and int(IPC_host[0]) == 0:
                if master_host[1] == IPC_host[1]:
                    common.print_green(f"Orin_host == {master_host[1]} && Ipc_host == {IPC_host[1]}")
                    if int(diff_output[0]) == 0:
                        # common.print_green(f"env_car：{diff_output[1]} == git config Car_id")
                        return {"status":"success","message":"orin -> IPC -> vehicle_v2 is OK"}
                    else:
                        return {"status":"error","message":f"env_car：{diff_output[1]} != git config Car_id"}
                        # common.print_red(f"env_car：{diff_output[1]} != git config Car_id")
                        # common.print_red(f"严重警告：车辆实际host与配置文件不符合，请速度联系FAE同学查验！")
                else:
                    return {"status":"error","message":f"Orin_host:{master_host[1]} != Ipc_host:{IPC_host[1]}"}
                    # common.print_red(f"Orin_host:{master_host[1]} != Ipc_host:{IPC_host[1]}")
                    # common.print_red(f"严重警告：域控host与ipc的host不符合，请速度联系FAE同学查验！")
        except Exception as e:
            return {"status": "error", "message": f"Host检查异常: {str(e)}"}

    def check_orin(self):
        """检查Orin连接"""
        try:
            result = main_host.orin_()
            if result == 0:
                return {"status": "success", "message": "ping orin-y ok"}
            else:
                return {"status": "error", "message": "ping orin-y failed，请及时联系对应FAE的同学！"}
        except Exception as e:
            return {"status": "error", "message": f"Orin检查异常: {str(e)}"}
    
    def check_network(self):
        """检查网络"""
        try:
            result = main_host.get_network()
            if result == 0:
                return {"status": "success", "message": "网络正常"}
            else:
                return {"status": "error", "message": "网络异常，如长时间不恢复，重新插拔一下SIM卡"}
        except Exception as e:
            return {"status": "error", "message": f"网络检查异常: {str(e)}"}
    
    def check_mount(self):
        try:
            # docker_mount = main_host.docker_data_status()
            # if docker_mount[0] == 0:
            #     common.print_green(f"容器内挂载正常{docker_mount[1]}")
            #     return {"status": "success", "message" :f"{dock}"}
            # else:
            #     common.print_yellow(f"{docker_mount[1]}")
            
            ipc_mount_and_docker_mount = main_host.diff_docker_mount_ipc_mount()
            # print(ipc_mount_and_docker_mount) 
            if ipc_mount_and_docker_mount[0] == 0:
                common.print_green(f"{ipc_mount_and_docker_mount[1]}")
                return {"status": "success", "message" :f"{ipc_mount_and_docker_mount[1]}"}
            else:
                common.print_red(f"{ipc_mount_and_docker_mount[1]}")
                return {"status": "error", "message" :f"{ipc_mount_and_docker_mount[1]}"}
        except Exception as e:
            return {"status": "error", "message": f"挂载检查异常: {str(e)}"}

    def check_version(self):
        """检查版本"""
        try:
            version_str = main_host.check_version()
            if isinstance(version_str, tuple):
                version_code = int(version_str[0])
                version_info = str(version_str[1]) if isinstance(version_str[1], (str, bytes)) else str(version_str[1])
            else:
                version_code = int(version_str)
                version_info = str(version_str)
            
            if version_code == 1:
                return {"status": "error", "message": version_info}
            else:
                version_info_str = version_info.decode('utf-8') if isinstance(version_info, bytes) else version_info
                if int(version_info_str[2:4]) < 14:
                    return {"status": "warning", "message": f"提示：您的版本{version_info_str},低于rc14"}
                else:
                    return {"status": "success", "message": f"版本正常: {version_info_str}"}
        except Exception as e:
            return {"status": "error", "message": f"版本检查异常: {str(e)}"}
    
    def check_disk(self):
        """检查磁盘"""
        # try:
        import shutil
        
        st = os.statvfs("/")
        free_space = (st.f_bavail * st.f_frsize) / (1024 * 1024 * 1024)
        
        ipc_status = "success" if round(free_space) >= 50 else "error"
        ipc_message = f"IPC端根目录剩余容量为: {round(free_space)} GB"
        
        orin_result = main_host.get_size_log()
        mount_disk = main_host.get_disk_status_for_ipc()
        
        if mount_disk[0] == 0:
            mount_status = True

        else:
            mount_status = False

        result = {
            "ipc": {
                "status": ipc_status,
                "message": ipc_message,
                "mount_msg": mount_disk[1].strip(),
                "mount_status": mount_status
            },
            "orin": {}
        }
        
        if orin_result[0] == 1:
            log_percent = orin_result[1]
            result["orin"] = {
                "status": "warning",
                "message": f"/log目录使用率: {log_percent}%"
            }
        elif orin_result[0] == 2:
            root_percent = orin_result[1]
            result["orin"] = {
                "status": "warning", 
                "message": f"根目录/使用率: {root_percent}%"
            }
        elif orin_result[0] == 0:
            result["orin"] = {
                "status": "success",
                "message": "Orin磁盘空间正常"
            }
        else:
            result["orin"] = {
                "status": "error",
                "message": "无法获取Orin磁盘状态"
            }
        print(result)
        return result
            
        # except Exception as e:
        #     return {
        #         "ipc": {
        #             "status": "error", 
        #             "message": f"IPC磁盘检查失败: {str(e)}"
        #         },
        #         "orin": {
        #             "status": "error",
        #             "message": "Orin磁盘检查失败"
        #         }
        #     }
    
    def check_isp(self):
        try:
            isp_result = main_host.get_isp()
            if isp_result != 1:
                isp_output = isp_result.decode('utf-8')
                # common.print_blue(f"isp-version = \n{isp_output}")
                # common.print_newline()
                return {
                    "ISP" : {"isp_version" : f"{isp_output}", "message":"success"}
                }
            else:
                return {
                    "ISP" : {"isp_version" : f"{isp_output}", "message":"errors"}
                }
                # common.print_red(f"get isp version failed")
        except Exception as e:
            return {"status": "error", "message": f"isp检查异常: {str(e)}"}

    def check_beiyun(self):
        """检查北云"""
        try:
            beiyun_result = main_host.beiyun()
            beiyun_status = "success" if beiyun_result == 0 else "error"
            beiyun_message = "ping beiyun ok" if beiyun_result == 0 else "北云ping不通，请及时排查，或者提醒师傅重新上电后搜星！"
            
            shm_result = main_host.shm_check()
            shm_result_str = shm_result.decode('utf-8') if isinstance(shm_result, bytes) else str(shm_result)
            shm_status = "success" if "false" in shm_result_str else "warning"
            shm_message = "node shm is off" if "false" in shm_result_str else "node 共享内存-shm is open，建议关闭！"
            
            gnss_result = main_host.beiyun_gnss()
            
            if isinstance(gnss_result, (tuple, list)) and len(gnss_result) >= 2:
                gnss_code, gnss_data = gnss_result[0], gnss_result[1]
                
                if gnss_code != 1: 
                    gnss_status = "success"
                    gnss_message = "Gnss固定解，状态OK" 
                else: 
                    gnss_status = "warning"
                    gnss_message = "检查状态，当前状态不是固定解"
                    
                gnss_display_message = gnss_data
                # print(gnss_display_message)
            else:    
                if gnss_result == 0:
                    gnss_status = "True"
                    gnss_message = "GNSS固定解"
                    gnss_display_message = f"GNSS检查返回数据: {gnss_result}"
                else:
                    gnss_status = "error"
                    gnss_message = "GNSS非固定解"
                    gnss_display_message = f"GNSS检查返回数据: {gnss_result[1]}"
            
            return {
                "beiyun_connection": {"status": beiyun_status, "message": beiyun_message},
                "shared_memory": {"status": shm_status, "message": shm_message},
                "gnss_fix": {"status": gnss_status, "message": gnss_display_message}
            }
            
        except Exception as e:
            return {
                "beiyun_connection": {"status": "error", "message": f"检查异常: {str(e)}"},
                "shared_memory": {"status": "error", "message": f"检查异常: {str(e)}"},
                "gnss_fix": {"status": "error", "message": f"检查异常: {str(e)}"}
            }

    def check_process(self):
        """检查进程"""
        try:
            status = main_host.get_process_status()
            if status:
                processed_status = {}
                for key, value in status.items():
                    # 处理bytes类型
                    if isinstance(value, bytes):
                        processed_status[str(key)] = value.decode('utf-8', errors='ignore')
                    else:
                        processed_status[str(key)] = str(value)
                
                if any(key.startswith('Q-') for key in processed_status.keys()):
                    return {
                        "status": "success", 
                        "message": "Found running processes",
                        "processes": processed_status
                    }
                else:
                    return {
                        "status": "warning", 
                        "message": "No Q- processes found", 
                        "processes": processed_status
                    }
            else:
                return {"status": "error", "message": "No status returned from process check"}
        except Exception as e:
            return {"status": "error", "message": f"进程检查异常: {str(e)}"}
    
    def log_message(self, format, *args):
        """简化日志输出"""
        print(f"{self.client_address[0]} - [{self.log_date_time_string()}] {format % args}")

def run_server(port=8080):
    """运行HTTP服务器"""
    server = HTTPServer(('0.0.0.0', port), qcraft_check)
    print(f"服务器运行在 http://0.0.0.0:{port}")
    print("可用接口:")
    print("  /           - 显示所有接口")
    print("  /status     - 状态汇总")
    print("  /full-check - 完整检查")
    print("  /orin       - Orin检查")
    # print("  /Host       - Host检查")
    print("  /lidar_check - lidar检查")
    print("  /network    - 网络检查")
    print("  /version    - 版本检查")
    print("  /mount      - 挂载检查")
    print("  /disk       - 磁盘检查")
    print("  /beiyun     - 北云检查")
    print("  /process    - 进程检查")
    print("  /isp    - isp版本检查")
    server.serve_forever()

if __name__ == "__main__":
    print("正在启动设备监控服务...")
    run_server(port=8080)