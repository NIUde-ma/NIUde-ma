#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

try:
    import os , subprocess ,re ,sys
    import common , requests , time
    from datetime import datetime, timedelta
except ImportError as e:
    common.print_red(f"not found {e}")

class host_env():
    
    # @__init__
    def __init__(self):
        self.hostname_oriny = "root"
        self.ip_oriny = "10.43.0.28"
        self.ip_beiyun = "192.168.5.111"
        self.LDR_CENTER = "192.168.5.201"
        self.LDR_FRONT_BLIND = "192.168.5.204"
        self.LDR_FRONT_LEFT_BLIND = "192.168.5.202"
        self.LDR_FRONT_RIGHT_BLIND = "192.168.5.203"
        self.LDR_REAR_BLIND = "192.168.5.205"
        self.path_oriny = "/etc/rc.local"
        self.j6_collect_log_path = "/home/qcraft/j6_collect_ipc.log"
        self.ipc_disk_cmd = "lsblk |grep /media/qcraft/qcraft-HD"
        self.get_ipc_disk_mount_name = "lsblk |grep /media/qcraft/qcraft-HD |awk '{print $1}'"
        self.path_beiyun = "/app/qcraft/onboard/lite/launch_plan/plans/orin_chery_collect.pb.txt"
        self.andand = "&&"
        self.new_line = "echo \n \n "
        self.v2_path = "/home/qcraft/qcraft_data/vehicles/v2/"
        self.version_path = "/app/qcraft/release/release_v2.json"
        self.clear_errors_cmd = 'ssh-keygen -f "/home/qcraft/.ssh/known_hosts" -R "10.43.0.28"'
        self.get_log_size_cmd = "df -h |grep '/log'"
        self.get_size_cmd = "df -h |grep '/' |head -n 1"
        self.get_beiyun_size_cmd = "timeout 3 nc 192.168.5.111 4444"
        self.url = 'https://www.baidu.com'
        self.get_ips_md5 = "md5sum /usr/share/camera/*"
        self.get_branch = "git --git-dir="
        self.git_log = "log -1"
        self.get_orin_times = "date +%Y-%m-%d" 
        self.get_env_host = "cat /home/qcraft/qcraft_data/env |grep -i CAR_ID="
        self.get_orin_hosts = "cat /map/.env |grep -i CAR_ID="
        self.get_continer_mount_sh = "docker exec qcraft-release-container cat /qcraft/scripts/start_up_main_j6_collect_ipc.sh |grep -E 'sdc|sdb' |awk '{print $2}'"
        # self.get_continer_mount = "docker exec qcraft-release-container lsblk |grep media"
        self.get_continer_mount_disk = "docker exec qcraft-release-container lsblk |grep mount_data |awk '{print $1}'"
        self.__output__()

    # @__output__
    def __output__(self):
        self.module_list = [
                            "Q-VISION_MODULE",
                            "Q-IMAGE_FORWARDING_MODULE",
                            "Q-TRACKER_MODULE",
                            "Q-LIDAR_PROCESS_MODULE",
                            "Q-AUTONOMY_HMI_MODULE",
                            "Q-NODE_MODULE",
                            "Q-SPIN_PUBLISHER_MODULE",
                            "Q-AROUND_VIEW_MODULE",
                            "Q-VISUAL_LOCALIZATION_MODULE",
                            "Q-VISION_FUSION_MODULE",
                            "Q-AUTONOMY_STATE_MODULE",
                            "Q-POSITIONING_MODULE",
                            "Q-RADAR_PROCESS_MODULE",
                            "Q-TSR_MODULE",
                            "Q-MAP_FUSION_MODULE"
                            ]

        self.systemd_services = "systemctl status qcraft_adnoa.service |grep Active:"
        self.output_for_dirve_car = "当前车辆Pose异常，请重启后再试"
        self.IPC_j6_logs_pose = "spin exceed max delay to OMC clock or latest pose"
        self.get_orin_logs = f"grep -r 'failed' /log/qcraft sort -u |head -n 10"
        self.log_list = f"cat {self.j6_collect_log_path} |grep OMC | sort -u"
        self.ipc_disk = "/media/qcraft/qcraft-HD"
        self.mount_name = "sdb"
        self.container_name = "qcraft-release-container"
        self.vehicles_paths = "/home/qcraft/qcraft_data/vehicles/.git"
        self.beiyun_output = "INS_RTKFIXED,"
        self.rc_local = "startup_chery.sh"
        self.by_shm = "shm_usage_check"
        self.oriny_disk_size = "/log"
        self.clean_logs = "rm -rf /var/log/*"
        self.startup = f"Errors：{self.path_oriny}中的自启动脚本未关闭"
        self.startup_atvice = f"True：{self.path_oriny}中的自启动脚本已经被注释"
        self.reboot_orin_cmd = "echo 1 > /sys/class/tegra_hv_pm_ctl/tegra_hv_pm_ctl/device/trigger_sys_reboot"
        self.cleaning_log = f"提示：{self.path_oriny}中，没有{self.clean_logs}，可以手动添加"
        self.found_failed_rc_local = f"cat: /etc/rc.local: No such file or directory"
        self.pose_output_list = "spin exceed max delay to OMC clock or latest pose"

    def ping_lidar(self,ip_address):
        cmd  = ["ping", "-c", "3", ip_address]

        try:
            result = subprocess.run(
                cmd,
                # shell=True,
                stdout=subprocess.PIPE,      
                stderr=subprocess.PIPE,    
                timeout=5
            )

            if result.returncode == 0:
                return 0 , ip_address
            else:
                return 1 , ip_address

        except subprocess.TimeoutExpired:
            common.print_red("cmd command is timeout")
        except Exception as e:
            common.print_red(f"cmd command is errors: {e}")

    def status_lidars(self):
        result_list = []
        
        devices = [
            (self.LDR_CENTER, "中心激光雷达"),
            (self.LDR_FRONT_BLIND, "前盲区激光雷达"),
            (self.LDR_FRONT_LEFT_BLIND, "前左盲区激光雷达"),
            (self.LDR_FRONT_RIGHT_BLIND, "前右盲区激光雷达"),
            (self.LDR_REAR_BLIND, "后盲区激光雷达")
        ]
        
        for ip, name in devices:
            status_code, ip_address = self.ping_lidar(ip)
            
            if status_code == 0:
                status_bool = True
                status_text = "正常"
                icon = "✅"
            else:
                status_bool = False
                status_text = "错误"
                icon = "❌"
            
            result = {
                "device_name": name,
                "ip_address": ip_address,
                "status": status_bool,
                "status_text": status_text,
                "status_code": status_code
            }
            
            result_list.append(result)

            # print(result_list)
            # print(f"{icon} {name} ({ip}): {status_text}")
        
        return result_list

    def get_network_time(self):
        try:
            response = requests.get(self.url, timeout=5)
            date_header = response.headers.get('Date')
            
            if date_header:
                gmt_time = datetime.strptime(date_header, '%a, %d %b %Y %H:%M:%S GMT')
                # beijing_time = gmt_time.replace(hour=(gmt_time.hour + 8) % 24)
                beijing_time = gmt_time + timedelta(hours=8)
                baidu_timestamp = beijing_time.timestamp()
                local_timestamp = time.time()

                diff_time = baidu_timestamp - local_timestamp
                if diff_time <= 300:
                    return 0 , diff_time
                else:
                    return 1 , diff_time

        except Exception as e:
            return 2, f"未知错误: {str(e)}"

    def get_pose_status(self):
        if os.path.exists(self.j6_collect_log_path):
            try:
                result = subprocess.run(
                    self.log_list,
                    shell=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    timeout=10
                )

                if result.returncode == 0:
                    output = result.stdout.decode('utf-8').strip()
                    if output is not None:
                        if self.IPC_j6_logs_pose in output:
                            return 1 , self.output_for_dirve_car
                        else:
                            return 0 , f"not found pose errorcode"
                    else:
                        return 1 , f"cmd command failed"

            except subprocess.TimeoutExpired:
                common.print_red("cmd command is timeout")
            except Exception as e:
                common.print_red(f"cmd command is errors: {e}")  

        else:
            return 1 , f"not found IPC j6_log"

    def Reboot_orin(self):
        Reboot_cmd = f"ssh {self.hostname_oriny}@{self.ip_oriny} '{self.reboot_orin_cmd}'"
        # print(Reboot_cmd)        

        try:
            result = subprocess.run(
                Reboot_cmd,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=20
            )

            if result.returncode == 0 :
                # time.sleep()
                return 0 , f"Orin-y is rebooting please wait 2 min , check process modules aging"
            else:
                return 1 , f"Orin-y is rebooting False"
        
        except subprocess.TimeoutExpired:
            common.print_red("cmd command is timeout")
        except Exception as e:
            common.print_red(f"cmd command is errors: {e}")  

    def get_orin_system_service_status(self):
        service_status_cmd = f"ssh {self.hostname_oriny}@{self.ip_oriny} {self.systemd_services}"

    def get_orin_logs_(self):
        logs_command_cmd = f"ssh {self.hostname_oriny}@{self.ip_oriny} {self.get_orin_logs}"

        try:
            result = subprocess.run(
                logs_command_cmd,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=10
            )

            if result.returncode == 0:
                output = result.stdout.decode('utf-8')
                if output is not None:
                    return 0 , output.strip()
                    # common.print_red(output.strip())
                else:
                    return 1 , output
                    # common.print_red(f"get orin-y logs is failed")
            else:
                return 1 , output
                # common.print_red("cmd command is errors")
        
        except subprocess.TimeoutExpired:
            common.print_red("cmd command is timeout")
        except Exception as e:
            common.print_red(f"cmd command is errors: {e}")  


    def get_disk_status_for_ipc(self):
        try:
            result = subprocess.run(
                self.ipc_disk_cmd,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=5
            )

            if result.returncode == 0:
                output = result.stdout.decode('utf-8')
                if output is not None:
                    if self.ipc_disk in output and self.mount_name in output:
                        return 0 , output
                    else:
                        return 1 , output
                else:
                    return 1 , output   

        except subprocess.TimeoutExpired:
            common.print_red("cmd command is timeout")
        except Exception as e:
            common.print_red(f"cmd command is errors: {e}")           
    
    def get_docker_status(self):
        try:
            import docker
        except ImportError:
            subprocess.run(["pip3", "install", "docker"], check=True)
            import docker
        
        try:
            client = docker.from_env()
            container = client.containers.get(self.container_name)
            status = container.status
            
            if status == "running":
                return 0
            else:
                return 1
        
        except docker.errors.NotFound:
            return 1
        except Exception as e:
            common.print_red(f"未知错误: {e}")
            return 1
            
    def docker_data_status(self):
        if self.get_docker_status() == 0:
            try:
                result = subprocess.run(
                    self.get_continer_mount_sh,
                    shell=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    timeout=5
                    )

                # result2 = subprocess.run(
                #     self.get_continer_mount,
                #     shell=True,
                #     stdout=subprocess.PIPE,
                #     stderr=subprocess.PIPE,
                #     timeout=5
                # )
                # if result is not None:
                if result.returncode == 0:
                    output = result.stdout.decode('utf-8').strip()
                    # output2 = result2.stdout.decode('utf-8').strip()
                    if self.mount_name in output:
                        # print(f"{output}")
                        return 0 , output
                    else:
                        # print(f"errors{output}")
                        return 1 , output
                else:
                    # print("ggg")
                    return 1 , output

            except subprocess.TimeoutExpired:
                common.print_red("cmd command is timeout")
            except Exception as e:
                common.print_red(f"cmd command is errors: {e}") 
                
        else:
            common.print_red(f"Not Found {self.container_name}")

    def diff_docker_mount_ipc_mount(self):
        if self.get_docker_status() == 0:
            try:
                result = subprocess.run(
                    self.get_continer_mount_disk,
                    shell=True,
                    stderr=subprocess.PIPE,
                    stdout=subprocess.PIPE,
                    timeout=5
                )
                
                result2 = subprocess.run(
                    self.get_ipc_disk_mount_name,
                    shell=True,
                    stderr=subprocess.PIPE,
                    stdout=subprocess.PIPE,
                    timeout=5

                )
                
                if result.returncode == 0 and result2.returncode ==0:
                    output = result.stdout.decode('utf-8').strip()
                    output2 = result2.stdout.decode('utf-8').strip()

                    if output == output2:
                        # print("True")
                        return 0 , f"Ipc：{output2} == docker：{output}"
                    else:
                        return 1 , f"Ipc：{output2} != docker：{output}"
                else:
                    common.print_red(f"command Failed")

            except subprocess.TimeoutExpired:
                common.print_red("cmd command is timeout")
            except Exception as e:
                common.print_red(f"cmd command is errors: {e}")  
        else:
            common.print_red("docker status is Failed")

    def get_orin_host(self):
        orin_hostname_cmd = f"ssh {self.hostname_oriny}@{self.ip_oriny} {self.get_orin_hosts}"

        try:
            result = subprocess.run(
                orin_hostname_cmd,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=10
            )

            if result.returncode == 0:
                output = result.stdout.decode('utf-8')
                # print(str(output)[-5:])
                return 0 , str(output)[-5:]
            else:
                # common.print_red(output)
                return 1 , output
        
        except subprocess.TimeoutExpired:
            common.print_red("cmd command is timeout,请检查域控/目录容量是否满足需求！")
        except Exception as e:
            common.print_red(f"cmd command is errors: {e}") 

    def get_env_hosts(self):
        env_name_cmd = f"{self.get_env_host}"

        try:
            result = subprocess.run(
                env_name_cmd,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=10
            )

            if result.returncode == 0:
                output = result.stdout.decode('utf-8')
                # common.print_blue(str(output)[-5:])
                return 0, str(output)[-5:]
            else:
                # common.print_red(output)
                return 1 , output

        except subprocess.TimeoutExpired:
            common.print_red("cmd command is timeout")
        except Exception as e:
            common.print_red(f"cmd command is errors: {e}") 


    def get_network(self):
        import requests
        try:
            response = requests.get(self.url)
            if response.status_code == 200:
                return 0
            else:
                return 1

        except requests.exceptions.SSLError as e:
            error_msg = str(e)
            common.print_red(f"SSL证书错误: {error_msg}")
            sys.exit(1)

    def diff_car_host_envhost(self):
        branch_cmd_command = f"{self.get_branch}{self.vehicles_paths} {self.git_log}"
        # common.print_green(branch_cmd_command)

        try:
            result = subprocess.run(
                branch_cmd_command,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=10
            )

            if result.returncode == 0:
                output = result.stdout.decode('utf-8')
                
                
                if self.get_env_hosts()[0] == 0:
                    car_ids = self.get_env_hosts()[1]
                    # print(int(car_ids))
                    clean_car_ids = str(car_ids).strip()
                    clean_output = output.replace('\n', ' ')

                    patterns = [f"[Q{clean_car_ids}]", f"Q{clean_car_ids}", f"[{clean_car_ids}]", str(clean_car_ids)]
                    if any(pattern in clean_output for pattern in patterns):
                        return 0, car_ids
                    else:
                        import glob
                        car_file_path = glob.glob(self.v2_path + f"Q{int(car_ids)}.pb.txt")
                        # print(f"{self.v2_path }/Q{int(car_ids)}.pb.txt")
                        if car_file_path:
                            # print(car_file_path)
                            return 2 , car_file_path
                        else:
                            # print("not found")
                            return 1 , int(car_ids)
                else:
                    common.print_red(f"{car_ids}")
            else:
                common.print_red(result.stdout)
        
        except subprocess.TimeoutExpired:
            common.print_red("cmd command is timeout")
        except Exception as e:
            common.print_red(f"cmd command is errors: {e}")

    def get_orin_time(self):
        time_cmd = f"ssh {self.hostname_oriny}@{self.ip_oriny} {self.get_orin_times}"
        try:
            result = subprocess.run(
                time_cmd,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=5
            )
            # print(result)
            if result.returncode == 0:
                output = result.stdout.decode('utf-8').strip()
                # print(output)
                if output:
                    return [0 , output]
                else:
                    return [1 , output]
            else:
                common.print_red(f"cmd command failed")
        
        except subprocess.TimeoutExpired:
            common.print_red("cmd command is timeout")
        except Exception as e:
            common.print_red(f"cmd command is errors: {e}")

    def get_size_log(self):
        commands = {
            'log': f"ssh {self.hostname_oriny}@{self.ip_oriny} {self.get_log_size_cmd}",
            'size': f"ssh {self.hostname_oriny}@{self.ip_oriny} {self.get_size_cmd}"
        }
        
        warnings = []
        
        try:
            for name, cmd in commands.items():
                result = subprocess.run(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5)
                
                if result.returncode != 0:
                    common.print_yellow(f"{name} cmd return is known")
                    continue
                    
                percent = int(result.stdout.decode('utf-8').split()[-2].rstrip('%'))
                
                if percent > 90:
                    warnings.append((name, percent))

            log_size = False
            path_size = False

            if warnings:
                # print(warnings)
                for name, percent in warnings:
                    if name == 'log':
                        # log_size = True
                        common.print_newline()
                        common.print_red(f"Errors：域控内部/log快满了，请及时清理：{percent}%")
                        return 1 , percent
                    else:
                        common.print_newline()
                        # path_size = True
                        common.print_red(f"Errors：域控根目录/快满了，请及时清理：{percent}%")
                        return 2 , percent
            else:
                return 0 , percent
                
        except subprocess.TimeoutExpired:
            common.print_red("cmd command is timeout")
        except Exception as e:
            common.print_red(f"cmd command is errors: {e}")


    def get_process_status(self):
        error = {}
        niude = {}

        for process in self.module_list:
            check_process_cmd = f"ssh {self.hostname_oriny}@{self.ip_oriny} 'ps aux | grep -i \"{process}\" | grep -v grep | awk \"{{print \\$2}}\"'"
            
            # print(f"Checking process: {process}")
            # print(f"Command: {check_process_cmd}")

            try:
                result = subprocess.run(
                    check_process_cmd,
                    shell=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    timeout=10
                )
                
                if result.returncode == 0:
                    pid = result.stdout.strip()
                    if pid:
                        pids = pid.decode('utf-8')
                        niude[process] = pids
                    else:
                        niude[process] = "Process not found"
                        # common.print_red(f"{niude[process]} not found ")
                else:
                    common.print_red(f"Command failed for {process}: {result.stderr}")
                    error[process] = result.stderr

            except subprocess.TimeoutExpired:
                common.print_red(f"Timeout checking {process}")
                error[process] = "Timeout"
            except Exception as e:
                common.print_red(f"Error checking {process}: {e}")
                error[process] = str(e)
        
        if niude:
            return niude
        else:
            return error


    # def get_size_log(self):
    #     log_size_cmd = f"ssh {self.hostname_oriny}@{self.ip_oriny} {self.get_log_size_cmd}"
    #     size_cmd = f"ssh {self.hostname_oriny}@{self.ip_oriny} {self.get_size_cmd}"

    #     try:
    #         result = subprocess.run(
    #             log_size_cmd,
    #             shell=True,
    #             stdout=subprocess.PIPE,
    #             stderr=subprocess.PIPE,
    #             timeout=5
    #         )

    #         result2 = subprocess.run(
    #             size_cmd,
    #             shell=True,
    #             stdout=subprocess.PIPE,
    #             stderr=subprocess.PIPE,
    #             timeout=5
    #         )

    #         if result.returncode == 0 and result2.returncode == 0:
    #             # if result.stdout.strip():
    #                 # print(result.stdout)
                
    #             log_size_output = result.stdout.decode('utf-8').split(" ")[-2:-1]
    #             size_output = result2.stdout.decode('utf-8').split(" ")[-2:-1]

    #             log_size_clean = log_size_output[0]
    #             log_clean = log_size_clean.rstrip('%')
    #             size_output_clean = size_output[0]
    #             size_clean = size_output_clean.rstrip('%')

    #             if int(log_clean) > 90 and int(size_clean) > 90:
    #                 common.print_red(f"Errors：域控内部/log快满了，请及时清理：{log_size_clean} \n"  + 
    #                                 f"Errors：域控根目录快满了，请及时清理：{size_output_clean}")
    #                 return 1

    #         else:
    #             common.print_yellow("cmd return is known")


    #     except subprocess.TimeoutExpired:
    #         common.print_red("cmd command is timeout")
    #     except Exception as e:
    #         common.print_red(f"cmd command is errors: {e}")


    def beiyun(self):
        check_ping = subprocess.run(
            f"ping -c 1 -W 2 {self.ip_beiyun}",
            shell=True, 
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=5
        )
         
        if check_ping.returncode == 0:
            # common.print_green(f"ping beiyun ok")
            return 0
        else:
            # common.print_red(f"北云ping不通，请及时排查，或者提醒师傅重新上电后搜星！")
            return 1

    def beiyun_gnss(self):
        cmd_command = f"{self.get_beiyun_size_cmd}"
        try:
            result = subprocess.run(
                cmd_command,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=5
            )

            if result.stdout is not None:
                # if self.beiyun_output in result.stdout:
                output = result.stdout.decode("utf-8")
                if self.beiyun_output in output:
                    return 0 
                else:
                    return 1 , output
                    # common.print_red(output)
            else:
                return 1
        
        except subprocess.TimeoutExpired:
            common.print_red("cmd command is timeout")
        except Exception as e:
            common.print_red(f"cmd command is errors: {e}")


    def orin_(self):
        check_ping = subprocess.run(
            f"ping -c 2 -W 2 {self.ip_oriny}",
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=10
        )

        if check_ping.returncode == 0:
            return 0
        else:
            return 1

    def get_isp(self):
        isp_cmd = f"ssh {self.hostname_oriny}@{self.ip_oriny} {self.get_ips_md5}"
        try:
            result = subprocess.run(
                isp_cmd,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=10
            )

            if result:
                return result.stdout
            else:
                return 1

        except subprocess.TimeoutExpired:
            common.print_red("cmd command is timeout")
        except Exception as e:
            common.print_red(f"cmd command is errors: {e}")

    def shm_check(self):
        # common.print_blue(f"connect to oriny ...")
        shm_command = f"ssh {self.hostname_oriny}@{self.ip_oriny} cat {self.path_beiyun} |grep {self.by_shm}"
        try:
            result = subprocess.run(
                shm_command,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=5
            )

            if result.returncode == 0:
                # if result.stdout.strip():
                    # print(result.stdout)
                return result.stdout 
            else:
                common.print_yellow("cmd return is known")


        except subprocess.TimeoutExpired:
            common.print_red("cmd command is timeout")
        except Exception as e:
            common.print_red(f"cmd command is errors: {e}")

    def rc_local_check(self):
        # common.print_blue(f"connect to oriny ...")

        rc_local_command = f"ssh {self.hostname_oriny}@{self.ip_oriny} cat {self.path_oriny}"
        try:    
            result = subprocess.run(
                rc_local_command,
                shell=True, 
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True,
                timeout=10
            )
            
            if result.returncode == 0 and result.returncode is not None:
                # common.print_green("cmd run")
                if result.stdout.strip():
                    # common.print_blue(result.stdout)
                    script_found = False
                    script_active = False
                    lines = result.stdout.split('\n')
                    for line in lines:
                        if self.rc_local in line:
                            script_found = True
                            if line.strip().startswith('#'):
                                # print(line)
                                script_active = True
                                break
                                
                    if self.rc_local in result.stdout and script_active == True:
                        return 1 , self.startup_atvice
                    elif self.rc_local in result.stdout and script_active == False:
                        return 3 , self.startup
                    else:
                        return 0 
                        # common.print_green(result.stdout)
                        # if self.clean_logs in result.stdout:
                        #     return 0 
                        # else:
                        #     # common.print_yellow(f"")
                        #     return 2 , self.cleaning_log
                else:
                    common.print_yellow("cmd return is known")
            else:
                # common.print_red(f"cmd error (return : {result.returncode})")
                # print("111")
                return 1 , self.found_failed_rc_local
                
                if result.stderr:
                    print("eee")
                    # common.print_red(f"error code: {result.stderr}")
                    return 1 , self.found_failed_rc_local

        except subprocess.TimeoutExpired:
            common.print_red("cmd command is timeout")
        except Exception as e:
            common.print_red(f"cmd command is errors: {e}")

    def clean_log_diff(self):
        rc_local_command = f"ssh {self.hostname_oriny}@{self.ip_oriny} cat {self.path_oriny}"
        try:    
            result = subprocess.run(
                rc_local_command,
                shell=True, 
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True,
                timeout=10
            )
            
            if result.returncode == 0 and result.returncode is not None:
                if result.stdout.decode('utf-8').strip():
                    print()
                    if self.clean_logs in result.stdout:
                        return 0 
                    else:
                        return 1 , self.cleaning_log
            else:
                return 1 , f"not found /etc/rc.local"
        except subprocess.TimeoutExpired:
            common.print_red("cmd command is timeout")
        except Exception as e:
            common.print_red(f"cmd command is errors: {e}")

    def check_version(self):
        version_check_command = f"ssh {self.hostname_oriny}@{self.ip_oriny} cat {self.version_path} "        
        
        try:
            result = subprocess.run(
                version_check_command,
                shell=True, 
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True,
                timeout=10
            )
            
            if result.returncode == 0:
                # common.print_green("cmd run")
                if result.stdout.strip():
                    # print(result.stdout)
                    text = r"rc\d+"
                    version = re.findall(text,result.stdout)
                    need_version = ''.join(version)
                    # print(need_version)
                    if "rc" not in need_version:
                        # common.print_blue(f"version is {version}")
                        return 1 , version
                    # common.print_blue(f"version is {version}")INS_RTKFIXED
                    else:
                        return 0 ,need_version 
                else:
                    common.print_red(f"not found version {version}")


            else:
                common.print_red(f"cmd error (return : {result.returncode})")
                if result.stderr:
                    common.print_red("error code:")
                    print(result.stderr)
                    
        except subprocess.TimeoutExpired:
            common.print_red("cmd command is timeout")
        except Exception as e:
            common.print_red(f"cmd command is errors: {e}")


if __name__ == "__main__":
    hostenv = host_env()
    # hostenv.status_lidars()
    hostenv.Reboot_orin()
    # hostenv.get_orin_logs_()
    # hostenv.docker_data_status()
    # hostenv.diff_docker_mount_ipc_mount()
    # hostenv.get_network_time()
    # hostenv.diff_car_host_envhost()
    # hostenv.get_orin_host()
    # hostenv.beiyun_gnss()
    # hostenv.get_process_status()
    # hostenv.get_size_log()
    # hostenv.shm_check()
    #hostenv.rc_local_check()
    # hostenv.check_version()