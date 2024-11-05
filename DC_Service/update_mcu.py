#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import os
import re
import time
import argparse

class UP:
    def __init__(self):
        
        self.local = []
        self.status = "mdc-tool upgrade query status"
        self.get_version = "mdc-tool upgrade display version"
        self.finish = "mdc-tool upgrade finish auto"
        self.mdc_uptime = 0
    
    def get_local_time(self):
        mdc_time = os.popen("cat /proc/uptime | cut -d. -f1").read()
        if int(mdc_time) < 180:
            print("mdc time is init ...")
            return 0
        else:
            print("mdc time is ok ...")
            return 1
        
    def get_status(self):
        mdc_status = os.popen(self.status).read().strip()
        init = "Startup, not have update service! Please enter update-mode!"
        update_module_a = "Upgrade reserve success"
        update_module_b = "Upgrade main success"
        
        if init in mdc_status:
            print("mdc status is True")
            return 1
        elif update_module_a in mdc_status and update_module_b in mdc_status:
            print("mdc install update is True")
            return 2
        else:
            print("mdc status is False please wait ！")
            return 0
    
    def gets_version(self):
        tar_gz = [file for file in os.listdir("/opt/usr/upgrade/download") if file.endswith(".tar.gz")]
        tag_gz_pattern = r"MCU-([\w-]+)\.tar\.gz"

        tar_gz_new = []

        for file in tar_gz:
            match = re.search(tag_gz_pattern, file)
            if match:
                extracted_part = match.group(1)
                tar_gz_new.append(extracted_part)
        
        self.local.append(tar_gz)
        
        self.update_install = f"mdc-tool upgrade install /opt/usr/upgrade/download/{' '.join(tar_gz)} -f 3 all"
        print(self.update_install)
        
        mdc_version = os.popen(self.get_version).read().strip()
        status = self.get_status()
        if status == 1:
            pattern = r"Mcu\s+mcu-app-firmware\s+\w+\s+\w+\s+(\S+)"
            match = re.search(pattern, mdc_version)
            if match:
                mcu_app_firmware_version = match.group(1)
                if tar_gz_new and tar_gz_new[0] != mcu_app_firmware_version:
                    print(f"Mcu mcu-app-firmware version: {mcu_app_firmware_version}--可以升级")
                    return 1
                else:
                    print(f"Mcu版本一致,不需要升级!")
                    return 0
            else:
                print("Mcu mcu-app-firmware version not found")
                
    def install_update(self):
        start_time = time.time()
        version_status = self.gets_version()
        
        if self.get_local_time() == 1:
            if version_status == 1:
                mdc_install_update = os.popen(self.update_install).read().strip()
                if mdc_install_update:
                    print(mdc_install_update)
                else:
                    print("mdc is reboot！")
            else:
                print("Mcu升级条件不满足！")
                exit(1)
        else:
            print("please wait mdc init")
            time.sleep(10)
            self.install_update()
            
        if time.time() - start_time > 300:
            print("Timeout: 代码运行超时，自动退出！")
            exit(1)
                        
    def finish_mcu(self):
        start_time = time.time()
        if self.get_local_time() == 1:
            if self.get_status() == 2:
                
                Verify = "Verify Success."
                Finish_a = "Finish start."
                Finish_b = "Finish done."
                
                mdc_finish = os.popen(self.finish).read().strip()
                if Verify in mdc_finish and Finish_a in mdc_finish and Finish_b in mdc_finish:
                    print("Finish is Done~！")
                    time.sleep(2)
                    if self.get_status() == 1 and self.gets_version() == 0:
                        print(f"恭喜您MDC升级成功！")                        
                else:
                    print("Finish is false!")
                    exit(1)
            else:
                print("mdc status is False please wait ！")
                time.sleep(10)
                self.finish_mcu()
        else:
            print("please wait mdc init")
            time.sleep(10)
            self.finish_mcu()
        
        if time.time() - start_time > 300:
            print("Timeout: 代码运行超时，自动退出！")
            exit(1)

def main(module):
    update = UP()
    name = os.popen("hostname").read().strip()
    
    if name == "AOS_A":
        if module == "update":
            update.install_update()
        elif module == "finish":
            update.finish_mcu()
        else:
            print("please check you input ..")
            parser.print_help() 
            exit(1)
    else:
        print("请检查是否是在A面进行升级操作！")
        exit(1)
        
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='注意要升级的功能!')
    parser.add_argument('module', metavar='MODULE', type=str, help='注意要升级的功能：update、finish')
    args = parser.parse_args()
    main(args.module)
