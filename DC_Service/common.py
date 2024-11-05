#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import argparse
class DiskInfo():
    def __init__(self):
        self.disk_name = ""
        self.disk_size = 0
        
    def get_disk(self,path):
        
        import os
        import sys 
        import time 
        import argparse
        import subprocess
        import psutil

        
        usage = psutil.disk_usage('/data')
        self.disk_size = usage.total / (1024 ** 3)
        # print(f"总磁盘大小：{self.disk_size:.2f} GB")
        
        
        self.disk_name = psutil.disk_partitions()
        
        for name in self.disk_name:
            print(f"Device: {name.device}")
            print(f"Mountpoint: {name.mountpoint}")
            # print(f"Fstype: {name.fstype}")
            # print(f"Opts: {name.opts}")
            print()
            # print(f"size: {name}")
            # if "/data" in {name.mountpoint}:
            #     print(3131)
        
        if "/data" in {name.mountpoint} and int(self.disk_size) >= 100 :
            print("find disk is mount")
            print(f"{name.device}")
            print(f"{path}")
            
            # umount_code = subprocess.run("sudo -S echo 'auto' | fuser -m -v -i -k /media/data_collect",shell=True, check=True)
            # umount_codes = subprocess.run("sudo -S echo 'auto' | umount /media/data_collect" ,shell=True, check=True)
            # time.sleep(2)
            
            # mount_code = subprocess.run(f"sudo -S echo 'auto' | mount {name.device} ~/Downloads" ,shell=True, check=True)
            # cp_data_code = subprocess.run(f"sudo -S echo 'auto' | cp /media/data_collect/{path} ~/Downloads" ,shell=True, check=True)
            # sync_code = subprocess.run("sync" ,shell=True, check=True)
            
        else:
            print("检测到未将数采盘加载，保险起见代码自动退出！")
            sys.exit(0)

                
def main_init():
    import os 
    
    try:
        import psutil
        return "lib is found"
    except ModuleNotFoundError:
        os.system("pip3 install psutil")
        return "lib is not found"

# def cp_data(path):
    
#     mount_code = subprocess.run(f"sudo -S echo 'auto' | mount {name.device} ~/Downloads" ,shell=True, check=True)
#     cp_data_code = subprocess.run(f"sudo -S echo 'auto' | cp /media/data_collect/{path} ~/Downloads" ,shell=True, check=True)
#     sync_code = subprocess.run("sync" ,shell=True, check=True)
       
def main():
    import time 
    import argparse
    
    disk_info = DiskInfo()
    lib = main_init()
    
    if lib == "lib is found":
        disk_info.get_disk(args.path)
    else:
        print("lib is not found")
        time.sleep(2)
        disk_info.get_disk(args.path)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='注意要转换包的路径！')
    parser.add_argument('path', metavar='PATH', type=str, help='需要解包的路径：')
    args = parser.parse_args()
    main()