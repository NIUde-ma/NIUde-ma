#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import argparse
import time
import os
from datetime import datetime, timezone, timedelta


class MP4():
    
    def __init__(self):
        
        self.data = None
        self.tc = ""
        self.times = 5
        self.my_dir = []
        
    def find_dir_in_folder(self,path):
        self.my_dir = []
        
        for root, dirs, files in os.walk(path):
            for file in files:
                if file.endswith(".mp4"):
                    self.my_dir.append(os.path.join(root,file))
                    print(self.my_dir)   
        return self.my_dir
    
    def get_local_time_from_timestamp(self,timestamp):
        utc_tz = timezone.utc
        
        utc_datetime = datetime.fromtimestamp(timestamp, tz=utc_tz)
        # dt_with_timezone = datetime(utc_datetime, tzinfo=timezone(timedelta(hours=8)))
        beijing_tz = timezone(timedelta(hours=8))  # UTC+8
        beijing_datetime = utc_datetime.astimezone(beijing_tz)
        # dt_without_timezone = dt_with_timezone.replace(tzinfo=None)
        return beijing_datetime

          
    def get_time(self,path):
        
        my_file = self.find_dir_in_folder(path)
        stat_info = os.stat(path)
        creation_time = round(stat_info.st_ctime)
        beijing_time = self.get_local_time_from_timestamp(creation_time)
        local_time = time.time()
        diff = round(abs(local_time - creation_time) / 60)
        print(f"{path} creat is {stat_info}")
        
        for files in my_file:
            
            # print(f"{files} mssssss")
            files_time = os.stat(files)
            creation_file_time = round(files_time.st_ctime)
            local_time_local = time.time()
            # print(round(abs(local_time_local)))
            diff_times = round(abs(local_time_local - creation_file_time) / 60)
            
            if diff_times < self.times:
                print(f"{files} cmd is scp ")
                
            else:
                print(f"{files} diff_time = {diff_times} min")
        
        return beijing_time
    
    def check_mp(self,path):
        import paramiko
        from moviepy.editor import VideoFileClip
            
            try:
            # 尝试创建VideoFileClip对象
                clip = VideoFileClip(path)
            # 如果成功，文件很可能是完整的
                return True
            except Exception as e:
            # 如果发生错误，文件可能不完整或损坏
                print(f"文件可能不完整或损坏: {e}")
                return False
            
def main(args):

    P4 = MP4()
    file_creation_time = P4.get_time(args.path)
    print(f"Creation time of {args.path}: {file_creation_time}")
    
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='注意要转换包的路径！')
    parser.add_argument('path', metavar='PATH', type=str, help='需要解包的路径：')
    args = parser.parse_args()
    main(args)