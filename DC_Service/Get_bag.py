#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import paramiko
import os

topic_list = ["/sensor/proto/vision_slot_image",
              "/iflytek/control/control_command"]

def get_latest_date_folder():
    try:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect('10.5.20.58', username='deniu', password='NIUde')
        
        
        remote_dir = '/media/data_collect'  #find 最新的bag！
        stdin, stdout, stderr = ssh.exec_command(f'find /media/data_collect -name "*.bag" | sort -r | head -n 1')
        latest_folder = stdout.read().decode().strip()
        bag_file = os.path.join(latest_folder)
        
        #拼接
        stdin, stdout, stderr = ssh.exec_command(f'ls -t {bag_file} | head -n 1')
        latest_bag_file = os.path.join(latest_folder, stdout.read().decode().strip())
        
        #执行
        stdin, stdout, stderr = ssh.exec_command(f'rosbag info {latest_bag_file}')
        recorded_topics = sorted(set(stdout.readlines()))
        
        ssh.close()
        
        #返回
        return latest_bag_file, recorded_topics
        
    except Exception as e:
        print(f"Error: {str(e)}")
        
if __name__ == '__main__':
    latest_bag_file, recorded_topics = get_latest_date_folder()
    
    if latest_bag_file and recorded_topics:
        print('\n最新的bag文件路径是：', latest_bag_file)
        print('\n录制的主题有：')
        
        for topic in recorded_topics:
            print(topic.strip())
            
            if any([t in topic for t in topic_list]):
                print("Topic检查为True")