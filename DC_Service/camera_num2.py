#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import argparse
import imaplib
import rosbag
import os
import logging
import re
import datetime

camera_path = [

    "ofilm_surround_front_120_8M",
    "ofilm_surround_front_left_100_2M",
    "ofilm_surround_front_right_100_2M",
    "ofilm_surround_rear_100_2M",
    "ofilm_surround_rear_left_100_2M",
    "ofilm_surround_rear_right_100_2M"
    # "ofilm_around_front_190_1M",
    # "ofilm_around_left_190_1M",
    # "ofilm_around_rear_190_1M",
    # "ofilm_around_right_190_1M"

]

lidar_path =  "lidar"


def analyze_camera_frames(path, camera_path, lidar_path):
    print(camera_path)
    camera_set = set(camera_path)
    uiu = os.listdir(path)
    path_set = set(uiu)
    lidar_paths = set(lidar_path)
    if camera_set.issubset(path_set):
        for camera_name in camera_path:
            camera_path_full = os.path.join(path, camera_name)
            lidar_path_full = os.path.join(path, lidar_path)
            camera_frames = os.listdir(camera_path_full)
            lidar_frames = os.listdir(lidar_path_full)

            if not (camera_frames,lidar_frames):
                print(f"路径 {camera_path_full} 与 {lidar} 下未找到任何帧，请检查！")
                continue

            frame_numbers = []
            lidar_numbers = []

            for frame_name in camera_frames:
                camera_number = int(re.sub(r'\.jpeg$', '', frame_name))
                frame_numbers.append(camera_number)

            for frame_names in lidar_frames:
                lidar_number = int(re.sub(r'\.pcd$', '', frame_names))
                lidar_numbers.append(lidar_number)
                
            lidar_frame_sorted = sorted(lidar_numbers)
            first_lidar_frame = lidar_frame_sorted[0]
            last_lidar_frame = lidar_frame_sorted[-1]
            # print(first_lidar_frame)
            # print(last_lidar_frame)
            frame_numbers_sorted = sorted(frame_numbers)
            first_frame = frame_numbers_sorted[0]
            last_frame = frame_numbers_sorted[-1]

            utc_time_first = datetime.datetime.utcfromtimestamp(first_frame / 1000)
            beijing_time_first = utc_time_first + datetime.timedelta(hours=8)

            utc_time_last = datetime.datetime.utcfromtimestamp(last_frame / 1000)
            beijing_time_last = utc_time_last + datetime.timedelta(hours=8)

            lidar_utc_time_first = datetime.datetime.utcfromtimestamp(first_lidar_frame / 1000)
            lidar_beijing_time_first = lidar_utc_time_first + datetime.timedelta(hours=8)
            
            lidar_utc_time_last = datetime.datetime.utcfromtimestamp(last_lidar_frame / 1000)
            lidar_beijing_time_last = lidar_utc_time_last + datetime.timedelta(hours=8)
            
            lidar_time = lidar_beijing_time_last - lidar_beijing_time_first
            
            lidar_real_time = lidar_time.total_seconds()
            
            lidar_expected_frames = lidar_real_time * 10 #lidar真实帧
    
            lidar_lost_frames = len(lidar_frames) - lidar_expected_frames #算法帧lidar
            # print(lidar_expected_frames)
            # print(lidar_lost_frames)
            niude_lidar = True
            if lidar_lost_frames > 3:
                niude_lidar = False
            else:
                niude_lidar = True
                
            front_time = beijing_time_last - beijing_time_first
            real_time = front_time.total_seconds()

            
            expected_frames = real_time * 20
            lost_frames = len(camera_frames) - expected_frames
            # print(lost_frames)
            lost_frame = ((lost_frames/len(camera_frames))*10000) # listdir

            # lost_fram = (20/(expected_frames)*10000) # 算法实际

            if lost_frame < 3 and niude_lidar == True:
                # print(lost_fram)
                print(f"\n{camera_name} 的第一帧为：{beijing_time_first} 最后一帧为：{beijing_time_last} 的实际帧数为：{len(camera_frames)}，算法得出帧数为：{expected_frames}，未丢帧 , 实际丢帧占比为万分之:{lost_frame} "+
                      
                      f"\nlidar的实际帧为：{lidar_expected_frames}, 解析的帧为：{len(lidar_frames)} 未丢帧！")
            else:
                print(f"{camera_name} 的第一帧为：{beijing_time_first} 最后一帧为：{beijing_time_last} 的实际帧数为：{len(camera_frames)}，算法得出帧数为：{expected_frames}，\033[1;31m 丢：{int(lost_frames)}帧\033[0m ，实际丢帧占比为万分之:{lost_frame}" +
                      f"\nlidar的第一帧为:{first_lidar_frame} 最后一帧为：{last_lidar_frame} ,\033[1;31m 丢：{int(lidar_lost_frames)}帧\033[0m")


    else:
        not_found = [item for item in camera_path if item not in uiu]
        errors_path = os.listdir(path)
        print(f"以下路径未找到，请检查camera_path变量：{not_found}，\nPath以下有这些：{errors_path}")
    
    if not camera_path:
        print(f"\033[1;31m请检查camera_path变量中有无输入正确值！\033[0m")



def read_Csv(path):
    import csv
    import pandas as pd
    import datetime
    # with open(f'{path}/gnss.csv', 'r', encoding='utf-8') as csvfile:
    #     gnss_reader = csv.reader(csvfile)
    #     for row in gnss_reader:
    #         print(row)
    df = pd.read_csv(f'{path}/gnss.csv')
    # times = df['utc_time'].values.tolist()
    times = df['utc_time'].values.tolist()
    status = df['gps_status'].values.tolist()
    
    # seen_times = set()
    for utc_time, gps_status in zip(times, status):
        utc_gnss = datetime.datetime.utcfromtimestamp(utc_time / 1000)
        
        gnss_beijing_time = (utc_gnss + datetime.timedelta(hours=8)).strftime('%Y-%m-%d %H:%M:%S')
        # time_unique = gnss_beijing_time.drop_duplicates()
        if gps_status != 42:
            print(f"问题时间点: {gnss_beijing_time}, GPS 状态: {gps_status} ,对应时间戳为：{utc_time}\n")
        
            
def multiprocessRUN(args):
    import multiprocessing

    bag_path = args.path
    
    pool = multiprocessing.Pool(processes=20)
    

    # analyze_camera_frames_result = pool.apply_async(analyze_camera_frames, (bag_path,camera_path,lidar_path))

    # read_csv_result = pool.apply_async(read_Csv, (bag_path,))
    
    pool.apply_async(analyze_camera_frames, (bag_path,camera_path,lidar_path))
    
    pool.close()
    pool.join()
    # camera_frames_result = analyze_camera_frames_result.get()

    print("\nData processing is done.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='注意要转换包的路径！')
    parser.add_argument('path', metavar='PATH', type=str, help='需要解包的路径：')
    args = parser.parse_args()
    bag_path = args.path
    read_Csv(bag_path)
    # analyze_camera_frames(bag_path, camera_path ,lidar_path) #调试时启用，正常就跑多线程
    multiprocessRUN(args)
