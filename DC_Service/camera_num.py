#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
#@author:shuaima6

import argparse
import imaplib
import os
import logging
import re
import datetime
import sys
import imghdr

string = 'ofilm'
def Find_topic_names(path, string):
    global sorted_folders
    matching_folders = []
    for foldername in os.listdir(path):
        if string in foldername:
            matching_folders.append(foldername)
    sorted_folders = sorted(matching_folders)
    
    return sorted_folders


# camera_path = [

#     "ofilm_surround_front_120_8M",
#     "ofilm_surround_front_left_100_2M",
#     "ofilm_surround_front_right_100_2M",
#     "ofilm_surround_rear_100_2M",
#     "ofilm_surround_rear_left_100_2M",
#     "ofilm_surround_rear_right_100_2M"
#     # "ofilm_around_front_190_1M",
#     # "ofilm_around_left_190_1M",
#     # "ofilm_around_rear_190_1M",
#     # "ofilm_around_right_190_1M"

# ]
# print(camera_path)
lidar_path =  "lidar"


def analyze_camera_frames(path, matching_folders, lidar_path):
    
    camera_set = set(sorted_folders)
    print(camera_set)
    uiu = os.listdir(path)
    path_set = set(uiu) 
    lidar_paths = set(lidar_path)
    if camera_set.issubset(path_set):
        for camera_name in sorted_folders:
            camera_path_full = os.path.join(path, camera_name)
            
            lidar_path_full = os.path.join(path, lidar_path)
            camera_frames = os.listdir(camera_path_full)
            lidar_frames = os.listdir(lidar_path_full)

            if not (camera_frames,lidar_frames):
                print(f"路径 {camera_path_full} 与 {lidar} 下未找到任何帧，请检查！")
                continue

            frame_numbers_jpeg = []
            frame_numbers_jpg = []
            lidar_numbers = []
            
            all_jpeg_or_jpg = None
            
            for frame_name in camera_frames:
                file_extension = os.path.splitext(frame_name)[1].lower()
                if file_extension == '.jpeg':
                    # print("picture is jpeg")
                    camera_number = int(re.sub(r'\.jpeg$', '', frame_name))
                    frame_numbers_jpeg.append(camera_number)
                    all_jpeg_or_jpg = True
                    
                elif file_extension == '.jpg':
                    # print("picture is jpg")
                    camera_number = int(re.sub(r'\.jpg$', '', frame_name))
                    frame_numbers_jpg.append(camera_number)
                    all_jpeg_or_jpg = False
                    
                else:
                    print(f"文件 {frame_name} 不是JPEG或JPG格式，请检查！")
                    sys.exit(1)
            
            for frame_names in lidar_frames:
                lidar_number = int(re.sub(r'\.pcd$', '', frame_names))
                lidar_numbers.append(lidar_number)
                
            lidar_frame_sorted = sorted(lidar_numbers)
            first_lidar_frame = lidar_frame_sorted[0]
            last_lidar_frame = lidar_frame_sorted[-1]
            # print(first_lidar_frame)
            # print(last_lidar_frame)
            
            if all_jpeg_or_jpg == True:
                # print('\033[1;45mpicture is jpeg\n \033[0m')
                frame_numbers_sorted = sorted(frame_numbers_jpeg)
                first_frame = frame_numbers_sorted[0]
                last_frame = frame_numbers_sorted[-1]
                
            elif all_jpeg_or_jpg == False:
                # print('\033[1;45mpicture is jpg\n \033[0m')
                frame_numbers_sorted = sorted(frame_numbers_jpg)
                first_frame = frame_numbers_sorted[0]
                last_frame = frame_numbers_sorted[-1]

            else:
                sys.exit(1)
            
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
                      f"\nlidar的第一帧为:{first_lidar_frame} 最后一帧为：{last_lidar_frame} ,\033[1;31m 丢：{int(lidar_lost_frames)}帧\033[0m" + 
                      f"\n\033[1;31m 注：此代码只能给出一个依据，其中复杂原理需要自行斟酌，如去解析长达1小时的bag才有确切依据~\033[0m")
                
                # print(f"\n\033[1;31m 注：此代码只能给出一个依据，其中复杂原理需要自行斟酌，如去解析长达1小时的bag才有确切依据~\033[0m")

    else:
        not_found = [item for item in sorted_folders if item not in uiu]
        errors_path = os.listdir(path)
        print(f"以下路径未找到，请检查camera_path变量：{not_found}，\nPath以下有这些：{errors_path}")
    
    if not sorted_folders:
        print(f"\033[1;31m传参失败！请检查camera_path变量中有无输入正确值！\033[0m")


class Gnss_path:
    def __init__(self,path):
        self.path = path
        self.gnss_status = [ ]
        self.gnss_times = [ ]
    
def Read_Gnss_Csv(path):
    import csv
    import pandas as pd
    import datetime
    
    # with open(f'{path}/gnss.csv', 'r', encoding='utf-8') as csvfile:
    #     gnss_reader = csv.reader(csvfile)
    #     for row in gnss_reader:
    #         print(row)
    # df = pd.read_csv(f'{path}/gnss.csv')
    global gnss_times
    gnss_path = os.path.join(path, "gnss.csv")
    

    
    if not os.path.exists(gnss_path):
        print(f"{path}下未找到gnss.csv，代码自动退出")
        sys.exit(0)
    else:
        df = pd.read_csv(f'{gnss_path}')
        
    # times = df['utc_time'].values.tolist()
    gnss_times = df['utc_time'].values.tolist()
    status = df['gps_status'].values.tolist()
    
    # seen_times = set()
    for utc_time, gps_status in zip(gnss_times, status):
        utc_gnss = datetime.datetime.utcfromtimestamp(utc_time / 1000)
        
        gnss_beijing_time = (utc_gnss + datetime.timedelta(hours=8)).strftime('%Y-%m-%d %H:%M:%S')
        # time_unique = gnss_beijing_time.drop_duplicates()
        if gps_status != 42:
            print(f"问题时间点: {gnss_beijing_time}, GPS 状态: {gps_status} ,对应时间戳为：{utc_time}\n")
        # else:
        #     print("\033[34m\nGnss_status is True\n\033[0m")
        # else:
        #     print(f"时间点: {gnss_beijing_time}, GPS 状态: {gps_status} ,对应时间戳为：{utc_time}\n")
            
def Read_Vehicle_Csv(path):
    
    import csv
    import pandas as pd
    import datetime
    
    vehicle_path = os.path.join(path,"vehicle.csv")

    if not os.path.exists(vehicle_path):
        print(f"{path}下未找到gnss.csv，代码自动退出")
        sys.exit(0)
    else:
        df = pd.read_csv(f'{vehicle_path}')
        vehicle_time = df["utc_time"].values.tolist()
        
        vgrow_time = abs(vehicle_time[0] - gnss_times[0])

        utc_vehicle = datetime.datetime.utcfromtimestamp(vehicle_time[0] / 1000)

        vehicle_beijing_time = (utc_vehicle + datetime.timedelta(hours=8)).strftime('%Y-%m-%d %H:%M:%S')

        if vgrow_time > 10:
            print(f"\033[1;31;40m Error = {vehicle_beijing_time} \033[0m Gnss与Vehicle的时间差异为{vgrow_time}ms，超出10ms阈值！\n")
        else:
            print(f"\033[1;34;40m True = {vehicle_beijing_time} \033[0m Gnss与Vehicle的时间差异为{vgrow_time}ms~\n")
            

        vehicle_timetamps = pd.DataFrame({'utc_time': vehicle_time})
        vehicle_timetamps['my_time'] = vehicle_timetamps.diff() 
        vehicle_timetamps.loc[0, 'my_time'] = 0
        exceeding_threshold = vehicle_timetamps[vehicle_timetamps['my_time'] > 21]
        # print(exceeding_threshold)
        
        for index, row in exceeding_threshold.iterrows():
            row_utc = datetime.datetime.utcfromtimestamp(row['utc_time'] / 1000)
            row_beijing_time = (utc_vehicle + datetime.timedelta(hours=8)).strftime('%Y-%m-%d %H:%M:%S')
            
            
            print(f"\033[31m问题出在第 -->\033[0m {index+2}行时间戳为: {int(row['utc_time'])}, 真实时间为：{row_beijing_time}，差值: {int(row['my_time'])}ms")
        # print(vehicle_time[0])
        # print(vehicle_time[-1])

        first_time = datetime.datetime.utcfromtimestamp(vehicle_time[0] / 1000)
        # first_beijing_time = (first_time + datetime.timedelta(hours=8)).strftime('%Y-%m-%d %H:%M:%S')
        first_beijing_time = first_time + datetime.timedelta(hours=8)
        last_time = datetime.datetime.utcfromtimestamp(vehicle_time[-1] / 1000)
        # last_beijing_time = (first_time + datetime.timedelta(hours=8)).strftime('%Y-%m-%d %H:%M:%S')
        last_beijing_time = last_time + datetime.timedelta(hours=8)
        Row_time = last_beijing_time - first_beijing_time
        real_time = round(Row_time.total_seconds()) #将实际时间进行取整处理~
        # print(real_time)
        Row_vehicle_msg = real_time * 50
        # print(Row_vehicle_msg)
        vs = Row_vehicle_msg - len(vehicle_time)
        algorithm = ((Row_vehicle_msg - len(vehicle_time)) / len(vehicle_time) * 10000)
        
        my_vehicle = None
        if algorithm > 5:
            my_vehicle = False
            print(f"\033[31mVehicle数量总量出现丢帧！ 算法帧总量为：{Row_vehicle_msg}, 实际为：{len(vehicle_time)}，丢 --> {vs} ，丢帧占比为：{algorithm} \033[0m 起始帧为：{vehicle_time[0]} --> 转为北京时间为：{first_beijing_time} ,最终帧为：{vehicle_time[-1]} --> 转为北京时间为：{last_beijing_time}")
        else:
            print(f"\033[1;34;40m Vehicle frame is True \033[0m")
        # print(round(algorithm))
        
        
    # vgrow_time = abs(vehicle_time[0] - gnss_times[0])
     
    # utc_vehicle = datetime.datetime.utcfromtimestamp(vehicle_time[0] / 1000)
    # vehicle_beijing_time = (utc_vehicle + datetime.timedelta(hours=8)).strftime('%Y-%m-%d %H:%M:%S')
    
    # if vgrow_time > 10:
    #     print(f"\033[1;31;40m Error = {vehicle_beijing_time} \033[0m Gnss与Vehicle的时间差异为{vgrow_time}ms，超出10ms阈值！")
    # else:
    #     print(f"\033[1;34;40m True = {vehicle_beijing_time} \033[0m Gnss与Vehicle的时间差异为{vgrow_time}ms~")
        
    #to gnss 与 vehicle时间戳第一帧进行比对！
    #vehicle 判断连续帧 间隔20ms
    #判断数据时间来对比解析的实际帧数
                     
            
def multiprocessRUN(args):
    import multiprocessing
    bag_path = args.path
    
    pool = multiprocessing.Pool(processes=20)
    

    # analyze_camera_frames_result = pool.apply_async(analyze_camera_frames, (bag_path,camera_path,lidar_path))

    # read_csv_result = pool.apply_async(read_Csv, (bag_path,))
    
    pool.apply_async(analyze_camera_frames, (bag_path,sorted_folders,lidar_path))
    
    pool.close()
    pool.join()
    # camera_frames_result = analyze_camera_frames_result.get()

    print("\n Cameras data processing is done.\n")
    print("---以下处理Gnss与Vehicle的Csv数据---\n")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='注意要转换包的路径！')
    parser.add_argument('path', metavar='PATH', type=str, help='需要解包的路径：')
    args = parser.parse_args()
    bag_path = args.path
    Find_topic_names(bag_path,string)
    if len(sys.argv) == 1 or '-h' in sys.argv or '--help' in sys.argv:
        parser.print_help()
        sys.exit(0)
        
    if not os.path.exists(bag_path):
        print(f"错误：路径 '{bag_path}' 不存在。")
        parser.print_help()
        
    Read_Gnss_Csv(bag_path)
    # analyze_camera_frames(bag_path, sorted_folders ,lidar_path) #调试时启用，正常就跑多线程
    multiprocessRUN(args)
    Read_Vehicle_Csv(bag_path)
    print(f"\033[0;33;40m如有新的需求，请及时联系shuaima6！\033[0m")
