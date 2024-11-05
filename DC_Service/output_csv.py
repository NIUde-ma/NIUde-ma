#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

#@author:shuaima6

import subprocess
import numpy as np
import bagpy
import rosbag
import pandas as pd
import matplotlib.pyplot as plt
import re
import time
import argparse
import os
import json
import multiprocessing
import logging
import csv

global my_dir

def find_dir_in_folder(bag_path):
    my_dir = []
    for root, dirs, files in os.walk(bag_path):
        for file in files:
            if file.endswith(".bag"):
                my_dir.append(os.path.join(root))
                   
    return my_dir

def find_bags_in_folder(path):
    bags = []
    for root, dirs, files in os.walk(path):
        for file in files:
            if file.endswith(".bag"):
                bags.append(os.path.join(root, file))
                
    logging.info("found %d bag files in %s", len(bags), path)
    
    return bags

def worker(bag_path,json_path):
    bags = []
    # save_path = []
    if os.path.isfile(bag_path):
        bags.append(bag_path)
    elif os.path.isdir(bag_path):
        for root, dirs, files in os.walk(bag_path):
            for file in files:
                if file.endswith(".bag"):
                    bags.append(os.path.join(root, file))
                    # save_path.append(os.path.join(root))
                    #return save_path
    else:
        raise ValueError("bag_path must be a file or directory")

    for my_bag in bags:
        save_path = os.path.dirname(my_bag)
        print(f"注：当前处理的Bag为：{my_bag}")
        bag_data = bagpy.bagreader(my_bag)
        rosbagData = rosbag.Bag(my_bag)
        combined_df = pd.DataFrame()
        # bag_data.topic_table
        topics = bag_data.topic_table['Topics']
        topics = topics.tolist()
        for t in topics:
            print(f"bag got topic {t}")
        #print(topics)
        neededTopics = []
        
        with open(f"{json_path}", "r", encoding="utf-8") as f: 
            content = json.load(f)
            neededTopics = content.values()
            print(neededTopics)
           
        cmds = []
        vehicle_service_topic = "/iflytek/vehicle_service"
        soc_state_topic = "/iflytek/system_state/soc_state"
        navifusion_topic = "/sensor/navi/navifusion"
        Pbox_topic = "/iflytek/sensor/pbox/gnss"
        
        if  "/iflytek/vehicle_service" in topics:
            vs = bag_data.message_by_topic("/iflytek/vehicle_service")
            print(vs)
            vs_path = vs
            png_directory = os.path.dirname(vs_path)
            df = pd.read_csv(vs_path)
            timestamp_column = df['header.timestamp']
            plt.figure(figsize=(30, 6))
            plt.plot(timestamp_column, color='blue', label='Vehicle')
            plt.legend()
            plt.ylabel('timestamp')
            plt.title('Comparison_Chart_check')
            plt.savefig(f"{png_directory}/Vehicle_timestamps.png")
        else:
            print(f"/iflytek/vehicle_service，在{topics}中为找到！")
        
        if  "/iflytek/system_state/soc_state" in topics:
            soc= bag_data.message_by_topic("/iflytek/system_state/soc_state")
            print(soc)
            soc_path = soc
            df3 = pd.read_csv(soc)
            timestamp_column3 = df3['header.timestamp']
            plt.figure(figsize=(30, 6))
            plt.plot(timestamp_column3, color='red',label='Soc_state')
            plt.legend()
            plt.ylabel('timestamp')
            plt.title('Comparison_Chart_check')
            plt.savefig(f"{png_directory}/Soc_state_timestamps.png")
        else:
            print(f"/iflytek/vehicle_service，在{topics}中为找到！")
        
        if "/sensor/navi/navifusion" in topics:
            Gnss = bag_data.message_by_topic("/sensor/navi/navifusion")
            print(Gnss)
            Gnss_path = Gnss
            df2 = pd.read_csv(Gnss_path)
            timestamp_column4 = df2['meta.timestamp_us']
            timestamp_column2 = df2['gps_status']
            plt.figure(figsize=(30, 6))
            plt.plot(timestamp_column2,color='red',label='Gnss_status')
            plt.legend()
            plt.ylabel('status_check')
            plt.title('Gnss_status_timestamps_check')
            plt.savefig(f"{png_directory}/Gnss_status.png")
            plt.figure(figsize=(30,6))
            plt.plot(timestamp_column4,color='red',label='Us_timestamps')
            plt.legend()
            plt.ylabel('timestamps_check')
            plt.title('Gnss_timestamps_Us_check')
            plt.savefig(f"{png_directory}/Gnss_timestamps.png")
        else:
            print(f"/sensor/navi/navifusion，在{topics}中为找到！")
        
        if "/iflytek/sensor/pbox/gnss" in topics:
            pbox = bag_data.message_by_topic("/iflytek/sensor/pbox/gnss")
            print(pbox)
            pbox_path = pbox
            dfp = pd.read_csv(pbox_path)
            # print(dfp)
            timestamp_columnp = dfp['header.timestamp']
            #print(timestamp_columnp)
            plt.figure(figsize=(30, 6))
            plt.plot(timestamp_columnp,color='green',label='Pbox_timestamps')
            plt.legend()
            plt.ylabel('status_check')
            plt.title('Pbox_timestamps_check')
            plt.savefig(f"{png_directory}/Pbox_timestamp.png")
        else:
            print(f"/iflytek/sensor/pbox/gnss，在{topics}中为找到！")  
        
        if "/iflytek/sensor/pbox/imu" in topics:
            imu = bag_data.message_by_topic("/iflytek/sensor/pbox/imu")
            print(imu)
            imu_path = imu
            dfi = pd.read_csv(imu_path)
            # print(dfi)
            
            df_100hz_interpolated = dfi.interpolate(method='linear', limit_direction='forward', axis=0)
            # print(df_100hz_interpolated)
            df_hz = df_100hz_interpolated.iloc[::2].reset_index(drop=True)
            # print(df_hz)
            df_hz.to_csv(f'{png_directory}/new_imu.csv', index=False)
            #print(dfii)
            # print(dfi)
            timestamp_columni = df_hz['header.timestamp']
            # print(timestamp_columni)
            plt.figure(figsize=(30, 6))
            plt.plot(timestamp_columni,color='black',label='imu_timestamps')
            plt.legend()
            plt.ylabel('status_check')
            plt.title('imu_timestamps_check')
            plt.savefig(f"{png_directory}/imu_timestamp.png")
        else:
            print(f"/iflytek/sensor/pbox/imu，在{topics}中为找到！")  
                       
        if "/iflytek/vehicle_service" and "/iflytek/system_state/soc_state" and "/iflytek/sensor/pbox/imu" in topics:
            # my_find = len(timestamp_columni)
            plt.figure(figsize=(30, 6))
            # plt.plot(timestamp_columnp, color='green',label='Soc_state')
            plt.plot(timestamp_column, color='blue', label='Vehicle')
            plt.plot(timestamp_column3, color='red',label='Soc_state') 
            plt.plot(timestamp_columni, color='black',label='imu_state')
            # plt.plot(timestamp_columnp, color='green',label='Soc_state')
            plt.legend()
            plt.xlabel('len')
            plt.ylabel('timestamp')
            plt.title('Comparison_Chart_check')
            plt.savefig(f"{png_directory}/Comparison_timestamps.png")
        else:
            print(f"融合图无法生成，请检查{topics}中是否存在vehicle、Soc_state")    

        # my_lidar_topic = "/sensor/lidar/rotation_packet"
        # if my_lidar_topic in topics:
        #     lidar = bag_data.message_by_topic("/sensor/lidar/rotation_packet")
        #     print(lidar)
        #     df_lidar = pd.read_csv(lidar)
        #     # print(df_lidar)
        cammsg_count={
            
        }
            
        for topic in neededTopics:
            if "camera" not in topic:
                print(f"{topic} don't have camera string,skiped!")
                continue
            if topic in topics:
                tsList = []
                topic_rename = topic.replace("/","_")
                topic_rename = topic_rename.lstrip("_")
                for t,m,s in rosbagData.read_messages(topics=[topic]):
                    try:
                        tsList.append(int(m.header.stamp.to_nsec()/1e6))
                        # print(tsList)
                    except:
                        pass


                csvFilePath = f"{png_directory}/{topic_rename}.csv"
                
                
                if len(tsList)>0:
                    with open(csvFilePath,'w') as f:
                        writer = csv.writer(f)
                        writer.writerow(['timestamps'])
                        camera_name = topic_rename
                        
                        for v in tsList:
                            writer.writerow([v])   
                    print(f"topic {topic} write done!")
                else:
                    print(f"failed to read {topic} timestamp,may be no standand ros msg header!")
                    
                if not os.path.exists(csvFilePath):
                    print(f"{topic} csv file generated failed")
                    continue
                
                print(f"handling {csvFilePath}")
                my_read = pd.read_csv(csvFilePath)
                
                # print(f"11111111111 :{my_read}")
                # a_len = len(my_read['timestamps'])
                if len(my_read['timestamps']) > 0:
                    if combined_df.empty:
                        # my_read['timestamps'] = combined_df['timestamps']
                        combined_df['timestamps'] = my_read['timestamps']
                        # print(combined_df['timestamps'])
                        # print(combined_df['timestamps'])
                    else:
                        combined_df[topic_rename] = my_read['timestamps']
                        # print(combined_df[topic_rename])
                else:
                    print(f"failed to read {topic_rename} CSV file!")
                #print(f"{topic_rename} {len(my_read['timestamps'])}"
                
                # plt.figure(figsize=(40, 6))
                # print(my_read['timestamps'])
                plt.figure(figsize=(40, 6))
                for column in combined_df.columns:
                    # plt.legend()
                    plt.plot(combined_df[column], label=column)
                    plt.legend()
                plt.xlabel('len')
                plt.ylabel('timestamp')
                plt.title('All Camera Time Check')
                plt.savefig(f"{png_directory}/all_camera_Frame.png")
                plt.close()

                camera_names = combined_df.columns.tolist()
                combined_df = combined_df.merge(combined_df[column], left_index=True, right_index=True, how="outer").fillna(0)
                plt.figure(figsize=(40, 20))
                for column in combined_df.columns:
                    plt.bar(range(1, len(camera_names)+1), column ,align='center')
                plt.xlabel("Camera")
                plt.ylabel("Number of Rows")
                plt.title("Comparison of Camera Rows")
                plt.xticks(range(1, len(camera_names)+1), camera_names)

                plt.savefig(f"{png_directory}/6V_camera_Frame.png")
                plt.close()
                
                # camera_names = combined_df.columns.tolist() 
                # my_lens = len(combined_df[column])
                # camera_lengths = [len(combined_df[column]) for column in camera_names]
                # plt.figure(figsize=(40, 20))
                # plt.bar(cs_time, front_120, align='center')
                # plt.xlabel("Camera")
                # plt.ylabel("Number of Rows")
                # plt.title("Comparison of Camera Rows")
                # plt.savefig(f"{png_directory}/6V_camera_Frame.png")
                # plt.close()
                
                
                read1 = my_read['timestamps']
                # print(my_read.head())
                # print(read1)
                my_read['last_two'] = my_read['timestamps'].apply(lambda x: str(x)[11:13])
                cs_time = my_read['last_two']
                front_120 = len(cs_time)
                print(f"注意{topic}有{front_120}行数据")
                niude = np.bincount(cs_time)
                max_count = np.max(niude)
                for i, count in enumerate(niude):
                    if count > 0:
                        print(f"{save_path}下的{topic}周期为 {i}: {count} 次")
                matching_percentage = ((max_count / front_120) * 100) *2
                if matching_percentage == 100.0:
                    print(f"{save_path}下的{topic}时间戳匹配率：100%")
                else:
                    print(f"{save_path}下的{topic}时间戳匹配率：{matching_percentage}%")
                plt.figure(figsize=(40, 6))
                plt.plot(read1, color='red',label='camera_check')
                plt.legend()
                plt.xlabel('len')
                plt.ylabel('timestamp')
                plt.title('camera_time_check')
                plt.savefig(f"{png_directory}/{topic_rename}camera_Frame.png")
                plt.close()
                
                

                
                # plt.figure(figsize=(40, 6))
                # plt.plot(read1, color='red',label='camera_check')
                # plt.legend()
                # plt.xlabel('len')
                # plt.ylabel('timestamp')
                # plt.title('camera_time_check')
                # plt.savefig(f"{save_path}/{topic_rename}camera_timestamps.png")
                
                
                plt.figure(figsize=(40, 6))
                plt.plot(cs_time, color='red',label='50ms')
                plt.legend()
                plt.xlabel('len')
                plt.ylabel('timestamp')
                plt.title('camera_time_check')
                plt.savefig(f"{png_directory}/{topic_rename}camera_cycle.png")
                plt.close()
            else:
                print(f"{topic} not in topics")
            
            
        # my_lidar_topic = "/sensor/lidar/rotation_packet"
        # if my_lidar_topic in topics:
        #     lidar = bag_data.message_by_topic("/sensor/lidar/rotation_packet")
        #     print(lidar)
        #     df_lidar = pd.read_csv(lidar)
        #     timestamps_lidar = df_lidar["header.stamp.secs"]
        #     # print(df_lidar)
        #     df_20hz_interpolated = combined_df[column].interpolate(method='linear', limit_direction='forward', axis=0)
        #     df_10hz = df_20hz_interpolated.iloc[::2].reset_index(drop=True)
        #     plt.figure(figsize=(40, 6))
        #     plt.plot(timestamps_lidar, color='red',label='lidar')
        #     plt.plot(df_10hz, label=column)
        #     plt.legend()
        #     plt.xlabel('len')
        #     plt.ylabel('timestamp')
        #     plt.title('camera_lidar_check')
        #     plt.savefig(f"{png_directory}/{topic_rename}camera_lidar.png")
        # else:
        #     print(f"未找到lidar的topic in {topics}")         
                
        # Gnss_topic = "/sensor/navi/navifusion"
        # for topic in neededTopics:
        #     if Gnss_topic in topic:
        #         topic_rename = topic.replace("/","_")
        #         topic_rename = topic_rename.lstrip("_")
        #         for t,m,s in rosbagData.read_messages(topics=[topic]):
        #             try:
        #                 tsList.append(int(m.status.stamp.to_nsec()))
        #                 print(tsList)
        #             except:
        #                 pass
                
        # for topic in neededTopics:
        #     if topic in topics:
        #         topic_rename = topic.replace("/","_")
        #         topic_rename = topic_rename.lstrip("_")
        #         cmd = f"rostopic echo -b {my_bag} -p {topic}/header > {save_path}/{topic_rename}.csv"
        #         if topic not in skipTopics:
        #             cmds.append(cmd)
                
        # for cmd in cmds:
        #     # subprocess.run(cmd,shell=True) 
        #     print(cmd)

            

            # value1 = content["Front-120"]
            # value2 = content["Vehicle"]
            # value3 = content["Soc_state"]
            # value4 = content["Front_left"]
            # value5 = content["Front_right"]
            # value6 = content["Rear_100"]
            # value7 = content["Rear_left"]
            # value8 = content["Rear_right"]
            # value9 = content["Front-120"]
            # # newvalue = str(value2[0])
            # print("dao wo le ")
            # found = False  #康康能对的上bags中的topic
            # #print("dao wo le ")
            # if any(topic in [value1, value2, value3] for topic in topics):
                
            #     print("\nTopics对上了，正在处理中~")
            #     found = True
            #     front_cmd = f"rostopic echo -b {my_bag} -p {value1}/header > {save_path}/front_camera.csv"
            #     subprocess.run(front_cmd, shell=True)
            #     vs = bag_data.message_by_topic(f"{value2}")
            #     soc = bag_data.message_by_topic(f"{value3}")
            #     print(vs)
            #     print(soc)
                
            # elif any(topic in [value2, value3, value4, value5, value6, value7, value8, value9] for topic in topics):
            #     print("\nTopics对上了，正在处理中～")
            #     found = True
            #     front_cmd2 = f"rostopic echo -b {my_bag} -p {value9}/header > {save_path}/front_camera.csv"
            #     rear_cmd = f"rostopic echo -b {my_bag} -p {value6}/header > {save_path}/rear_camera.csv"
            #     lf_cmd = f"rostopic echo -b {my_bag} -p {value4}/header > {save_path}/front_left_camera.csv"
            #     rf_cmd = f"rostopic echo -b {my_bag} -p {value5}/header > {save_path}/front_right_camera.csv"
            #     lr_cmd = f"rostopic echo -b {my_bag} -p {value7}/header > {save_path}/rear_left_camera.csv"
            #     rr_cmd = f"rostopic echo -b {my_bag} -p {value8}/header > {save_path}/rear_rightcamera.csv"
            #     subprocess.run(front_cmd2,shell=True)
            #     subprocess.run(rear_cmd,shell=True)
            #     subprocess.run(lf_cmd,shell=True)
            #     subprocess.run(rf_cmd,shell=True)
            #     subprocess.run(lr_cmd,shell=True)
            #     subprocess.run(rr_cmd,shell=True)   
            #     vs = bag_data.message_by_topic(f"{value2}")
            #     soc = bag_data.message_by_topic(f"{value3}")
            #     print(vs)
            #     print(soc)

            # else:
            #     print("Errors Get:", topic)    
            # if not found:
            #     print("一个都没找到，请检查Json文件中的值跟topic中的是否对应")
            #     exit()

                    
        # print(topics)
        # vs = bag_data.message_by_topic(f"{value2}")
        # soc = bag_data.message_by_topic(f"{value3}")
        # print(vs)
        # print(soc)
        # #front = bag_data.message_by_topic("/sensor/fdc/camera/front_120_8M/compressed")
        # #front_cmd = f"rostopic echo -b {my_bag} -p {value1}/header > {save_path}/front_camera.csv"
        # front_cmd2 = f"rostopic echo -b {my_bag} -p {value9}/header > {save_path}/front_camera.csv"
        # rear_cmd = f"rostopic echo -b {my_bag} -p {value6}/header > {save_path}/rear_camera.csv"
        # lf_cmd = f"rostopic echo -b {my_bag} -p {value4}/header > {save_path}/front_left_camera.csv"
        # rf_cmd = f"rostopic echo -b {my_bag} -p {value5}/header > {save_path}/front_right_camera.csv"
        # lr_cmd = f"rostopic echo -b {my_bag} -p {value7}/header > {save_path}/rear_left_camera.csv"
        # rr_cmd = f"rostopic echo -b {my_bag} -p {value8}/header > {save_path}/rear_rightcamera.csv"
        
        # #subprocess.run(front_cmd, shell=True)
        # subprocess.run(front_cmd2,shell=True)
        # subprocess.run(rear_cmd,shell=True)
        # subprocess.run(lf_cmd,shell=True)
        # subprocess.run(rf_cmd,shell=True)
        # subprocess.run(lr_cmd,shell=True)
        # subprocess.run(rr_cmd,shell=True)
        # print(vs[0])
        # soc = bag_data.message_by_topic("/iflytek/system_state/soc_state")
        # print(soc)
        # vs_pilot_path = vs[0] 
        # print(vs_pilot_path)
        # csv_files = [f for f in os.listdir(vs_pilot_path) if f.endswith('iflytek-vehicle_service.csv')]
        
        # print(csv_files)
        # timestamps = []
        # stabilities = []

        # for file in csv_files:
        # vs_path = vs
        # soc_path = soc
        # camera_path = f"{save_path}/front_camera.csv"
        # front2_path = f"{save_path}/front_camera.csv" #df 4
        # rear_path = f"{save_path}/rear_camera.csv" #df 5
        # left_front_path = f"{save_path}/front_left_camera.csv" #df 6
        # right_front_path = f"{save_path}/front_right_camera.csv" #df 7
        # left_rear_path = f"{save_path}/rear_right_camera.csv" #df 8
        # right_rear_path = f"{save_path}/rear_rightcamera.csv" #df 9
        
        # df4 = pd.read_csv(front2_path)
        # df5 = pd.read_csv(rear_camera)
        # df6 = pd.read_csv(left_front_path)
        # df7 = pd.read_csv(right_front_path)
        # df8 = pd.read_csv(left_rear_path)
        # df9 = pd.read_csv(right_rear_path)
        
        # df = pd.read_csv(vs_path)
        # df3 = pd.read_csv(soc_path)
        # # df2 = pd.read_csv(camera_path)
        # df2 = pd.read_csv(camera_path)
        # timestamp_column4 = df4['field.stamp']
        # timestamp_column5 = df5['field.header.stamp']
        # timestamp_column6 = df6['field.header.stamp']
        # timestamp_column7 = df7['field.header.stamp']
        # timestamp_column8 = df8['field.header.stamp']
        # timestamp_column9 = df9['field.header.stamp']
        # df9['last_two_digits'] = df9['field.header.stamp'].apply(lambda x: str(x)[11:13])
        # df8['last_two_digits'] = df8['field.header.stamp'].apply(lambda x: str(x)[11:13])
        # df7['last_two_digits'] = df7['field.header.stamp'].apply(lambda x: str(x)[11:13])
        # df6['last_two_digits'] = df6['field.header.stamp'].apply(lambda x: str(x)[11:13])
        # df5['last_two_digits'] = df5['field.header.stamp'].apply(lambda x: str(x)[11:13])
        # df4['last_two_digits'] = df4['field.stamp'].apply(lambda x: str(x)[11:13])
        # df2['last_two_digits'] = df2['field.stamp'].apply(lambda x: str(x)[11:13])
        # timestamp_column = df['header.timestamp']
        # timestamp_column2 = df2['field.stamp']
        # timestamp_column3 = df3['header.timestamp']
        # timestamp_column_test = df2['last_two_digits']
        # timestamp_column_test4 = df4['last_two_digits']
        # timestamp_column_test5 = df5['last_two_digits']
        # timestamp_column_test6 = df6['last_two_digits']
        # timestamp_column_test7 = df7['last_two_digits']
        # timestamp_column_test8 = df8['last_two_digits']
        # timestamp_column_test9 = df9['last_two_digits']
        
        # #统计时间戳跳变逻辑，但是考虑到每个test中的数不同
        # front_120 = len(timestamp_column_test4)
        # print(f"注意front-120有{time_len}行数据")
        # niude4 = np.bincount(timestamp_column_test4)
        # max_count4 = np.max(niude4)
        # for i, count in enumerate(niude4):
        #     if count > 0:
        #         print(f"{save_path}下的front-120周期为 {i}: {count} 次")
        # matching_percentage4 = ((max_count4 / front_120) * 100) *2
        # if matching_percentage4 == 100.0:
        #     print(f"{save_path}front-120时间戳匹配率：100%")
        # else:
        #     print(f"{save_path}front-120时间戳匹配率：{matching_percentage4}%")
        
        # rear_60 = len(timestamp_column_test5)
        # print(f"注意rear-60有{rear_60}行数据")
        # niude5 = np.bincount(timestamp_column_test5)
        # max_count5 = np.max(niude5)
        # for i, count in enumerate(niude5):
        #     if count > 0:
        #         print(f"{save_path}下的rear-60周期为 {i}: {count} 次")
        # matching_percentage5 = ((max_count5 / rear_60) * 100) *2
        # if matching_percentage5 == 100.0:
        #     print(f"{save_path}rear-60时间戳匹配率：100%")
        # else:
        #     print(f"{save_path}rear-60时间戳匹配率：{matching_percentage5}%")
        
        # left_front = len(timestamp_column_test6)
        # print(f"注意left_front有{left_front}行数据")
        # niude6 = np.bincount(timestamp_column_test6)
        # max_count6 = np.max(niude6)
        # for i, count in enumerate(niude6):
        #     if count > 0:
        #         print(f"{save_path}下的left_front周期为 {i}: {count} 次")
        # matching_percentage6 = ((max_count6 / left_front) * 100) *2
        # if matching_percentage5 == 100.0:
        #     print(f"{save_path}left_front时间戳匹配率：100%")
        # else:
        #     print(f"{save_path}left_front时间戳匹配率：{matching_percentage6}%")
        
        # right_front = len(timestamp_column_test7)
        # print(f"注意right_front有{right_front}行数据")
        # niude7 = np.bincount(timestamp_column_test7)
        # max_count7 = np.max(niude7)
        # for i, count in enumerate(niude7):
        #     if count > 0:
        #         print(f"{save_path}下的right_front周期为 {i}: {count} 次")
        # matching_percentage7 = ((max_count7 / right_front) * 100) *2
        # if matching_percentage7 == 100.0:
        #     print(f"{save_path}right_front时间戳匹配率：100%")
        # else:
        #     print(f"{save_path}right_front时间戳匹配率：{matching_percentage7}%")
        
        # left_rear = len(timestamp_column_test8)
        # print(f"注意left_rear有{left_rear}行数据")
        # niude8 = np.bincount(timestamp_column_test8)
        # max_count8 = np.max(niude8)
        # for i, count in enumerate(niude8):
        #     if count > 0:
        #         print(f"{save_path}下的right_front周期为 {i}: {count} 次")
        # matching_percentage8 = ((max_count8 / left_rear) * 100) *2
        # if matching_percentage8 == 100.0:
        #     print(f"{save_path}right_front时间戳匹配率：100%")
        # else:
        #     print(f"{save_path}right_front时间戳匹配率：{matching_percentage8}%")
        
        
        # right_rear = len(timestamp_column9)
        # print(f"注意right_rear有{right_rear}行数据")
        # niude9 = np.bincount(timestamp_column_test9)
        # max_count9 = np.max(niude9)
        # for i, count in enumerate(niude9):
        #     if count > 0:
        #         print(f"{save_path}下的right_front周期为 {i}: {count} 次")
        # matching_percentage9 = ((max_count9 / right_rear) * 100) *2
        # if matching_percentage9 == 100.0:
        #     print(f"{save_path}right_front时间戳匹配率：100%")
        # else:
        #     print(f"{save_path}right_front时间戳匹配率：{matching_percentage9}%")
        
        
        # time_len = len(timestamp_column_test)
        # print(f"注意front-120有{time_len}行数据")
        # niude = np.bincount(timestamp_column_test)
        # max_count = np.max(niude)
        # #print(max_count)
        # #matching_percentage = (max_count / time_len) * 100
        # #print(matching_percentage)
        
        # for i, count in enumerate(niude):
        #     if count > 0:
        #         print(f"{save_path}下的front-120周期为 {i}: {count} 次")
                
        # matching_percentage = ((max_count / time_len) * 100) *2       
        # if matching_percentage == 100.0:
        #     print(f"{save_path}front-120时间戳匹配率：100%")
        # else:
        #     print(f"{save_path}front-120时间戳匹配率：{matching_percentage}%")
        # # for i, count in enumerate(niude):
        # #     if count > 0:
        # #         print(f"相机的周期为 {i}: {count} 次")
                                        
        # is_cycling = False
        # sequence_length = len(timestamp_column_test)
        # #print(f"时间戳的数量为：{sequence_length}")
        # for i in range(sequence_length):
        #     if timestamp_column_test[i%sequence_length] != timestamp_column_test[(i+1)%sequence_length]:
        #         is_cycling = True
        #         break
        #print(timestamp_column_test)
        #print(timestamp_column_test)
        # last_two_digits = timestamp_column_test[11:13]
        # print(last_two_digits) 
        # timestamp_column2 = df2['header.timestamp']
        #print(timestamp_column2)
        
        # plt.figure(figsize=(30, 6))
        # plt.plot(timestamp_column2, color='red',label='front_120_8M')
        # plt.legend()
        # plt.xlabel('len')
        # plt.ylabel('timestamp')
        # plt.title('camera_time_check')
        # plt.savefig(f"{save_path}/camera_timestamps.png")
        
        
        # plt.figure(figsize=(40, 6))
        # plt.plot(timestamp_column_test, color='red',label='50ms')
        # plt.legend()
        # plt.xlabel('len')
        # plt.ylabel('timestamp')
        # plt.title('camera_time_check')
        # plt.savefig(f"{save_path}/camera.png")
        # print(timestamp_column)
        #diff = len(timestamp_column)
        #print(diff)
        # print(diff)
        # timestamps = []
        # stabilities = []
        # print(file_path)
        # for myfile in file_path:
        # print(myfile)
        # timestamps.append(timestamp_column)
        # stabilities.append(diff)
        # plt.figure(figsize=(30, 6))
        
        # plt.plot(timestamp_column, color='blue', label='Vehicle')
        # plt.plot(timestamp_column3, color='red',label='Soc')
        # plt.legend()
        # plt.annotate('Vehicle_time', xy=(x1, y1), xytext=(x_text1, y_text1), arrowprops=dict(arrowstyle='->'))
        # plt.annotate('Soc_time', xy=(x2, y2), xytext=(x_text2, y_text2), arrowprops=dict(arrowstyle='->'))
        # plt.bar(diff, timestamp_column)
        # plt.xlabel('len')
        # plt.ylabel('timestamp')
        # plt.title('Comparison_Chart_check')
        # # plt.grid(True)
        # plt.savefig(f"{save_path}/Comparison_timestamps.png")
        
    # num_bags = len(bags)
    # return num_bags

def muiltiThreadingRun(args):
    bag_path = args.bag_path
    json_path = args.json_path
    bags = find_bags_in_folder(bag_path)
    # niu = len(bags)
    # mybag = str(bags[0:niu])
    # print(mybag)
    
    if len(bags) == 0:
        print(f"给出的路径下没有bag文件！")
        return
    
    pool = multiprocessing.Pool(processes=20)
    for bag_path in bags:
        # print(bag_path)
        pool.apply_async(worker, (bag_path, json_path))
        
    pool.close()
    pool.join()
    
    print("ALL Bags is Done")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='注意要转换包的路径！')
    parser.add_argument('bag_path', metavar='PATH', type=str, help='需要解包的路径：')
    parser.add_argument('json_path',metavar='PATH', type=str, help='Json路径：')
    args = parser.parse_args()
    #worker(args.bag_path,args.json_path)
    # find_dir_in_folder(args.bag_path)
    #muiltiThreadingRun(args)
    # worker(arg.bag_path,args.json_path)
    worker("/data_cold/autoupload/jac_s811_37xu2/common/20240104/20240104-13-49-26/","./panorama.json")
    #find_dir_in_folder(args.bag_path)
    
    
    
    
    
    
    
    
   
#     for topic, msg, t in bag.read_messages(topics=["/sensor/fdc/camera/front_120_8M/compressed"]):
#     # print(msg.header.stamp)
#     tsDict[topic]=[]

# for topic, msg, t in bag.read_messages(topics=["/sensor/fdc/camera/front_120_8M/compressed"]):
#     try:
#         tsDict[topic].append(int(msg.header.stamp.to_nsec()/1e6))
#     except:
#         pass

# bag.close()
# for key,v in tsDict.items():
#     print(key,len(v))

# outputCsvDir = './outputCsv'
# os.makedirs(outputCsvDir,exist_ok=True)
# for key,v in tsDict.items():
#     fileName = key.replace('/','_').lstrip('_')
#     if len(v)>0:
#         with open(os.path.join(outputCsvDir,fileName+'.csv'),'w') as f:
#             # header = ['timestamp']
#             writer = csv.writer(f)
#             # writer.writerow(header)
#             # writer.writerow(v)
#             for vv in v:
#                 writer.writerow([vv])
#             print('write {} done'.format(fileName))

# csvFileList = os.listdir(outputCsvDir)
# for csvFile in csvFileList:
#     if csvFile.endswith('.csv'):
#         pass