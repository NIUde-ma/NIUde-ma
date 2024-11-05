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
        # niude_path = os.getcwd(save_path)
        # print(f"MY PATH IS {niude_path}")
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

        
        if  "/iflytek/vehicle_service" in topics:
            vs = bag_data.message_by_topic("/iflytek/vehicle_service")
            print(vs)
            vs_path = vs
            niude_path = os.path.dirname(vs_path)
            # print(f"MY PATH IS {niude_path}")
            df = pd.read_csv(vs_path)
            timestamp_column = df['header.timestamp']
            plt.figure(figsize=(30, 6))
            plt.plot(timestamp_column, color='blue', label='Vehicle')
            plt.legend()
            plt.ylabel('timestamp')
            plt.title('Comparison_Chart_check')
            plt.savefig(f"{niude_path}/Vehicle_timestamps.png")
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
            plt.savefig(f"{niude_path}/Soc_state_timestamps.png")
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
            plt.savefig(f"{niude_path}/Gnss_status.png")
        
            plt.figure(figsize=(30,6))
            plt.plot(timestamp_column4,color='red',label='Us_timestamps')
            plt.legend()
            plt.ylabel('timestamps_check')
            plt.title('Gnss_timestamps_Us_check')
            plt.savefig(f"{niude_path}/Gnss_timestamps.png")
        else:
            print(f"/sensor/navi/navifusion，在{topics}中为找到！")
        
        if "/iflytek/vehicle_service" and "/iflytek/system_state/soc_state" in topics:
            plt.figure(figsize=(30, 6))
            plt.plot(timestamp_column, color='blue', label='Vehicle')
            plt.plot(timestamp_column3, color='red',label='Soc_state')
            plt.legend()
            plt.xlabel('len')
            plt.ylabel('timestamp')
            plt.title('Comparison_Chart_check')
            plt.savefig(f"{niude_path}/Comparison_timestamps.png")
        else:
            print(f"融合图无法生成，请检查{topics}中是否存在vehicle、Soc_state")    
      
        
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
                        tsList.append(int(m.header.stamp.to_nsec()))
                        # print(tsList)
                    except:
                        pass
            

                csvFilePath = f"{niude_path}/{topic_rename}.csv"
                
                
                if len(tsList)>0:
                    with open(csvFilePath,'w') as f:
                        writer = csv.writer(f)
                        writer.writerow(['timestamps'])
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
                if len(my_read['timestamps']) > 0:
                    if combined_df.empty:
                        # my_read['timestamps'] = combined_df['timestamps']
                        combined_df['timestamps'] = my_read['timestamps']
                        
                        # print(combined_df['timestamps'])
                    else:
                        combined_df[topic_rename] = my_read['timestamps']
                else:
                    print(f"failed to read {topic_rename} CSV file!")
                
                plt.figure(figsize=(40, 6))
                for column in combined_df.columns:
                    plt.plot(combined_df[column], label=column)
                
                plt.legend()
                plt.xlabel('len')
                plt.ylabel('timestamp')
                plt.title('All Camera Time Check')
                plt.savefig(f"{niude_path}/all_camera_clock_times.png")
                plt.close()

                read1 = my_read['timestamps']
                # print(my_read.head())
                # print(read1)
                my_read['last_two'] = my_read['timestamps'].apply(lambda x: str(x)[11:13])
                cs_time = my_read['last_two']
                front_120 = len(cs_time)

                modified_topic = topic.replace("/", "_")  # 将斜杠替换为下划线或其他合法字符
                plt.figure(figsize=(20, 6))
                plt.bar(topic, front_120, align='center')
                plt.xlabel("Camera")
                plt.ylabel("Number of Rows")
                plt.title("Comparison of Camera Rows")
                plt.savefig(f"{niude_path}/{modified_topic}_Frame.png")
                plt.close()
                
                
                
                print(f"注意{topic}有{front_120}行数据")
                
                plt.figure(figsize=(40, 6))
                for i, column in enumerate(combined_df.columns):
                    num_rows = len(combined_df[column])
                    plt.bar(column, num_rows, align='center')
                    plt.text(i, num_rows, str(num_rows), ha='center', va='bottom')
                plt.xlabel("Camera")
                plt.ylabel("Number of Rows")
                plt.title("Comparison of Camera Rows")
                plt.savefig(f"{niude_path}/Six_camera_frames.png")
                plt.close()              

                
                # plt.figure(figsize=(20, 6))
                # for t, f in zip(topic, front_120):
                #     plt.bar(t, f, align='center')
                #     plt.text(t, f, str(f), ha='center', va='bottom')
                # plt.xlabel("Camera")
                # plt.ylabel("Number of Rows")
                # plt.title("Comparison of Camera Rows")
                # plt.savefig(f"{niude_path}/combined_Frame.png")
                # plt.close()
                
                # fig, axes = plt.subplots(nrows=2, ncols=3, figsize=(20, 12))
                # for ax, t, f in zip(axes.flatten(), topic, front_120):
                #     ax.bar(t, f, align='center')
                #     ax.text(t, f, str(f), ha='center', va='bottom')
                #     ax.set_xlabel("Camera")
                #     ax.set_ylabel("Number of Rows")
                #     ax.set_title(t)
                # plt.tight_layout()
                # plt.savefig(f"{niude_path}/combined_frame.png")
                # plt.close()
                # plt.figure(figsize=(40, 6))
                # for i in range(len(topic)):
                #     plt.bar(topic[i], front_120, align='center')
                # plt.xlabel("Camera")
                # plt.ylabel("Number of Rows")
                # plt.title("Comparison of Camera Rows")
                # plt.savefig(f"{niude_path}/6V_camera_Frame.png")
                # plt.close()
                
                niude = np.bincount(cs_time)
                max_count = np.max(niude)
                for i, count in enumerate(niude):
                    if count > 0:
                        print(f"{niude_path}下的{topic}周期为 {i}: {count} 次")
                matching_percentage = ((max_count / front_120) * 100) *2
                if matching_percentage == 100.0:
                    print(f"{niude_path}下的{topic}时间戳匹配率：100%")
                else:
                    print(f"{niude_path}下的{topic}时间戳匹配率：{matching_percentage}%")
                plt.figure(figsize=(40, 6))
                plt.plot(read1, color='red',label='camera_check')
                plt.legend()
                plt.xlabel('len')
                plt.ylabel('timestamp')
                plt.title('camera_time_check')
                plt.savefig(f"{niude_path}/{topic_rename}camera_Frame.png")
                
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
                plt.savefig(f"{niude_path}/{topic_rename}camera_cycle.png")
            else:
                print(f"{topic} not in topics")
                
            
            
            # fig, ax = plt.subplots(figsize=(20, 6))  # 创建一个图形和一个坐标轴
            # ax.bar(topic, front_120)  # 绘制条形图
            # ax.set_xlabel("Camera")  # 设置x轴标签
            # ax.set_ylabel("Number of Rows")  # 设置y轴标签
            # ax.set_title("Comparison of Camera Rows")  # 设置图表标题
            # plt.savefig(f"{niude_path}/combined_Frame.png")  # 保存图表
            # plt.close()
            
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
    # worker(args.bag_path,args.json_path)
    # find_dir_in_folder(args.bag_path)
    muiltiThreadingRun(args)
    # worker(arg.bag_path,args.json_path)
    # worker("/data_cold/autoupload/jac_s811_23gc9/trigger/20231209/20231209-19-47-10","./panorama.json")
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