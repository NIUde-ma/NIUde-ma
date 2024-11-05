#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import imaplib
import rosbag
import os
import argparse
import logging
import multiprocessing

def find_dir_in_folder(bag_path):
    my_dir = []
    for root, dirs, files in os.walk(bag_path):
        for file in files:
            if file.endswith(".bag"):
                my_dir.append(os.path.join(root))
                print(my_dir)   
    return my_dir

def find_bags_in_folder(path):
    bags = []
    
    if not os.path.exists(path):
        logging.error("path not exist: %s", path)
        return bags
    
    if os.path.isfile(path):
        if path.endswith(".bag"):
            bags.append(path)
        
    else:
        for root, dirs, files in os.walk(path):
            for file in files:
                if file.endswith(".bag"):
                    bags.append(os.path.join(root, file))
                
    logging.info("found %d bag files in %s", len(bags), path)
    
    return bags

def msg_get(bag_path):
    camTopics = [
                    "/sensor/fdc/camera/front_120_8M",
                    "/sensor/fdc/camera/front_left_100_2M",
                    "/sensor/fdc/camera/front_right_100_2M",
                    "/sensor/fdc/camera/rear_100_2M",
                    "/sensor/fdc/camera/rear_left_100_2M",
                    "/sensor/fdc/camera/rear_right_100_2M",
                    "/iflytek/vehicle_service",
                    "/sensor/lidar/rotation_packet",
                    "/sensor/navi/navifusion"
                ]
    
    bag_file = find_bags_in_folder(bag_path)

    print(bag_file)
    processed_paths = []
    for path in bag_file:
        processed_paths.append(path)
        
    for path in processed_paths:
        bag = rosbag.Bag(path, "r")
    # bag_data = bag.read_messages(camTopics) 
    # for topic, msg, t in bag_data:
    
    #     print(t)
        bag_nums = bag.get_type_and_topic_info(camTopics)
        topics_data = bag_nums.topics
        duration_time = bag.get_end_time() - bag.get_start_time()
        print(f"\n正在处理的bag为：{path}\n Bag_times:{duration_time}\n")
        # print(f"Bag_times: {duration}")
        
        for topic, data in topics_data.items():
            print(f"Topic_name: {topic}")
            print(f"Message_counts: {data.message_count}\n")


        # for topic, msg, t in bag_data:
        #     print(t)


def multiprocessRUN(args):
    bag_path = args.bag_path
    bags = find_bags_in_folder(bag_path)
    
    if len(bags) == 0:
        print(f"给出的路径下没有bag文件！")
        return
    
    pool = multiprocessing.Pool(processes=20)
    
    for bag_path in bags:
        # print(bag_path)
        pool.apply_async(msg_get, (bag_path, ))

    pool.close()
    pool.join()

    print("ALL Bags is Done")  
    
if __name__ == "__main__":   
    parser = argparse.ArgumentParser(description='注意要转换包的路径！')
    parser.add_argument('bag_path', metavar='PATH', type=str, help='需要解包的路径：')
    args = parser.parse_args()
    multiprocessRUN(args)
    # bag_path = args.bag_path
    # msg_get(bag_path)