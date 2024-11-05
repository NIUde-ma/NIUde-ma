#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import os
import sys
import subprocess
import argparse
import time
import rosbag
import rospy
import multiprocessing
import logging

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logging.basicConfig(level=logging.WARNING, format='%(asctime)s - %(levelname)s - %(message)s')

class CHECK:
    def __init__(self):
        self.topics = []
        self.bag_dir = []
        self.returns = 0
        
        self.scc_need_topics = ["/sensor/fdc/camera/front_120_8M",
                                "/sensor/fdc/camera/front_left_100_2M",
                                "/sensor/fdc/camera/front_right_100_2M",
                                "/sensor/fdc/camera/rear_left_100_2M",
                                "/sensor/fdc/camera/rear_right_100_2M",
                                "/sensor/lidar/rotation_packet",
                                "/sensor/navi/navifusion",
                                "/iflytek/vehicle_service",
                                "/iflytek/uss/usswave_info",
                                "/iflytek/system/version",
                                "/iflytek/system/diagnosis",
                                "/iflytek/sensor/imu",
                                "/iflytek/sensor/gnss",
                                "/iflytek/sdmap/sdmap_info",
                                "/iflytek/radar_rr_perception_info",
                                "/iflytek/radar_rl_perception_info",
                                "/iflytek/radar_fm_perception_info",
                                "/iflytek/planning/plan",
                                "/iflytek/planning/hmi",
                                "/iflytek/planning/debug_info",
                                "/iflytek/other/time_synchronization",
                                "/iflytek/localization/egomotion",
                                "/iflytek/hmi/mcu_to_soc_bsw/chery_e0y_mdc510",
                                "/iflytek/hmi/soc_to_mcu_bsw/chery_e0y_mdc510",
                                "/iflytek/hmi/inner",
                                "/iflytek/fusion/road_fusion_compress",
                                "/iflytek/fusion/road_fusion",
                                "/iflytek/fusion/occupancy/objects",
                                "/iflytek/fusion/objects_compress",
                                "/iflytek/fusion/objects",
                                "/iflytek/fsm/soc_state",
                                "/iflytek/fsm/hmi_soc_outer",
                                "/iflytek/ehr/sdmap_info",
                                "/iflytek/ehr/debuginfo",
                                "/iflytek/control/debug_info",
                                "/iflytek/control/control_command",
                                "/iflytek/camera_perception/traffic_sign_recognition",
                                "/iflytek/camera_perception/objects",
                                "/iflytek/camera_perception/lane_topo_debug_info",
                                "/iflytek/camera_perception/lane_topo",
                                "/iflytek/camera_perception/lane_lines_debug_info",
                                "/iflytek/camera_perception/lane_lines",
                                "/iflytek/alarm_info/fm_service",
                                "/iflytek/alarm_info/fm_b_service",
                                "/iflytek/alarm_info/fm_a_service",
                                "/iflytek/adas_function/debug_info",
                                "/iflytek/adas_function/adas"]
        
        self.apa_need_topics = ["/iflytek/adas_function/adas",
                                "/iflytek/adas_function/debug_info",
                                "/iflytek/alarm_info/fm_a_service",
                                "/iflytek/alarm_info/fm_b_service",
                                "/iflytek/alarm_info/fm_service",
                                "/iflytek/camera_perception/3d_general_objects",
                                "/iflytek/camera_perception/3d_occupancy_objects",
                                "/iflytek/camera_perception/deceler",
                                "/iflytek/camera_perception/ground_line",
                                "/iflytek/camera_perception/occupancy_objects",
                                "/iflytek/camera_perception/parking_slot_list",
                                "/iflytek/control/control_command",
                                "/iflytek/control/debug_info",
                                "/iflytek/fsm/hmi_soc_outer",
                                "/iflytek/fsm/soc_state",
                                "/iflytek/fusion/objects",
                                "/iflytek/fusion/objects_compress",
                                "/iflytek/fusion/occupancy/objects",
                                "/iflytek/fusion/parking_slot",
                                "/iflytek/hmi/inner",
                                "/iflytek/hmi/mcu_to_soc_bsw/chery_e0y_mdc510",
                                "/iflytek/hmi/soc_to_mcu_bsw/chery_e0y_mdc510",
                                # "/iflytek/localization/ego_pose",
                                "/iflytek/localization/egomotion",
                                "/iflytek/mega/local_map",
                                "/iflytek/other/time_synchronization",
                                "/iflytek/planning/debug_info",
                                "/iflytek/planning/hmi",
                                "/iflytek/planning/plan",
                                "/iflytek/sensor/gnss",
                                "/iflytek/sensor/imu",
                                "/iflytek/system/diagnosis",
                                "/iflytek/system/version",
                                "/iflytek/uss/uss_perception_info",
                                "/iflytek/uss/usswave_info",
                                "/iflytek/vehicle_service",
                                "/sensor/fdc/camera/surround_front_190_1M",
                                "/sensor/fdc/camera/surround_left_190_1M",
                                "/sensor/fdc/camera/surround_rear_190_1M",
                                "/sensor/fdc/camera/surround_right_190_1M",
                                "/sensor/lidar/rotation_packet",
                                "/sensor/proto/vision_slot_image",
                                "/sensor/proto/vision_slot_image/compressed"]
        
        self.all_need_topics = ["/sensor/fdc/camera/front_120_8M",
                                "/sensor/fdc/camera/front_left_100_2M",              
                                "/sensor/fdc/camera/front_right_100_2M",
                                "/sensor/fdc/camera/rear_left_100_2M",              
                                "/sensor/fdc/camera/rear_right_100_2M",
                                "/sensor/fdc/camera/surround_front_190_1M",
                                "/sensor/fdc/camera/surround_left_190_1M",
                                "/sensor/fdc/camera/surround_rear_190_1M",
                                "/sensor/fdc/camera/surround_right_190_1M",
                                "/sensor/proto/vision_slot_image",
                                "/sensor/proto/vision_slot_image/compressed",
                                "/iflytek/camera_perception/lane_lines",
                                "/iflytek/camera_perception/lane_lines_debug_info",
                                "/iflytek/camera_perception/lane_topo",
                                "/iflytek/camera_perception/lane_topo_debug_info",
                                "/iflytek/camera_perception/objects",
                                "/iflytek/camera_perception/traffic_sign_recognition"
                                ]
        
    def get_bags(self, path):
        self.bag_dir = []
        for root, dirs, files in os.walk(path):
            for file in files:
                if file.endswith(".bag") and file.split("_")[-2] != "no":
                    self.bag_dir.append(os.path.join(root, file))
                    # print(f"Found bag file: {os.path.join(root, file)}")
                    if self.bag_dir is not None:
                        print(f"Found bag file: {os.path.join(root, file)}")
                    else:
                        print("No bag file found, please check your input")
                        exit(1)
        return self.bag_dir
    
    def select_module_for_topics(self, bag_file, need):
        if bag_file is not None:
            bag = rosbag.Bag(bag_file)
            topic_msg_count = {topic: 0 for topic in need}
            nneed_topics = []
            topics = need 
            for topic, msg, t in bag.read_messages(topics=topics):
                if topic in topic_msg_count:
                    nneed_topics.append(topic)
                    if msg is not None:
                        topic_msg_count[topic] += 1
                    else:
                        print(f"{topic} msg is empty")
            
            missing_topics = []
            empty_topics = []
            
            for need_topic in need:
                if need_topic not in nneed_topics:
                    print(f"Not found topic: {need_topic}")
                    missing_topics.append(need_topic)
                else:
                    print(f"Matched topic: {need_topic} , msg count : {topic_msg_count[need_topic]}")
                    if topic_msg_count[need_topic] == 0:
                        print(f"Warning: Topic {need_topic} has empty messages")
                        empty_topics.append(need_topic)

            bag.close()

            if missing_topics:
                print(f"Error: Missing topics in {bag_file}: {missing_topics}")
                return 1
            elif empty_topics:
                print(f"Warning: Empty messages in topics in {bag_file}: {empty_topics}")
                return 2
            else:
                print(f"All required topics found in {bag_file}")
                return 0
        else:
            print("No bag file found, please check your input")
            return 1
    

    
    # def get_scc_topics(self, bag_file):
    #     if bag_file is not None:
    #         bag = rosbag.Bag(bag_file)
    #         topic_msg_count = {topic: 0 for topic in self.scc_need_topics}
              
    #         topics = self.scc_need_topics
            
    #         nneed_topics = []
            
    #         for topic, msg, t in bag.read_messages(topics=topics):
    #             if topic in topic_msg_count:
    #                 nneed_topics.append(topic)
    #                 if msg is not None:
    #                     topic_msg_count[topic] += 1
    #                 else:
    #                     print(f"{topic} msg is empty")
                        
    #         missing_topics = []
    #         empty_topics = []

    #         for need_topic in self.scc_need_topics:
    #             if need_topic not in nneed_topics:
    #                 print(f"Not found topic: {need_topic}")
    #                 missing_topics.append(need_topic)
    #             else:
    #                 print(f"Matched topic: {need_topic}")
    #                 if topic_msg_count[need_topic] == 0:
    #                     print(f"Warning: Topic {need_topic} has empty messages")
    #                     empty_topics.append(need_topic)
                    

    #         bag.close()

    #         if missing_topics:
    #             print(f"Error: Missing topics in {bag_file}: {missing_topics}")
    #             return 1
    #         elif empty_topics:
    #             print(f"Warning: Empty messages in topics in {bag_file}: {empty_topics}")
    #             return 2
    #         else:
    #             print(f"All required topics found in {bag_file}")
    #             return 0
    #     else:
    #         print("No bag file found, please check your input")
    #         return 1

def process_bag_file(ck, bag_file, module):
    print(f"Processing bag file: {bag_file}")
    
    if module == "SCC":
        need = ck.scc_need_topics
    elif module == "APA":
        need = ck.apa_need_topics
    elif module == "ALL":
        need = ck.all_need_topics
    else:
        print(f"Invalid module: {module}")
        return

    result = ck.select_module_for_topics(bag_file, need)
    if result == 0:
        print(f"{bag_file} is check done!")
    else:
        print(f"{bag_file} is check Error!")

def multiprocessRUN(bag_path, module):
    ck = CHECK()
    bag_files = ck.get_bags(bag_path)
    cpu_count = multiprocessing.cpu_count()
    
    if cpu_count < 20:
        logging.warning(f"你的CPU只有{cpu_count}个线程")
        pool = multiprocessing.Pool(processes=max(6, cpu_count - 6))
    else:
        logging.info(f"您的CPU有{cpu_count}个线程，已经击败全国80%的选手！")
        pool = multiprocessing.Pool(processes=cpu_count - 2)
    
    try:
        for bag_file in bag_files:
            if bag_file is None:
                print("please check you input")
                exit(1)
            else:
                pool.apply_async(process_bag_file, args=(ck, bag_file, module))
        
        pool.close()
        pool.join()
        
    except Exception as e:
        logging.error(f"An error occurred: {e}")
        pool.terminate()
        pool.join()
        
    except KeyboardInterrupt:
        pool.terminate()
        pool.join()
        logging.warning("code exit by user, closeing...")
        
if __name__ == "__main__":
    # multiprocessing.freeze_support()
    parser = argparse.ArgumentParser(description='注意要转换包的路径！')
    parser.add_argument('bag_path', metavar='PATH', type=str, help='需要解包的路径：')
    parser.add_argument('module', metavar='MODULE', type=str, help='需要查看的功能：SCC|APA|ALL')
    args = parser.parse_args()
    try:
        # ck = CHECK()
        # process_bag_file(ck, args.bag_path, args.module)
        multiprocessRUN(args.bag_path, args.module)
    except KeyboardInterrupt:
        logging.info("code exit by user, closeing...")
