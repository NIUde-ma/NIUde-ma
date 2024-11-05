#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import sys
import rosbag
import subprocess

def check_topic_in_rosbag(bag_file, topic):
    # 使用subprocess.run运行rosbag命令
    try:
        result = subprocess.run(['rosbag', 'info', bag_file], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        if topic in result.stdout:
            return True  # topic存在
        else:
            return False  # topic不存在
    except subprocess.CalledProcessError as e:
        print(f"Error running rosbag info: {e}")
        return False

def check_topics_in_bag(bag_file_path, topics):
    """
    检查rosbag文件中是否存在指定的topics
    :param bag_file_path: rosbag文件路径
    :param topics: 要检查的topics列表
    :return: 存在的topics列表
    """
    topics_found = []
    with rosbag.Bag(bag_file_path, 'r') as bag:
        for topic, msg, t in bag.read_messages():
            topics_found.append(topic)
    return topics_found

# 使用例子
if len(sys.argv) != 2:
    print("Usage: python script.py <bag_file_path>")
    sys.exit(1)

bag_file_path = sys.argv[1]
topics_to_check = [
    '/iflytek/fusion/objects',
    '/iflytek/fusion/road_fusion',
    '/sensor/fdc/camera/front_30_8M',
    '/sensor/fdc/camera/front_120_8M/compressed'
]

found_topics = check_topics_in_bag(bag_file_path, topics_to_check)
print(f"在 {bag_file_path} 中找到的topics: {found_topics}")

missing_topics = set(topics_to_check) - set(found_topics)
if missing_topics:
    print(f"{bag_file_path} 丢失的topics: {missing_topics}")
else:
    print("所有指定的topics都存在")
