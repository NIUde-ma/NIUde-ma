#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import argparse
import json
import pandas as pd
import bagpy
from bagpy import bagreader
import os

def read_json(bag_path,json_path):
    bags = []
    for root, dirs, files in os.walk(bag_path):
        for file in files:
            if file.endswith(".bag"):
                bags.append(os.path.join(root, file))
                my_bag = str(bags[0])
                # print(bags)
                # print(my_bag)
                bag_data = bagreader(str(bags[0]))
                topics = bag_data.topic_table['Topics']
                print(topics)
                
    with open(f"{json_path}", "r", encoding="utf-8") as f: 
        content = json.load(f)
        
        value1 = content["Front-120"]
        value2 = content["Rear-60"]
        
        for topic in topics:
            if value1 == topic:
                print(1)
            elif value2 == topic:
                print(2)
                
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='注意要转换包的路径！')
    parser.add_argument('bag_path', metavar='PATH', type=str, help='需要解包的路径：')
    parser.add_argument('json_path',metavar='PATH', type=str, help='Json路径：')
    args = parser.parse_args()
    read_json(args.bag_path,args.json_path)