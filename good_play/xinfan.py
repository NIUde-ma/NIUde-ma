#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import requests
import time
from fake_useragent import UserAgent
from datetime import datetime
import json

def get_json_():
    
    with open("/root/Downloads/my_py/food_work/food.json",'r',encoding='utf-8') as f:
        data = json.load(f)
        a2_url_local_am = data.get("a2_food_url_am", "Unknown")
        a2_url_local_pm = data.get("a2_food_url_pm", "Unknown")
        a4_url_local_am = data.get("a4_food_url_am", "Unknown")
        a4_url_local_pm = data.get("a4_food_url_pm", "Unknown")
        yuanshen = data.get("kunkun_yuanshen", "Unknown")
        fan = data.get("kunkun_fan", "Unknown")
        kunming = data.get("kunkun_jiming", "Unknown")
        Cookies = data.get("Cookie","Unknown")
        return a2_url_local_am, a2_url_local_pm, a4_url_local_am, a4_url_local_pm, yuanshen, fan, kunming, Cookies

def get_local():
    currentDateAndTime = datetime.now()
    times = int(currentDateAndTime.strftime("%H"))+8
    print("local_time is :",times)
    month_and_day = currentDateAndTime.strftime("%m-%d")
    return times,month_and_day

def get_Food_a2(times,month_and_day,a2_url_local_am,a2_url_local_pm, Cookies):
    if times <= 12:
        date = "AM"
        a2_url = a2_url_local_am
    else:
        date = "PM"
        a2_url = a2_url_local_pm

    ua = UserAgent()
    
    headers = {
        "User-Agent": ua.random,
        "Accept": "application/json, text/javascript, */*; q=0.01",
        "Accept-Encoding": "gzip, deflate, br, zstd",
        "Connection": "keep-alive",
        "Cookie": Cookies ,
    }

    response = requests.get(a2_url, headers=headers)

    if response.status_code != 200:
        print("请求失败！")
        exit(1)
    else:
        print("请求成功！",response.status_code)
        page_content = response.json()
        matched_content = []
        
        for item in page_content["content"]["list"]:
            if month_and_day in item:
                matched_content.append({
                    "lineName": item["lineName"],
                    "menu": item[month_and_day]
                })
        
        message_a2 = {
            "msg_type": "text",
            "content": {
                "text": (
                f"<at user_id=\"all\">所有人</at> 干饭王给你带来--A2--{date}--食堂的菜单~\n"
                f"今日菜单 ({month_and_day}):\n")
            }
        }
        
        for content in matched_content:
            message_a2["content"]["text"] += f"餐线: {content['lineName']}\n菜单: {content['menu']}\n\n"
        
        return message_a2

    response_a2 = requests.get(a2_url, headers=headers)
    
    if response_a2.status_code != 200:
        print("请求 A2 餐厅菜单失败！")
        exit(1)
    else:
        page_content_a2 = response_a2.json()
        matched_content_a2 = []
        
        for item in page_content_a2["content"]["list"]:
            if month_and_day in item:
                matched_content_a2.append({
                    "lineName": item["lineName"],
                    "menu": item[month_and_day]
                })
        
        message_a2 = {
            "msg_type": "text",
            "content": {
                "text": f"<at user_id=\"all\">所有人</at> 干饭王给你带来--A2--{date}--食堂的菜单~\n"
            }
        }
        
        for content in matched_content_a2:
            message_a2["content"]["text"] += f"餐线: {content['lineName']}\n菜单: {content['menu']}\n\n"
    return message_a2

def get_Food_a4(times,month_and_day,a4_url_local_am, a4_url_local_pm, Cookies):
    if times <= 12:
        date = "AM"
        a4_url = a4_url_local_am
    else:
        date = "PM"
        a4_url = a4_url_local_pm

    ua = UserAgent()
    
    headers = {
        "User-Agent": ua.random,
        "Accept": "application/json, text/javascript, */*; q=0.01",
        "Accept-Encoding": "gzip, deflate, br, zstd",
        "Connection": "keep-alive",
        "Cookie": Cookies ,
    }

    response = requests.get(a4_url, headers=headers)

    if response.status_code != 200:
        print("请求失败！")
        exit(1)
    else:
        print("请求成功！",response.status_code)
        page_content = response.json()
        matched_content = []
        
        for item in page_content["content"]["list"]:
            if month_and_day in item:
                matched_content.append({
                    "lineName": item["lineName"],
                    "menu": item[month_and_day]
                })
        
        message_a4 = {
            "msg_type": "text",
            "content": {
                "text": (
                f"<at user_id=\"all\">所有人</at> 干饭王给你带来--A4--{date}--食堂的菜单~\n"
                f"今日菜单 ({month_and_day}):\n")
            }
        }
        
        for content in matched_content:
            message_a4["content"]["text"] += f"餐线: {content['lineName']}\n菜单: {content['menu']}\n\n"
        
        return message_a4
    


def send_kunkun(food, kunming):
    send_url = kunming
    response_a2 = requests.post(send_url, json=food_a2)
    
    if response_a2.status_code == 200:
        result = response_a2.json()
        print("机器人回复:", result)
    else:
        print("请求失败:", response_a2.status_code, response_a2.text)

def send_kunkun2(food, kunming):
    send_url = kunming
    response_a4 = requests.post(send_url, json=food_a4)
    
    if response_a4.status_code == 200:
        result = response_a4.json()
        print("机器人回复:", result)
    else:
        print("请求失败:", response_a4.status_code, response_a4.text)

if __name__ == '__main__':
    times , month_and_day = get_local()
    a2_url_local_am, a2_url_local_pm, a4_url_local_am, a4_url_local_pm, yuanshen, fan, kunming, Cookies = get_json_()
    food_a2 = get_Food_a2(times, month_and_day,a2_url_local_am, a2_url_local_pm, Cookies)
    food_a4 = get_Food_a4(times, month_and_day,a4_url_local_am, a4_url_local_pm, Cookies)
    send_kunkun(food_a2,kunming)
    time.sleep(3)
    send_kunkun2(food_a4,kunming)