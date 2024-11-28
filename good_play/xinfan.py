#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import requests
import time
import json
from datetime import datetime
from fake_useragent import UserAgent
from bs4 import BeautifulSoup


# def get_cookie(login_urls):
#     ua = UserAgent()
    
#     headers = {
#         "User-Agent": ua.random,
#         "Accept": "text/html,application/xhtml+xml,application/xml;q=0.9,*/*;q=0.8",
#         "Accept-Encoding": "gzip, deflate, br, zstd",
#         "Accept-Language":"en-US,en;q=0.5",
#         # "User-Agent":"Mozilla/5.0 (X11; Ubuntu; Linux x86_64; rv:132.0) Gecko/20100101 Firefox/132.0",
#         "Connection": "keep-alive",
#         "Cookie":"SESSION=bd3b7c41-e39c-41fc-8aad-671c00358a6c"
#     }

#     session = requests.Session()
#     # session.headers.update(headers)

#     login_data = {
#         "login": "wjjiang10",
#         "password": "Jieshao520"
#     }
    
#     # response = session.get(login_urls, headers=headers, data=login_data, allow_redirects=False)
    

#     # if response.status_code != 200:
#     #     print("Login failed",response.status_code)
#     #     return None
    
#     redirect_url = "https://ipark.iflytek.com/portal-web/pc/menu/food/weekMenu"
#     final_response = session.get(redirect_url,headers=headers, data=login_data, allow_redirects=False)
#     print(final_response.text)
#     cookies = session.cookies.get_dict()
    
#     idd = cookies.get('JSESSIONID')
#     print(session.cookies)
#     print(idd)
#     return idd

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
        login_urls = data.get("test_url","Unknown")
        return a2_url_local_am, a2_url_local_pm, a4_url_local_am, a4_url_local_pm, yuanshen, fan, kunming, Cookies, login_urls

def get_local():
    currentDateAndTime = datetime.now()
    times = int(currentDateAndTime.strftime("%H"))+8
    print("local_time is :",times)
    month_and_day = currentDateAndTime.strftime("%m-%d")
    return times,month_and_day

def get_Food_a2(times,month_and_day,a2_url_local_am,a2_url_local_pm, Cookies):
    # passwds = "JSESSIONID=" + idd
    # print(passwds)
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
    # passwds = "JSESSIONID=" + idd

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
    


def send_kunkun(food, fan):
    send_url = kunming
    response_a2 = requests.post(send_url, json=food_a2)
    
    if response_a2.status_code == 200:
        result = response_a2.json()
        print("机器人回复:", result)
    else:
        print("请求失败:", response_a2.status_code, response_a2.text)

def send_kunkun2(food, fan):
    send_url = kunming
    response_a4 = requests.post(send_url, json=food_a4)
    
    if response_a4.status_code == 200:
        result = response_a4.json()
        print("机器人回复:", result)
    else:
        print("请求失败:", response_a4.status_code, response_a4.text)

if __name__ == '__main__':
    times , month_and_day = get_local()
    a2_url_local_am, a2_url_local_pm, a4_url_local_am, a4_url_local_pm, yuanshen, fan, kunming, Cookies, login_urls = get_json_()
    # idd = get_cookie(login_urls)
    food_a2 = get_Food_a2(times, month_and_day,a2_url_local_am, a2_url_local_pm, Cookies)
    food_a4 = get_Food_a4(times, month_and_day,a4_url_local_am, a4_url_local_pm, Cookies)
    send_kunkun(food_a2,fan)
    time.sleep(3)
    send_kunkun2(food_a4,fan)
    # get_cookie(login_urls)