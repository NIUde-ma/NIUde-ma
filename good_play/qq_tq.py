#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import requests
from fake_useragent import UserAgent
import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
import time
from datetime import datetime
import json

def get_json_():
    with open("/root/Downloads/work/local.json",'r',encoding='utf-8') as f:
        data = json.load(f)
        from_email = data.get("from_email", "Unknown")
        weather_key = data.get("weather_key", "Unknown")
        email_passwd = data.get("email_passwd", "Unknown")
        get_location_key = data.get("get_location_key", "Unknown")
        return from_email, weather_key, email_passwd, get_location_key
        # print(f"{from_email},{weather_key},{email_passwd},{get_location_key}")

def get_current_location(get_location_key):
    ua = UserAgent()
    headers = {
        "User-Agent": ua.random
    }
    url = f"https://ipinfo.io/json?token={get_location_key}"
    
    response = requests.get(url,headers=headers)

    if response.status_code == 200:
        data = response.json()
        city = data.get("city", "Unknown")
        region = data.get("region", "Unknown")
        country = data.get("country", "Unknown")
        location = data.get("loc", "Unknown")
        
        print(f"城市: {city}")
        print(f"地区: {region}")
        print(f"国家: {country}")
        print(f"地理位置: {location}")
    else:
        print(f"Failed to retrieve location data. Status code: {response.status_code}")


def get_weather(weather_key):
    ua = UserAgent()

    url = f"https://api.weatherstack.com/current?access_key={weather_key}&query=hefei"

    headers = {
        "User-Agent": ua.random
    }

    response = requests.get(url, headers=headers)

    if response.status_code == 200:
        weather_data = response.json()
        location = weather_data["location"]["name"]
        temperature = weather_data["current"]["temperature"]
        
        if temperature <= 10:
            niude = "孩子多穿点衣服，别被冻死了！"
            
        elif temperature <= 15:
            niude = "别浪再穿件卫衣，别Tm感冒了！"
        elif temperature >= 24:
            niude = "我滴孩，天气也忒热了，短袖即可！"
        else:
            niude = "温度适宜,穿两件即可，再多就伤身了哦~"
            
        weather_description = weather_data["current"]["weather_descriptions"][0]
        humidity = weather_data["current"]["humidity"]
        wind_speed = weather_data["current"]["wind_speed"]
        
        ### data ###
        now = datetime.now()
        weekday = now.weekday()
        weekday_names = ["星期一", "星期二", "星期三", "星期四", "星期五", "星期六", "星期日"]

        weather_info = (
            f"当前日期: {now.strftime('%Y-%m-%d')}\n"
            f"星期: {weekday_names[weekday]}\n"
            f"城市: {location}\n"
            f"温度: {temperature}°C\n"
            f"天气: {weather_description}\n"
            f"湿度: {humidity}%\n"
            f"风速: {wind_speed} km/h\n"
            f"小助手温馨提示: {niude}"
        )
        return weather_info
    else:
        return f"Failed to retrieve weather data. Status code: {response.status_code}"

def send_email(weather_info, from_email, email_passwd):
    from_email = from_email 
    from_password = email_passwd  # 开通服务的授权码密码，不是QQ密码！
    smtp_server = "smtp.qq.com"  
    send_receivers = [from_email] 
    smtp_port = 465  
    to_email = from_email
    msg = MIMEMultipart()
    msg['From'] = from_email
    msg['To'] = to_email
    msg['Subject'] = "自动天气预报小助手"

    msg.attach(MIMEText(weather_info, 'plain'))

    try:
        server = smtplib.SMTP_SSL(smtp_server, smtp_port)
        server.login(from_email, from_password)
        server.sendmail(from_email, to_email, msg.as_string())
        server.quit()
        print("Email sent successfully!")
    except Exception as e:
        print(f"Failed to send email: {e}")

if __name__ == '__main__':
    from_email, weather_key, email_passwd, get_location_key = get_json_()
    # get_current_location()
    weather_info = get_weather(weather_key)
    send_email(weather_info, from_email ,email_passwd)
