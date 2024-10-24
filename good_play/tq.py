#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import requests
from fake_useragent import UserAgent
import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart




def get_weather():
    ua = UserAgent()

    url = "https://api.weatherstack.com/current?access_key=175a60bc38cb90f8609bc5d8ca7c5347&query=hefei"

    # params_use = {
    #     "access_key": "175a60bc38cb90f8609bc5d8ca7c5347",
    #     "query": "heife"
    # }

    headers = {
        "User-Agent": ua.random
    }

    response = requests.get(url, headers=headers)
    # print(response.status_code)

    if response.status_code == 200:
        weather_data = response.json()
        # print(weather_data)
        
        location = weather_data["location"]["name"]
        temperature = weather_data["current"]["temperature"]
        weather_description = weather_data["current"]["weather_descriptions"][0]
        humidity = weather_data["current"]["humidity"]
        wind_speed = weather_data["current"]["wind_speed"]
        
        print(
            f"城市: {location}\n"
            f"温度: {temperature}°C\n"
            f"天气: {weather_description}\n"
            f"湿度: {humidity}%\n"
            f"风速: {wind_speed} km/h"
        )
        # return weather_info
    else:
        return f"Failed to retrieve weather data. Status code: {response.status_code}"

# def send_email(weather_info, to_email):
#     from_email = "maf629209@gmail.com"  # 你的邮箱地址
#     from_password = "NIUdebutan6!"  # 你的邮箱密码
#     smtp_server = "smtp.gmail.com"  # 你的 SMTP 服务器地址
#     smtp_port = 587  # 你的 SMTP 服务器端口

#     msg = MIMEMultipart()
#     msg['From'] = from_email
#     msg['To'] = to_email
#     msg['Subject'] = "合肥天气预报"

#     msg.attach(MIMEText(weather_info, 'plain'))

#     try:
#         server = smtplib.SMTP(smtp_server, smtp_port)
#         server.starttls()
#         server.login(from_email, from_password)
#         server.sendmail(from_email, to_email, msg.as_string())
#         server.quit()
#         print("Email sent successfully!")
#     except Exception as e:
#         print(f"Failed to send email: {e}")

if __name__ == '__main__':
    get_weather()
    # weather_info = get_weather()
    # send_email(weather_info, "maf629209@gmail.com")  # 接收者的邮箱地址
