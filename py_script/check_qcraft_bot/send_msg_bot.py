#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import common
import requests
import json
import os
import subprocess
import sys


def _local_json(json_path="./local_config.json"):
    """读取本地配置文件获取飞书机器人URL"""
    if os.path.exists(json_path):
        try:
            with open(json_path, 'r', encoding='utf-8') as file:
                data = json.load(file)
                bot_urls = data.get("bot_url", "")
                if bot_urls:
                    return bot_urls
                else:
                    common.print_red("错误: local_config.json 中没有找到 bot_url 配置")
                    return None
        except FileNotFoundError:
            common.print_red("错误: 找不到 local_config.json 文件")
        except json.JSONDecodeError:
            common.print_red("错误: JSON 文件格式不正确")
        except Exception as e:
            common.print_red(f"错误: {e}")
    else:
        common.print_red(f"{json_path} is not found")
    return None


def _get_car_info(car_config="../ssh_all_car/car_config.json", car_id=None):
    if not car_id:
        common.print_red("错误: 未提供车辆ID")
        return None
    
    if os.path.exists(car_config):
        try:
            with open(car_config, 'r', encoding='utf-8') as file:
                data = json.load(file)
            
            if car_id not in data:
                common.print_red(f"错误: 未找到车辆ID '{car_id}'")
                return None
            else:
                ip_address = data[car_id]
                return {
                    "car_id": car_id,
                    "ip_address": ip_address
                }
        except FileNotFoundError:
            common.print_red("错误: 找不到 car_config.json 文件")
        except json.JSONDecodeError:
            common.print_red("错误: JSON 文件格式不正确")
        except Exception as e:
            common.print_red(f"错误: {e}")
    else:
        common.print_red(f"{car_config} is not found")
    return None


def _get_car_status(car_info):
    if not car_info:
        return None
    
    try:
        ip_address = car_info.get("ip_address")
        car_id = car_info.get("car_id")
        
        status_url = f"http://{ip_address}:8080/status"
        print(f"请求URL: {status_url}")
    
        try:
            response = requests.get(status_url, timeout=60)
            print(f"响应状态码: {response.status_code}")
            print(f"响应内容类型: {response.headers.get('Content-Type', 'unknown')}")
            print(f"响应内容长度: {len(response.text)} 字符")
            
            if response.status_code == 200:
                print(f"响应内容预览:\n{response.text[:200]}")
                
                try:
                    json_data = response.json()
                    return json_data
                except json.JSONDecodeError as json_err:
                    if response.text.strip().startswith('<'):
                        print("返回的是HTML内容，不是JSON")
                        return {
                            "car_id": car_id,
                            "status": "online",
                            "content_type": "html",
                            "html_title": "监控服务页面",
                            "url": status_url,
                            "message": "车辆监控服务正常运行，返回的是HTML界面"
                        }
                    else:
                        common.print_red(f"JSON解析错误: {json_err}")
                        common.print_red(f"响应内容: {response.text[:500]}")
                        return None
            else:
                common.print_red(f"错误: 获取{car_id}车辆状态失败，HTTP状态码: {response.status_code}")
                common.print_red(f"响应内容: {response.text[:200]}")
                return None
                
        except requests.exceptions.RequestException as e:
            common.print_red(f"网络请求错误: {e}")
            return None
            
    except Exception as e:
        common.print_red(f"获取车辆状态时发生错误: {e}")
        return None



def _send_feishu_message(bot_url, message):
    if not bot_url:
        common.print_red("错误: 飞书机器人URL为空")
        return False
    
    try:
        payload = {
            "msg_type": "text",
            "content": {
                "text": message
            }
        }
        
        headers = {
            "Content-Type": "application/json"
        }
        
        response = requests.post(bot_url, data=json.dumps(payload), headers=headers, timeout=10)
        
        if response.status_code == 200:
            common.print_green("消息发送成功")
            return True
        else:
            common.print_red(f"消息发送失败，状态码: {response.status_code}")
            common.print_red(f"响应内容: {response.text}")
            return False
            
    except requests.exceptions.RequestException as e:
        common.print_red(f"发送消息时网络错误: {e}")
        return False
    except Exception as e:
        common.print_red(f"发送消息时发生错误: {e}")
        return False


def _send_msg_main(car_id):
    # 1. 获取飞书机器人URL
    bot_url = _local_json()
    if not bot_url:
        return
    
    # 2. 获取车辆信息
    car_info = _get_car_info(car_id=car_id)
    if not car_info:
        message = f"❌ 未找到车辆ID: {car_id}"
        _send_feishu_message(bot_url, message)
        return
    
    car_status = _get_car_status(car_info)
    # print(car_status)
    
    if car_status:
        status_text = json.dumps(car_status, ensure_ascii=False, indent=2)
        message = f"🚗 车辆状态查询\n\n车辆ID: {car_id}\nIP地址: {car_info['ip_address']}\n\n状态信息:\n```\n{status_text}\n```"
    else:
        message = f"⚠️ 车辆状态查询失败\n\n车辆ID: {car_id}\nIP地址: {car_info['ip_address']}\n\n可能原因:\n1. 车辆离线\n2. 网络连接问题\n3. API接口异常"
    
    _send_feishu_message(bot_url, message)


def main():
    if len(sys.argv) < 2:
        common.print_red("使用方法: python script.py <车辆ID>")
        common.print_red("示例: python script.py car_001")
        sys.exit(1)
    
    car_id = sys.argv[1]
    _send_msg_main(car_id)

if __name__ == "__main__":
    main()