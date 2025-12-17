#!/usr/bin/env python3
from flask import Flask, request, jsonify
import subprocess
import json
import re
import threading
import os

app = Flask(__name__)

def extract_car_id(text):
    """从消息中提取车辆ID"""
    if not text:
        return None
    
    # 移除@机器人部分
    text = re.sub(r'@[^ ]+', '', text).strip()
    
    # 取第一个单词
    words = text.split()
    return words[0] if words else None

def run_script_async(car_id):
    """异步执行检查脚本"""
    try:
        print(f"🚗 开始异步检查车辆: {car_id}")
        
        result = subprocess.run(
            ["python3", "send_msg_bot.py", car_id],
            capture_output=True,
            text=True,
            timeout=30
        )
        
        if result.returncode == 0:
            output = result.stdout[:500] if result.stdout else "检查完成"
            print(f"✅ 车辆 {car_id} 检查完成")
            print(f"输出: {output[:200]}")
        else:
            error = result.stderr or result.stdout or "检查失败"
            print(f"❌ 车辆 {car_id} 检查失败")
            print(f"错误: {error[:200]}")
            
    except Exception as e:
        print(f"⚠️ 执行脚本出错: {e}")

@app.route('/webhook', methods=['GET', 'POST'])
def webhook():
    """飞书webhook接口"""
    
    # 1. 处理GET请求（飞书验证）
    if request.method == 'GET':
        print("收到GET请求（验证）")
        challenge = request.args.get('challenge')
        if challenge:
            print(f"返回挑战码: {challenge}")
            return jsonify({"challenge": challenge})
        return "OK"
    
    # 2. 处理POST请求（接收消息）
    print("收到POST请求（消息）")
    
    try:
        # 获取JSON数据
        data = request.get_json()
        if not data:
            print("错误: 没有收到JSON数据")
            return jsonify({"error": "No JSON data"}), 400
        
        print(f"收到数据，类型: {type(data)}")
        
        # 调试：打印关键字段
        print(f"数据keys: {list(data.keys())}")
        
        # 尝试解析消息内容
        text = ""
        
        # 方法1: 直接获取text
        if 'text' in data:
            text = data['text']
        # 方法2: 从content获取
        elif 'content' in data:
            if isinstance(data['content'], dict):
                text = data['content'].get('text', '')
            elif isinstance(data['content'], str):
                try:
                    content_dict = json.loads(data['content'])
                    text = content_dict.get('text', '')
                except:
                    text = data['content']
        # 方法3: 从event获取
        elif 'event' in data:
            event = data['event']
            if 'message' in event:
                message = event['message']
                if 'content' in message:
                    content = message['content']
                    if isinstance(content, str):
                        try:
                            content_dict = json.loads(content)
                            text = content_dict.get('text', '')
                        except:
                            text = content
        
        print(f"提取到的文本: '{text}'")
        
        # 提取车辆ID
        car_id = extract_car_id(text)
        print(f"提取的车辆ID: '{car_id}'")
        
        if not car_id:
            # 如果没有提取到车辆ID，回复提示
            return jsonify({
                "msg_type": "text",
                "content": {"text": "请指定车辆ID，例如: @RX_BOT q3707"}
            })
        
        print(f"✅ 识别到车辆ID: {car_id}")
        
        # 立即回复确认消息
        immediate_response = {
            "msg_type": "text",
            "content": {
                "text": f"收到！正在检查车辆 {car_id}，请稍等..."
            }
        }
        
        # 启动异步线程执行检查
        thread = threading.Thread(target=run_script_async, args=(car_id,))
        thread.daemon = True
        thread.start()
        
        print(f"已启动异步检查线程，立即返回响应")
        
        return jsonify(immediate_response)
        
    except Exception as e:
        print(f"❌ 处理消息出错: {e}")
        import traceback
        traceback.print_exc()
        
        return jsonify({
            "msg_type": "text",
            "content": {"text": f"处理消息时出错: {str(e)}"}
        })

@app.route('/')
def home():
    """首页"""
    import socket
    hostname = socket.gethostname()
    ip = socket.gethostbyname(hostname)
    
    return f"""
    <h1>🚗 RX Bot 服务运行正常</h1>
    <p>✅ 服务状态: <strong>运行中</strong></p>
    <p>🏠 主机名: {hostname}</p>
    <p>📡 内网IP: {ip}</p>
    <p>🌐 公网IP: 请查看路由器或使用 curl ifconfig.me</p>
    <hr>
    <h3>📱 访问地址:</h3>
    <ul>
        <li>首页: <a href="/">/</a></li>
        <li>测试: <a href="/test">/test</a></li>
        <li>Webhook: <code>/webhook</code></li>
    </ul>
    <h3>🔧 飞书配置:</h3>
    <ul>
        <li>请求地址: <code>http://你的公网IP:5000/webhook</code></li>
        <li>请求方式: POST</li>
        <li>Content-Type: application/json</li>
    </ul>
    <p><strong>注意:</strong> 确保防火墙已开放5000端口</p>
    """

@app.route('/test')
def test():
    """测试页面"""
    return """
    <h2>测试RX Bot</h2>
    <form action="/test_action" method="get">
        <label>车辆ID:</label>
        <input type="text" name="car_id" value="q3707" required>
        <button type="submit">测试</button>
    </form>
    <p>或者直接访问: <a href="/test_action?car_id=q3707">/test_action?car_id=q3707</a></p>
    <hr>
    <h3>测试Webhook:</h3>
    <form action="/webhook" method="post" target="_blank">
        <label>模拟飞书消息:</label><br>
        <textarea name="json_data" rows="5" cols="50">
{
    "text": "@RX_BOT q3708"
}
        </textarea><br>
        <button type="submit">发送测试</button>
    </form>
    """

@app.route('/test_action', methods=['GET'])
def test_action():
    """执行测试"""
    car_id = request.args.get('car_id', 'q3707')
    
    try:
        result = subprocess.run(
            ["python3", "send_msg_bot.py", car_id],
            capture_output=True,
            text=True,
            timeout=10
        )
        
        if result.returncode == 0:
            output = result.stdout[:1000] if result.stdout else "检查完成"
            return f"""
            <h2>✅ 测试成功 - 车辆 {car_id}</h2>
            <pre>{output}</pre>
            <p><a href="/test">返回测试</a> | <a href="/">返回首页</a></p>
            """
        else:
            error = result.stderr or result.stdout or "未知错误"
            return f"""
            <h2>❌ 测试失败 - 车辆 {car_id}</h2>
            <pre>{error[:1000]}</pre>
            <p><a href="/test">返回测试</a> | <a href="/">返回首页</a></p>
            """
            
    except Exception as e:
        return f"""
        <h2>⚠️ 测试异常</h2>
        <pre>错误: {str(e)}</pre>
        <p><a href="/test">返回测试</a> | <a href="/">返回首页</a></p>
        """

if __name__ == '__main__':
    print("=" * 60)
    print("🚀 RX Bot 公网服务启动")
    print("=" * 60)
    
    # 获取本机IP
    import socket
    hostname = socket.gethostname()
    local_ip = socket.gethostbyname(hostname)
    
    print(f"📡 内网地址: http://{local_ip}:5000/")
    print(f"🌐 公网地址: http://你的公网IP:5000/")
    print(f"🔗 Webhook: http://你的公网IP:5000/webhook")
    print("")
    print("📋 配置步骤:")
    print("  1. 确保防火墙开放5000端口")
    print("  2. 路由器设置端口转发")
    print("  3. 飞书配置webhook地址")
    print("=" * 60)
    
    # 绑定所有IP地址
    app.run(host='0.0.0.0', port=5000, debug=True)