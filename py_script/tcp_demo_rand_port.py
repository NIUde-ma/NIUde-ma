import socket
import time
import threading
import select

class TcpClient:
    def __init__(self, remote_host, remote_port, message, interval):
        #self.local_port = local_port
        self.remote_host = remote_host
        self.remote_port = remote_port
        self.message = message
        self.interval = interval
        self.last_send_time = 0
        self.client_socket = None
        self.is_connected = False
        
    def connect(self):
        try:
            if self.client_socket:
                self.client_socket.close()
                
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.connect((self.remote_host, self.remote_port))
            self.client_socket.settimeout(4.0)
            self.is_connected = True
            print(f"Connected to {self.remote_host}:{self.remote_port}")
            # 如果需要知道系统分配的端口号
            local_address = self.client_socket.getsockname()
            print(f"Local port: {local_address[1]}")
            return True
        except Exception as e:
            print(f"Connection error: {e}")
            self.is_connected = False
            return False
            
    def send_message(self):
        """发送消息线程"""
        while True:
            try:
                if not self.is_connected:
                    if not self.connect():
                        time.sleep(5)  # 连接失败等待5秒后重试
                        continue
                
                # 发送数据
                self.client_socket.send(self.message.encode())
                self.last_send_time = time.time()
                print(f"Sent: {self.message}")
                
            except Exception as e:
                print(f"Send error: {e}")
                self.is_connected = False
                
            time.sleep(self.interval)
            
    def receive_message(self):
        """接收消息线程"""
        while True:
            if not self.is_connected:
                time.sleep(1)
                continue
                
            try:
                ready = select.select([self.client_socket], [], [], 4.0)
                if ready[0]:
                    data = self.client_socket.recv(1024)
                    if data:
                        current_time = time.time()
                        response_time = current_time - self.last_send_time
                        print(f"Received: {data.decode()}")
                        print(f"Response time: {response_time:.2f} seconds")
                    else:
                        # 连接断开
                        print("Server closed connection")
                        self.is_connected = False
                
            except socket.timeout:
                pass  # 超时继续等待
            except Exception as e:
                print(f"Receive error: {e}")
                self.is_connected = False
                
    def start(self):
        # 创建发送和接收线程
        send_thread = threading.Thread(target=self.send_message)
        receive_thread = threading.Thread(target=self.receive_message)
        
        # 设置为守护线程
        send_thread.daemon = True
        receive_thread.daemon = True
        
        # 启动线程
        send_thread.start()
        receive_thread.start()
        
        # 保持主线程运行
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nProgram terminated by user")
            if self.client_socket:
                self.client_socket.close()

if __name__ == "__main__":
    with open('./lever.config', 'r') as f:
        MESSAGE = f.read()
    # 配置参数
    # LOCAL_PORT = 7778
    REMOTE_HOST = '192.168.0.122'
    REMOTE_PORT = 7777
    # MESSAGE = "getting version\r\ngnss2bodyoffset 0 0 0 5 5 5\r\nins2gnssoffset 1 1 1 5 5 5\r\n"
    INTERVAL = 5  # 发送间隔(秒)
    
    # 创建并启动客户端
    client = TcpClient(REMOTE_HOST, REMOTE_PORT, MESSAGE, INTERVAL)
    client.start()
