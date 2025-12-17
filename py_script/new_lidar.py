      
import socket, ssl, pprint, time
import random
import struct
import binascii
import sys
import argparse

default_host = "192.168.5.205"
default_port = 9347
timestamp = time.strftime("%Y%m%d_%H%M%S")
filename = f'lidar_intrinsics_{timestamp}.txt'

def convert_azimuth_hex(hex_string, prefix, file):
    results = []
    for i in range(0, len(hex_string), 4):
        hex_chunk = hex_string[i:i+4]
        big_endian_hex = hex_chunk[2:4] + hex_chunk[0:2]
        decimal_value = int(big_endian_hex, 16)
        if decimal_value > 32767:
            decimal_value -= 65536
        double_value = decimal_value / 256.0
        results.append(double_value)
    for result in results:
        file.write(f"{prefix}: {result}\n")

def process_azimuth(hex_string):
    lengths = [400, 400, 400]
    prefixes = ['even_azimuth_offsets', 'odd_azimuth_offsets', 'elevations']
    with open(filename, 'w') as file:
        start = 0
        for i, length in enumerate(lengths):
            end = start + length
            hex_part = hex_string[start:end]
            convert_azimuth_hex(hex_part, prefixes[i], file)
            start = end

def convert_firetimes_hex(hex_string, prefix, file):
    results = []
    for i in range(0, len(hex_string), 4):
        hex_chunk = hex_string[i:i+4]
        big_endian_hex = hex_chunk[2:4] + hex_chunk[0:2]
        decimal_value = int(big_endian_hex, 16)
        results.append(decimal_value)
    for result in results:
        file.write(f"{prefix}: {result}\n")

def process_response_payload(response_payload):
    # 将十六进制字符串转换回原始文本
    try:
        # 尝试将十六进制字符串解码为原始文本
        intrinsics_str = bytes.fromhex(response_payload).decode('utf-8')
    except:
        # 如果解码失败，直接使用原始字符串
        intrinsics_str = response_payload
        
    lines = intrinsics_str.strip().split('\n')
    
    # 检查第一行是否为预期格式
    if len(lines) > 0:
        first_line = lines[0].strip()
        if first_line != "Laser id,Elevation,Azimuth" and first_line != "Channel,Elevation,Azimuth":
            print(f"Warning: Hesai intrinsics string could be wrong. First line is {first_line}")
    
    line_counter = 0
    elevations = []
    azimuth_offsets = []
    
    # 从第二行开始处理数据行
    for line in lines[1:]:
        # 跳过空行或太短的行
        if len(line.strip()) < 5:
            continue
            
        line_counter += 1
        parts = line.strip().split(',')
        
        if len(parts) < 3:
            print(f"Warning: Invalid line format: {line}")
            continue
            
        try:
            line_id = int(parts[0])
            elevation = float(parts[1])
            azimuth = float(parts[2])
            
            # 验证line_id是否与行号一致
            if line_id != line_counter:
                print(f"Warning: Line ID mismatch. Expected: {line_counter}, Got: {line_id}")
                break
            
            # 存储数据
            elevations.append(elevation)
            azimuth_offsets.append(azimuth)
        except ValueError as e:
            print(f"Error parsing line: {line}. Error: {e}")
            continue
    
    # 按照指定格式写入文件
    with open(filename, 'w') as file:
        # 先写入elevations
        for elevation in elevations:
            file.write(f"elevations: {elevation}\n")
        
        # 再写入azimuth_offsets
        for azimuth in azimuth_offsets:
            file.write(f"azimuth_offsets: {azimuth}\n")
    
    print(f"Conversion results written to {filename}")

def test_connection(host, port):
    """测试是否能够连接到指定的主机和端口"""
    try:
        print(f"测试连接到 {host}:{port}...")
        # 创建socket连接
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(5)  # 设置5秒超时
        s.connect((host, port))
        s.close()
        print("连接成功!")
        return True
    except socket.error as e:
        print(f"连接测试失败: {e}")
        return False

class PTC:
    def __init__(self, host=default_host, port=default_port):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # 移除绑定随机端口的操作
        self.s.settimeout(30)
        print(f"正在连接到 {host}:{port}...")
        self.s.connect((host, port))
        print("连接成功")

    def closeSocket(self):
        try:
            self.s.shutdown(1)
        except:
            pass
        # self.s.close()

    def ByteToHex(self, h):
        return ''.join(["%02x" % x for x in h]).strip()

    def read_bytes(self, payload_size):
        chunks = []
        bytes_received = 0
        while bytes_received < payload_size:
            chunk = self.s.recv(payload_size - bytes_received)
            if chunk == b"":
                raise RuntimeError("Socket has been unexpectedly closed")
            chunks.append(chunk)
            bytes_received = bytes_received + len(chunk)

        return b''.join(chunks)

    def sender(self, cmd_code, payload):
        """发送命令到LiDAR并接收响应
        参数:
        cmd_code -- 命令代码
        payload -- 负载数据，应输入类似 '015d010207' 格式
        返回:
        包含响应信息的字典
        """
        if cmd_code in [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e',
                        'f']:
            cmd_code = "0" + str(cmd_code)
        print("payload :")
        print(payload)
        if not payload or payload.upper() == "NONE":
            payload_len = 0
            p = '4774' + str(cmd_code) + "00" + struct.pack('>L', payload_len).hex()
        else:
            payload_len = len(bytes.fromhex(payload))
            p = '4774' + str(cmd_code) + "00" + struct.pack('>L', payload_len).hex() + payload
        data = bytes.fromhex(p)
        self.s.send(data)
        response = self.s.recv(8)
        print("response: ")
        print(response)
        r_cmd = bytes.hex(response[2:3])
        r_returnCode = bytes.hex(response[3:4])
        if bytes.hex(response[4:8]) == "\x00\x00\x00\x00":
            r_length = 0
            response_payload = ""
        else:
            r_length = int(bytes.hex(response[4:8]), 16)
            #response_payload = self.read_bytes(r_length)
            response_payload = self.read_bytes(r_length)
        # print("command is: %s, get return code: %s, return length: %s, \nreturn string:\n%s" % (
        #     r_cmd, r_returnCode, r_length,  binascii.hexlify(response_payload).decode('utf-8')))
        final_response = {
            "response_command": r_cmd,
            "response_return_code": r_returnCode,
            "response_payload_length": r_length,
            "response_payload": binascii.hexlify(response_payload).decode('utf-8')
        }
        return final_response

if __name__ == "__main__":
    # 创建命令行参数解析器
    parser = argparse.ArgumentParser(description='获取LiDAR内参数据')
    parser.add_argument('--host', default=default_host, help=f'LiDAR主机IP地址 (默认: {default_host})')
    parser.add_argument('--port', type=int, default=default_port, help=f'LiDAR端口号 (默认: {default_port})')
    parser.add_argument('--test', action='store_true', help='仅测试连接，不执行数据获取')
    args = parser.parse_args()
    
    # 如果指定了测试模式，仅测试连接
    if args.test:
        test_connection(args.host, args.port)
        sys.exit(0)
    
    print(f"请确保PC能够ping通LiDAR: {args.host}")
    
    try:
        # 先进行连接测试
        if not test_connection(args.host, args.port):
            print("连接测试失败，无法继续操作。请检查网络连接和LiDAR状态。")
            sys.exit(1)
        
        ss = PTC(args.host, args.port)
        response = ss.sender('05', "")
        process_response_payload(response['response_payload'])
        ss.closeSocket()
    except Exception as e:
        print(f"发生错误: {e}")
        # 打印详细的错误信息以便诊断
        import traceback
        traceback.print_exc()

    