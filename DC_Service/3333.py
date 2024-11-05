#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import json

# 你的列表
ascii_list = [10, 43, 8, 216]

# 将 ASCII 码值列表转换为字符串
json_str = ''.join(chr(code) for code in ascii_list)

# 使用 repr 函数生成包含转义字符的字符串表示，并去掉额外的引号
json_str = repr(json_str)[1:-1]

# 反序列化 JSON 字符串
try:
    data = json.loads(json_str)
    print(data)
except json.JSONDecodeError as e:
    print(f"JSON decoding error: {e}")