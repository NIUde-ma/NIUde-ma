#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import canmatrix
from subprocess import check_output
import can

matrix = canmatrix.formats.loadp("your_dbc_file.dbc")

signal_name = "your_signal_name"

blf_file = "your_blf_file.blf"
output = check_output(["blc", blf_file]).decode("utf-8").splitlines()

for line in output:
    parts = line.split()
    if len(parts) >= 5 and parts[3] == "Rx" and parts[4] == signal_name:
        signal_value = parts[6]  # 信号值在第6个位置
        print(f"{signal_name}: {signal_value}")