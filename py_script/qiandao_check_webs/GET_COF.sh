#!/bin/bash

car_env_path="/home/qcraft/qcraft_data/env"
config_path="/home/qcraft/qcraft_data/vehicles/v2"
ip_config_path="/home/qcraft/qcraft_data/vehicles/v2/lidar/installation"

car_id=$(grep -i '^CAR_ID=' "$car_env_path" 2>/dev/null | cut -d'=' -f2)

if [[ -z "$car_id" ]]; then
    echo "[]"
    exit 1
fi

config_file="$config_path/$car_id.pb.txt"

if [[ ! -f "$config_file" ]]; then
    echo "[]"
    exit 1
fi

temp_file=$(mktemp)

grep -E -i "LIDAR" -A 1 "$config_file" | grep "key:" | awk '{print $2}' | sed 's/"//g' | \
while read -r sn; do
    [[ -z "$sn" ]] && continue
    
    # 查找配置文件
    find "$ip_config_path" -type f -name "*${sn}*.pb.txt" 2>/dev/null | \
    while read -r file; do
        ip=$(grep -E 'ip:[[:space:]]*"' "$file" | head -1 | sed 's/.*"\([^"]*\)".*/\1/')
        
        if [[ -n "$ip" ]]; then
            # 提取lidar_id，处理两种格式：
            # 1. lidar_id: "LDR_FRONT" (带引号)
            # 2. lidar_id: LDR_FRONT (不带引号)
            lidar_line=$(grep -E 'lidar_id:' "$file" | head -1)
            lidar_id=""
            
            if [[ -n "$lidar_line" ]]; then
                # 尝试提取带引号的内容
                lidar_id=$(echo "$lidar_line" | sed -n 's/.*"\([^"]*\)".*/\1/p')
                
                # 如果没有引号，提取冒号后的内容
                if [[ -z "$lidar_id" ]]; then
                    lidar_id=$(echo "$lidar_line" | sed -n 's/lidar_id:[[:space:]]*//p' | awk '{print $1}')
                fi
            fi
            
            # 如果没有找到lidar_id，使用文件名（去掉.pb.txt后缀）
            if [[ -z "$lidar_id" ]]; then
                lidar_id=$(basename "$file" .pb.txt)
            fi
            
            # 写入临时文件
            echo "(\"$ip\", \"$lidar_id\")" >> "$temp_file"
        fi
    done
done

# 输出结果
echo "["
if [[ -s "$temp_file" ]]; then
    first=true
    while IFS= read -r line; do
        if [[ "$first" == true ]]; then
            echo "            $line"
            first=false
        else
            echo "            ,$line"
        fi
    done < "$temp_file"
fi
echo "]"

# 清理临时文件
rm -f "$temp_file"