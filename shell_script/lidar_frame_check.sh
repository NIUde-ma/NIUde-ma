#!/bin/bash

helper() {
    echo -e     "$1 is your input run_path"
}

checker() {
    local run_path="$1"
    local min_count=10
    
    if [ ! -d "$run_path" ]; then
        echo "Error: Directory '$run_path' does not exist"
        return 1
    fi
    
    local temp_file=$(mktemp)
    
    ./scripts/dump_lidar_frame.sh --run="$run_path" --start=130 --end=150 --lidar_ids=LDR_FRONT 2>&1 | \
    grep "mid_tim" | \
    awk '{print substr($11, 0, index($11, "."))}' | \
    uniq -c > "$temp_file"
    
    local all_pass=true
    while read -r count timestamp; do
        echo "Timestamp: $timestamp, Count: $count"
        if [ "$count" -lt "$min_count" ]; then
            echo "ERROR: Count $count is below minimum threshold $min_count for timestamp $timestamp"
            all_pass=false
        fi
    done < "$temp_file"
    
    # 清理临时文件
    
    if $all_pass; then
        echo "SUCCESS: All timestamps have count >= $min_count"
        return 0
    else
        echo "FAILED: Some timestamps have count < $min_count"
        return 1
    fi
}
