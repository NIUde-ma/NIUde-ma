#!/bin/bash

helper() {
    echo -e "while fix calibrations"
    echo -e "$0 input your car_id"
}

hostname=$(hostname)

function_dirs() {
    fix_dir_txt="/qcraft/fix_car.txt"
    fix_vehicle_path="/hosthome/qcraft/vehicles"

    car_ids="$1"

    if [ ! -f "$fix_dir_txt" ]; then
        echo -e "not found fix_car.txt"
        return 1
    fi

    if [ ! -d "$fix_vehicle_path" ]; then
        echo -e "请找凯哥确认使用的vehicle路径！,$fix_vehicle_path not found "
        return 1
    fi

    while IFS= read -r dir; do
        if [ -n "$dir" ]; then
            echo "Processing: $dir"
            bazel run -c opt //offboard/tools:lc_lite_run_recovery -- --base_dir=/media/s3/run_data_2 --run_name="$dir" --vehicle_param_dir=/hosthome/qcraft/vehicles --car_id="$car_ids"
            if [ $? -eq 0 ]; then
                echo -e "fix done for $dir"
            else
                echo -e "$dir fix errors"
                exit 1
            fi
        fi
    done < "$fix_dir_txt"
}

case $1 in
    "$1")
        if [ -n "$1" ] ; then
            function_dirs "$1"
        else
            helper
        fi
        ;;
    *)
        helper
        ;;
esac
