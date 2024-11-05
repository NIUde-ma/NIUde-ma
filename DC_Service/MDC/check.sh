#!/bin/bash

mdc_hostname=$(hostname)

current_path="$(cd "$( dirname "$0")" && pwd )"
echo ${current_path}
current_work_path="${current_path}/../"
echo ${current_work_path}

###SOC_B_services:

COM_SERVICE_SH="${current_work_path}/script/com_service_pnc_node_exec_run.sh"

GNSS_SERVICE_SH="${current_work_path}/script/gnss_service_node_exec_run.sh"

KOALA_SERVICE_SH="${current_work_path}/script/localization_exec_run.sh"

HMI_ON_SOC_SH="${current_work_path}/script/hmi_on_soc_exec_run.sh"

HMI_SERVICE_SH="${current_work_path}/script/hmi_service_exec_run.sh"

FINITE_SERVICE_SH="${current_work_path}/script/finite_state_manager_exec_run.sh"

CAMERA_PERCEPTION_SH="${current_work_path}/script/panorama_view_camera_perception_exec_run.sh"

OBJECT_FUSION_SERVICE_SH="${current_work_path}/script/obstacle_fusion_exec_run.sh"

STATIC_FUSION_SERVICE_SH="${current_work_path}/script/static_fusion_exec_run.sh"

CONTROL_SERVICE_SH="${current_work_path}/script/control_exec_run.sh"

PLANNING_SERVICE_SH="${current_work_path}/script/planning_exec_run.sh"

###SOC_A_services:

SOC_A_COMMUNICATION_SH="${current_work_path}/script/soc_a_communication_exec_run.sh"

PANORAMA_VIEW_CAMERA_PERCEPTION_LANELINE_SH="${current_work_path}/script/panorama_view_camera_perception_laneline_exec_run.sh"

function check() {
    for script in "$@"; do
        process_name=$(basename "$script" .sh)
        if ps -ef | grep -v grep | grep "$process_name" &> /dev/null; then
            echo -e "\033[32m $process_name is Running \033[0m\n"
        else
            echo -e "\033[31m $process_name is not Up \033[0m\n"
        fi
    done
}

while true
do
  sleep 2
  clear
  if [ $mdc_hostname == "AOS_B" ];then
    check "$COM_SERVICE_SH" "$GNSS_SERVICE_SH" "$KOALA_SERVICE_SH" "$HMI_ON_SOC_SH" "$HMI_SERVICE_SH" "$FINITE_SERVICE_SH" "$CAMERA_PERCEPTION_SH" "$OBJECT_FUSION_SERVICE_SH" "$STATIC_FUSION_SERVICE_SH" "$CONTROL_SERVICE_SH" "$PLANNING_SERVICE_SH"
  else
    check "$SOC_A_COMMUNICATION_SH" "$PANORAMA_VIEW_CAMERA_PERCEPTION_LANELINE_SH"
done

