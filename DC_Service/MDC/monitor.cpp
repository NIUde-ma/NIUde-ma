#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cstdlib>

// 函数：获取指定进程名的PID
std::vector<int> getPIDsByName(const std::string& processName) {
    std::vector<int> pids;
    std::string command = "pgrep " + processName;
    FILE* pipe = popen(command.c_str(), "r");
    if (!pipe) {
        std::cerr << "无法执行命令: " << command << std::endl;
        return pids;
    }

    char buffer[128];
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        pids.push_back(std::stoi(buffer));
    }

    pclose(pipe);
    return pids;
}

// 函数：读取指定PID的进程状态信息
void readProcessStatus(int pid) {
    std::string path = "/proc/" + std::to_string(pid) + "/status";
    std::ifstream file(path);

    if (!file.is_open()) {
        std::cerr << "无法打开进程状态文件: " << path << std::endl;
        return;
    }

    std::string line;
    while (getline(file, line)) {
        std::cout << line << std::endl;
    }

    file.close();
}

int main() {
    std::vector<std::string> processNames = {
        "planning_exec",
        "control_exec",
        "static_fusion_exec",
        "obstacle_fusion_exec",
        "uss_perception_exec",
        "mega_exec",
        "panorama_view_camera_perception_exec",
        "ehr_exec",
        "finite_state_manager_exec",
        "hmi_on_soc_exec",
        "localization_exec",
        "soc_b_communication_exec",
        "panorama_view_camera_node_exec",
        "sdmap_exec",
        "hmi_service_exec",
        "gnss_service_node_exec",
        "com_service_pnc_node_exec"
    };

    for (const auto& processName : processNames) {
        std::vector<int> pids = getPIDsByName(processName);
        if (pids.empty()) {
            std::cout << "没有找到进程: " << processName << std::endl;
        } else {
            for (int pid : pids) {
                std::cout << "进程名: " << processName << ", PID: " << pid << std::endl;
                readProcessStatus(pid);
            }
        }
    }

    return 0;
}
