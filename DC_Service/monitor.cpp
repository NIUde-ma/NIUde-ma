#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cstdlib>
#include <sstream>
#include <chrono>
#include <thread>
#include <unistd.h>


// 获取主机名
std::string getHost() {
    std::string command = "hostname";
    FILE* pipe = popen(command.c_str(), "r");
    if (!pipe) {
        std::cerr << "无法执行命令: " << command << std::endl;
        return "";
    }

    char buffer[128];
    std::string result = "";
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        result += buffer;
    }

    pclose(pipe);
    // 去除换行符
    if (!result.empty() && result.back() == '\n') {
        result.pop_back();
    }

    return result;
}

// 获取指定进程名的PID
std::vector<int> getPIDsByName(const std::string& processName) {
    std::vector<int> pids;
    std::string command = "pgrep -f " + processName;
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

void readFile(const std::string& filePath, std::vector<std::string>& processNames) {
    std::ifstream ifs(filePath);
    if (!ifs.is_open()) {
        std::cerr << "open file failed: " << filePath << std::endl;
        return;
    }

    std::string line;
    while (std::getline(ifs, line)) {
        line.erase(line.find_last_not_of("\n\r") + 1);
        processNames.push_back(line);
    }

    ifs.close();
}

// 读取指定PID的进程状态信息
void readProcessStatus(int pid) {
    std::string path = "/proc/" + std::to_string(pid) + "/status";
    std::ifstream file(path);

    if (!file.is_open()) {
        std::cerr << "无法打开进程状态文件: " << path << std::endl;
        return;
    }

    std::string line;
    std::string vmSize, vmRSS, cpus_list;
    while (getline(file, line)) {
        if (line.find("VmSize:") != std::string::npos) {
            vmSize = line.substr(line.find(":") + 1);
        } else if (line.find("VmRSS:") != std::string::npos) {
            vmRSS = line.substr(line.find(":") + 1);
        } else if (line.find("Cpus_allowed_list") != std::string::npos) {
            cpus_list = line.substr(line.find(":") + 1);
        }
    }

    file.close();

    std::cout << "Virtual Memory Size: " << vmSize << std::endl;
    std::cout << "Resident Set Size: " << vmRSS << std::endl;
    std::cout << "Cpus_allowed_list: " << cpus_list << std::endl;
}

// 获取磁盘io使用率
std::vector<unsigned long> getDiskStats(const std::string& diskName) {
    std::string path = "/proc/diskstats";
    std::ifstream file(path);

    if (!file.is_open()) {
        std::cerr << "无法读取文件: " << path << std::endl;
        return {};
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::vector<std::string> tokens;
        std::string token;
        while (iss >> token) {
            tokens.push_back(token);
        }

        // 检查磁盘名称是否匹配
        if (tokens.size() >= 3 && tokens[2] == diskName) {
            std::vector<unsigned long> stats;
            for (size_t i = 3; i < tokens.size(); ++i) {
                stats.push_back(std::stoul(tokens[i]));
            }
            file.close();
            return stats;
        }
    }

    file.close();
    std::cerr << "未找到磁盘: " << diskName << std::endl;
    return {};
}

// 获取指定PID的进程CPU使用率
double getCPUUsage(int pid) {
    std::string path = "/proc/" + std::to_string(pid) + "/stat";
    std::ifstream file(path);

    if (!file.is_open()) {
        std::cerr << "无法打开进程状态文件: " << path << std::endl;
        return -1;
    }

    std::string line;
    std::getline(file, line);
    file.close();

    std::istringstream iss(line);
    std::vector<std::string> tokens;
    std::string token;
    while (iss >> token) {
        tokens.push_back(token);
    }

    // 获取进程的CPU时间（utime + stime）
    unsigned long utime = std::stoul(tokens[13]);
    unsigned long stime = std::stoul(tokens[14]);
    unsigned long total_time = utime + stime;

    // 获取系统启动时间（以jiffies为单位）
    std::ifstream uptime_file("/proc/uptime");
    std::string uptime_line;
    std::getline(uptime_file, uptime_line);
    uptime_file.close();

    double uptime = std::stod(uptime_line.substr(0, uptime_line.find(' ')));
    double hertz = sysconf(_SC_CLK_TCK);

    // 计算进程的CPU使用率
    double seconds = uptime - (std::stoul(tokens[21]) / hertz);
    double cpu_usage = 100.0 * ((total_time / hertz) / seconds);

    return cpu_usage;
}

// 读取 /proc/stat 文件并解析CPU信息
std::vector<unsigned long> readCPUStat() {
    std::ifstream file("/proc/stat");
    if (!file.is_open()) {
        std::cerr << "无法打开 /proc/stat 文件" << std::endl;
        return {};
    }

    std::string line;
    std::getline(file, line);
    file.close();

    std::istringstream iss(line);
    std::string cpu;
    unsigned long user, nice, system, idle, iowait, irq, softirq, steal, guest, guest_nice;
    iss >> cpu >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal >> guest >> guest_nice;

    std::vector<unsigned long> cpuTimes = {user, nice, system, idle, iowait, irq, softirq, steal, guest, guest_nice};
    return cpuTimes;
}

// 计算CPU利用率
double calculateCPUUsage(const std::vector<unsigned long>& prevTimes, const std::vector<unsigned long>& currTimes) {
    unsigned long prevIdle = prevTimes[3] + prevTimes[4];
    unsigned long currIdle = currTimes[3] + currTimes[4];

    unsigned long prevNonIdle = prevTimes[0] + prevTimes[1] + prevTimes[2] + prevTimes[5] + prevTimes[6] + prevTimes[7];
    unsigned long currNonIdle = currTimes[0] + currTimes[1] + currTimes[2] + currTimes[5] + currTimes[6] + currTimes[7];

    unsigned long prevTotal = prevIdle + prevNonIdle;
    unsigned long currTotal = currIdle + currNonIdle;

    unsigned long totald = currTotal - prevTotal;
    unsigned long idled = currIdle - prevIdle;

    if (totald == 0) {
        return 0.0;  // 防止除零错误
    }

    double cpuUsage = (totald - idled) * 100.0 / totald;
    return cpuUsage;
}

int main() {
    std::string mdc_host = getHost();
    std::vector<std::string> processNames;

    // 读取 monitor_need.txt 文件
    std::string monitorFilePath;
    if (mdc_host == "AOS_A") {
        monitorFilePath = "/opt/usr/iflytek/cluster_a/gea/script/monitor_lib/monitor_need.txt";
    } else if (mdc_host == "AOS_B") {
        monitorFilePath = "/opt/usr/iflytek/cluster_b/gea/script/monitor_lib/monitor_need.txt";
    } else {
        std::cerr << "unknown host: " << mdc_host << std::endl;
        return 1;
    }

    readFile(monitorFilePath, processNames);

    if (processNames.empty()) {
        std::cerr << "No process names found in monitor_need.txt" << std::endl;
        return 1;
    }

    double hertz = sysconf(_SC_CLK_TCK);
    double interval = 1.0 / hertz;  // 计算间隔时间

    std::vector<unsigned long> prevCPUTimes = readCPUStat();
    if (prevCPUTimes.empty()) {
        return 1;
    }

    while (true) {
        std::string diskName = "mmcblk0";
        std::vector<unsigned long> stats = getDiskStats(diskName);

        if (!stats.empty()) {
            std::cout << "磁盘 " << diskName << " 的统计信息:" << std::endl;

            //iostatus list
            std::vector<std::string> fieldDescriptions = {
                "读完成次数",
                "合并读完成次数",
                "读扇区数",
                "读操作花费的毫秒数",
                "写完成次数",
                "合并写完成次数",
                "写扇区数",
                "写操作花费的毫秒数",
                "正在进行的 I/O 操作数",
                "I/O 操作花费的毫秒数",
                "加权 I/O 操作花费的毫秒数",
                "丢弃完成次数",
                "合并丢弃完成次数",
                "丢弃扇区数",
                "丢弃操作花费的毫秒数",
                "冲刷完成次数",
                "冲刷操作花费的毫秒数"
            };

            for (size_t i = 0; i < stats.size(); ++i) {
                std::cout << "----------------------------------------" << std::endl;
                std::cout << fieldDescriptions[i] << ": " << stats[i] << std::endl;
            }
        }

        std::cout << std::endl;

        std::cout << "----------------------------------------" << std::endl;
        std::vector<unsigned long> currCPUTimes = readCPUStat();
        if (currCPUTimes.empty()) {
            return 1;
        }

        std::cout << std::endl;
        std::cout << std::endl;
        std::cout << std::endl;

        double cpuUsage = calculateCPUUsage(prevCPUTimes, currCPUTimes);
        std::cout << "CPU整体利用率: " << cpuUsage << "%" << std::endl;
        std::cout << "----------------------------------------" << std::endl;

        for (const auto& processName : processNames) {
            std::vector<int> pids = getPIDsByName(processName);
            if (pids.empty()) {
                std::cout << "打印之: " << pids[0] << std::endl;
                std::cout << "没有找到进程: " << processName << std::endl;
            } else {
                for (int pid : pids) {
                    std::cout << "进程名: " << processName << ", PID: " << pid << std::endl;
                    readProcessStatus(pid);
                    double processCPUUsage = getCPUUsage(pid);
                    if (processCPUUsage >= 0) {
                        std::cout << "进程 PID: " << pid << " 的CPU使用率: " << processCPUUsage << "%" << std::endl;
                    } else {
                        std::cerr << "无法获取进程的CPU使用率" << std::endl;
                    }
                }
            }
            std::cout << std::endl;
        }

        prevCPUTimes = currCPUTimes;
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    return 0;
}