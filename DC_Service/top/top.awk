#!/usr/bin/awk -f
# RS表示Record Seperate，记录分隔符，根据top输出的格式规范，此处已top头部的Tasks信息进行分割
BEGIN { RS="Tasks"; FS="\n"} 
{
    # cmds为定义的需要处理关心的进程命令列表，可以根据分析的需要进行删减
    cmds["t"] = "top"
    cmds["entrypoint"] = "entrypoint.sh"

    if (NR == 1) {
        # 输出标题行
        printf("total user nice sys idle iow irq sirq host occupy ")
        for (c in cmds) {
            printf("%s ", c)
        }
        printf("\r\n")
    }

    foundCpu = 0
    for (i = 1; i <= NF; ++i) {
        ret = match($i, "[0-9]+%cpu")
        if (ret > 0) {
            foundCpu = 1
            split($i, array, " ")
            # array element as 400%cpu 112%user   4%nice  33%sys 249%idle   0%iow   0%irq   1%sirq   0%host
            total = 0
            idle = 0
            for (str in array) {
                split(array[str], percent, "%")
                if (percent[2] == "cpu") {
                    total = percent[1]
                } else if (percent[2] == "idle") {
                    idle = percent[1]
                }
                printf("%5.1f ", percent[1]) 
            }

            #计算总占用
            printf("%5.1f ", total - idle)
        }
    }

    # 过滤无效数据
    if (foundCpu == 0) {
        next
    }

    INDEX_CPU_PERCENT = 9
    for (cmd in cmds) {
        # 匹配对应cmd并累计对应的占用求和进行统计，cmd可以为正则表达式
        occupy = 0.0f
        for (i = 1; i <= NF; ++i) {
            ret = match($i, cmds[cmd])
            if (ret > 0) {
                split($i, array, " ")
                occupy += array[INDEX_CPU_PERCENT]
            }
        }
        printf("%5.1f ", occupy)
    }
    printf("\r\n")
}