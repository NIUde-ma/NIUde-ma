ubuntu搭建aarch64 cuda交叉编译环境记录

leaf_csdn

已于 2024-05-20 10:13:58 修改

阅读量1.6k
 收藏 7

点赞数 1
分类专栏： 嵌入式计算平台 文章标签： 计算机视觉 嵌入式硬件
版权

嵌入式计算平台
专栏收录该内容
6 篇文章1 订阅
订阅专栏
背景介绍
在windows环境下安装的VM虚拟机中配置的ubuntu系统，需要编译用于jetson xavier nx平台下可执行的程序，ubuntu 20.04系统为amd64架构，而jetson为arm，或者也叫aarch64架构。嵌入式平台jetson安装到设备中后不具备开发条件，需要在ubuntu系统中构建并编译好在jetson上运行的程序，然后拷贝到jetson平台中执行。

交叉编译需要的环境
目的不同，交叉编译的环境可能不同。本次，我们主要是想在jetson上执行基于CUDA的结构光三维重建程序，尚不涉及deep learning等。因此，在ubuntu中我们需要两个东西：1) 用于开发c++和cuda的开发工具，类似microsoft visual studio, 在ubunu中我们选择eclipse作为IDE；2）用于aarch64架构的编译器

eclipse安装
起初我是想下载Nvidia提供的IDE：nsight eclipse edition. 现在进入官网后提示cuda11.0之后不再支持独立的nsight eclipse edition软件，而是提供一个nsight eclipse plugins插件，可以在安装eclipse后添加该插件

首先安装eclipse，由于这个软件是基于java的，因此，需要安装jdk

sudo apt update
sudo apt install default-jdk
1
2
安装完成后查看java是否成功

java -version
1

接下来下载eclipse，到官网https://www.eclipse.org/downloads/packages/找到eclipse for c/c++版本并下载eclipse-cpp-2023-09-R-linux-gtk-x86_64.tar.gz

下载完成后对其进行解压，

tar -zxvf eclipse-cpp-2023-09-R-linux-gtk-x86_64.tar.gz
1
解压完成后进入eclipse文件夹，运行eclipse可执行程序

cd eclipse
sudo ./eclipse 
1
2
为了使用方便最好创建一个程序的快捷方式

cd /usr/share/applications/
sudo nano eclipse.desktop
1
2
在弹出来的编辑窗中，修改eclipse执行文件的路径exec和图标路径icon, Ctrl+O 保存更改，然后按下 Ctrl+X 退出编辑器

[Desktop Entry]
Name=Eclipse
Type=Application
Exec=/home/Downloads/eclipse/eclipse
Terminal=false
Icon=/home/Downloads/eclipse/icon.xpm
Comment=Integrated Development Environment
NoDisplay=false
Categories=Development;IDE;
Name[en]=Eclipse
1
2
3
4
5
6
7
8
9
10
使用命令将新的桌面快捷方式添加到应用程序列表

sudo desktop-file-install eclipse.desktop
1
这样就能在ubuntu所有应用中找到eclipse的快捷图标

eclipse配置cuda环境
安装cudatoolkit-11.4

sudo apt-get update
sudo apt-get install cuda-toolkit-11-4
1
2
安装完成后，使用vim或者其他编辑器在.bashrc中配置cuda的系统环境变量

cd ~
vim .bashrc
1
2
在文档的开始添加如下信息，表示cuda的主路径，bin所在路径和lib所在路径，要注意的是，把最新安装的cuda路径放在$PATH之前，这样系统检测的版本才会是11.4

export CUDA_HOME=/usr/local/cuda-11.4/:$CUDA_HOME
export PATH=/usr/local/cuda-11.4/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-11.4/lib64:$LD_LIBRARY_PATH
1
2
3

退出编辑器，使用命令使其生效

source .bashrc
1
查看cuda版本是否正确

nvcc -V
1

官方提示cuda11之后是使用nsight eclipse的插件，为了在eclipse中安装这个插件，根据官网的操作指南：https://docs.nvidia.com/cuda/nsightee-plugins-install-guide/index.html

我们在/usr/local/cuda-11.4/bin中找到nsight_ee_plugins_manage.sh

进入该路径下，根据官网指示执行eclipse插件安装

cd /usr/local/cuda-11.4/bin
./nsight_ee_plugins_manage.sh  install  /home/Downloads/eclipse/
1
2
由此完成nsight eclipse plugins的安装

aarch64交叉编译器
使用命令安装aarch64下所需的gcc和g++

sudo apt-get install gcc-aarch64-linux-gnu
sudo apt-get install g++-aarch64-linux-gnu
1
2
使用命令查看安装版本

aarch64-linux-gnu-gcc -v
1


aarch64-linux-gnu-g++ -v
1

使用aarch64的gcc和g++分别编译如下的hello world，然后将可执行文件放到Jetson板子上运行
ubuntu上新建hello.c文件，代码如下

#include <stdio.h>

int main(void)
{
	printf("Hello World!-c \n");
}

1
2
3
4
5
6
7
cd到hello.c所在路径，使用aarch64-linux-gnu-gcc编译器编译

aarch64-linux-gnu-gcc hello.c -o hello_c
1
将输出的文件hello_c放到嵌入式板子上运行

./hello_c
1
运行结果显示 Hello World!-c 即成功。同样的道理，.cpp文件的编译使用aarch64-linux-gnu-g++进行编译和验证

eclipse配置cuda交叉编译环境
新建一个cuda工程，如图所示，后续的选择默认

右键该工程，添加一个.cu文件，粘贴如下的测试代码。

#include <iostream>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>


__global__ void hello_from_gpu()
{
    printf("Hello World from the GPU!\n");
}

int main(void)
{
    hello_from_gpu<<<2, 4>>>();
    cudaDeviceSynchronize();
    return 0;
}

1
2
3
4
5
6
7
8
9
10
11
12
13
14
15
16


然后，进入属性中对其进行设置，主要是修改CUDA Toolkit中cuda工具的路径，交叉编译的目标物和架构，使用的编译器等如图

Build project，如图所示，文本划线报错不用管

最后，将生成的文件拷贝到嵌入式机器中，输入指令

cd <文件所在目录>
./ArmCrossTest
1
2
由此完成交叉编译环境的搭建和测试
————————————————

                            版权声明：本文为博主原创文章，遵循 CC 4.0 BY-SA 版权协议，转载请附上原文出处链接和本声明。
                        
原文链接：https://blog.csdn.net/weixin_43956164/article/details/133814366
