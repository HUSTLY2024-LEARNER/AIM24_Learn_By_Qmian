# ROSHow

一个订阅消息，发布消息的节点。

没有使用 catkin 工具，而是选择了比较基础的 cmake 配置。

完成了在 wsl-ubuntu-20.04 与 windows11 之间的通信任务。

下面是配置步骤

1. 安装自己的 ubuntu-20.04
2. 在 ubuntu-20.04 上配置开发环境
3. 在 windows11 上安装 foxglove
4. 测试通信，大功告成

## 安装自己的 ubuntu-20.04

这里不介绍如何安装 ubuntu-20.04，可以参考官方文档。
下面是几个可能的方法

1. 使用 wsl2 https://blog.csdn.net/weixin_44203681/article/details/114297151
2. 使用虚拟机软件(vmware) https://blog.csdn.net/qq_45657288/article/details/116084337

还有其他方法，只要能同时跑两个系统就行，这里不列出。

## 配置开发环境

保证网络通畅。

先安装 apt 加速软件。

默认 ubuntu 找不到网络上的 apt-fast 软件，要手动添加软件库

```bash
sudo add-apt-repository ppa:apt-fast/stable
```

然后更新软件库索引

```bash
sudo apt-get update
```

最后安装 apt-fast

```bash
sudo apt-get install -y apt-fast
```

安装过程中会出现三个需要选择的界面，  
首先是选择使用的软件包安装器，选择 apt-get  
第二个是选择使用的线程数目，保持 5 不变  
最后是禁用警告，选择 yes

安装完 apt-fast 之后，就可以愉快配置环境了：

```bash
sudo apt-get install vim git ssh net-tools \
	build-essential cmake gcc-10 g++-10 \
	wget curl ros-noetic-desktop-base \
	libtbb-dev libasan6 openvino gdb \
	ros-noetic-foxglove-bridge
```

切换 c/c++ 的编译器版本：

```bash
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-10 100
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-10 100
```

为 ros noetic 加上环境变量，用来寻找ros里面的工具。
使用你喜欢的编辑器打开 ~/.bashrc 文件，添加如下行：

```bash
source /opt/ros/noetic/setup.sh
```

然后运行

```bash
source ~/.bashrc
```

接下来自行安装依赖库 fmt 与 spdlog，然后单独编译 ROSHow 。

运气好的话就结束了。

## 在 windows11 上安装 foxglove

整个过程是假设机器人上面的系统是 ubuntu ，自己的电脑是 windows

利用 ros 将机器人的状态发送到自己电脑，实时显示的同时还能操控一些变量

下载地址 https://foxglove.dev/

安装完之后，配置中文，大功告成。

## 测试

在 ubuntu 上运行 ROSHow

打开一个终端，运行 roscore

```bash
roscore
```

再打开一个终端，运行 foxglove_bridge 

```bash
roslaunch foxglove_bridge foxglove_bridge.launch
``

再打开一个终端，执行 ifconfig 查看本机的 ip 地址

```bash
ifconfig
```

然后在 windows 上连接 foxglove 到 ubuntu  

选择 foxglove websocket 连接，将 localhost 替换为 ubuntu 的 ip 地址，建立连接

打开 ubuntu 终端，自行找到ROSHow编译的结果，运行

```bash
./LangYa_ROSHow_main
```

现在就可以使用 foxglove 查看并控制 ubuntu 上的 ROSHow 了。
