---
title: ROS 入门 - ROS Fuerte 环境搭建
date: 2019-06-07 15:00:00
---
# ROS 入门 - ROS Fuerte 环境搭建
***
> 版权声明：本文为 {{ site.name }} 原创文章，可以随意转载，但必须在明确位置注明出处！

> 安装 Kinetic 请查看下一篇文章！

最近跟导师聊天，他要求暑假把 ROS 系统学会，所以这里记录自己的学习过程，昨天搭好了 ROS 环境，今天就跟大家分享一下自己的搭建步骤。

### 1、选择 ROS 版本
我用的是 Fuerte 版本，其实初学没必要纠结版本，只是我有一本 ROS 的书用的是 Fuerte，所以我就安装它了，不过不同的版本安装过程大同小异。

我参考的官网安装教程：[ROS Fuerte Install](http://wiki.ros.org/fuerte/Installation/Ubuntu)

学习 ROS 一定要好好利用官网 [wiki.ros.org](http://wiki.ros.org) 上的教程。

### 2、安装 VMware10
因为是学习使用，所以在虚拟机里跑 Ubuntu 方便点，直接装真机有点麻烦，就不折腾了。

我用的 VMware10，官网下载地址：
```
https://download3.vmware.com/software/wkst/file/VMware-workstation-full-10.0.0-1295980.exe
```

直接复制链接到迅雷里面下载会很快，不要直接到官网下。

我的 Github 上有注册码软件，下载下来就有注册码了：[VMware10 注册码](https://github.com/DLonng/dlonng.github.io/releases/tag/1.0)

如果你使用 VirtualBox 虚拟机软件，官网已经提供了已经配置好的环境：[VirtualBox ROS](http://nootrix.com/downloads/#RosVM)，不过还是建议你自己动手配置一下。


### 3、下载 Ubuntu 12.04
ROS Fuerte 要求 Ubuntu 12.10 之前的版本，比较好的选择是 Ubuntu 12.04，下载地址:[Ubuntu 12.04](http://old-releases.ubuntu.com/releases/12.04.0/)

推荐用迅雷种子下载，比较快和稳定，我下载是 x64 版本：[ubuntu-12.04-desktop-amd64.iso.torrent](http://old-releases.ubuntu.com/releases/12.04.0/ubuntu-12.04-desktop-amd64.iso.torrent)

### 4、安装 Ubuntu 12.04
安装过程其实非常简单，网上的教程很多，这里就不详细说了，不想找的可以看看这篇：[VMware10 Install Ubuntu](https://blog.csdn.net/qq_21387171/article/details/43450303)

基本步骤是：新建虚拟机基本步骤 -> 导入 iso 镜像文件使用简易安装 -> 等待自动安装完成。

如果网速不好，给你 2 个快速安装的小技巧：
1. 安装过程开始不要选择联网下载程序
2. 语言包安装时间挺长，网速不好的话也可以跳过，以后开机还可以安装的。

注意：输入的 Root 密码不要忘记了，后面要用的。

### 5、安装 VMTools
Ubuntu 系统安装完成之后，为了以后使用方便，我们再安装一个 VMTools，VMWare 可以自行安装，如下点击安装 VMTools 即可：

<div  align="center">
<img src="{{ site.url }}/images/ros/ros_install/vmtools.png"/>
</div>


点击后，Ubuntu 会弹出 VMTools 的文件夹，我们直接把压缩包解压到桌面，然后打开 shell 进入 VMTools 文件夹，再使用超级权限执行安装即可：

<div  align="center">
<img src="{{ site.url }}/images/ros/ros_install/install_vmtools.png"/>
</div>

安装过程一路回车（yes）就行了，安装完重启，Ubuntu 系统就可以全屏，并且能执行和 Win 的复制粘贴功能了，非常方便学习。


### 6、安装 ROS Fuerte
我参考的官网安装步骤：[Ubuntu 12.04 Install ROS Fuerte](http://wiki.ros.org/fuerte/Installation/Ubuntu)

#### 6.1 配置软件源
「Ctrl + Alt + T」打开 Shell 终端，键入：
```shell
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'
```
#### 6.2 设置 keys
这一步是为软件源设置密码，直接继续键入：
```shell
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
```
#### 6.3 更新软件源
继续键入如下命令来更新刚刚配置的软件源：
```shell
sudo apt-get update
```

#### 6.4 桌面完整安装
新手推荐这种最简单的安装方式，虽然安装包比较大，一共 768 MB 左右，建议找个好的网来安装，但安装完以后就不需要再配置啥了，省的麻烦：
```shell
sudo apt-get install ros-fuerte-desktop-full
```
静静等待安装完成吧，如果最后提示安装有错误，那可以尝试重新安装，重新安装只会重装失败的安装包。

#### 6.5 配置 Shell 环境
如果你现在在 shell 中键入 roscore 会发现命令无法找到，那是因为我们还没有配置 ros 的环境变量，键入：
```shell
echo "source /opt/ros/fuerte/setup.bash" >> ~/.bashrc
```
这个命令意思是让 shell 知道到哪里去找 ros 程序，我们再键入：
```shell
. ~/.bashrc
```
这个命令相当于更新当前 shell 的环境变量，我们再次启动 ros：
```shell
roscore
```
ros 启动啦，环境搭建成功！
<div  align="center">
<img src="{{ site.url }}/images/ros/ros_install/roscore.png"/>
</div>

#### 6.6 安装 2 个小工具
为了以后更好的学习 ros，官网建议我们安装如下两个工具，具体是干嘛的，我们到时候用到再说吧：
```shell
sudo apt-get install python-rosinstall python-rosdep
```

### 7、总结
ROS Fuerte 的环境这就搭建完成了，其他版本搭建可能有些细微的差别，建议以 [ROS 官网](http://wiki.ros.org/Distributions)教程为主。

环境配置过程需要一点 Linux 命令基础，不熟悉的同学可以翻翻我之前写过的 Linux 基础文章，相信会对你有用的。

> {{ site.prompt }}

<div  align="center">
<img src="{{ site.url }}/images/wechart.jpg" width = "200" height = "200"/>
