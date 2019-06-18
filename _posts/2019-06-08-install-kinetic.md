---
title: ROS 入门 - Ubuntu 16.04 安装 Kinetic
date: 2019-06-08 20:00:00
---
# ROS 入门 - Ubuntu 16.04 安装 Kinetic
***
> 版权声明：本文为 {{ site.name }} 原创文章，可以随意转载，但必须在明确位置注明出处！

这两天把 ROS 官网的 20 讲初级教程搞定了，接下来写写文章，做为对学习过程中的一次总结。

这次来配置 Kinetic 安装环境，这是必备步骤，鉴于很多小伙伴不喜欢看官网的英文文档，所以龙哥这次就充当个翻译吧。

不过还是强烈建议你阅读英文文档，我一开始也不喜欢，但是我经常逼着自己读原汁原味的英文，现在已经养成看英文的习惯了。

使用 Kinetic 版本是因为它是官方的长期支持版本（LTS），支持到 2021 年，目前 K 版本的资料也很丰富。

这里也感谢之前的评论区老铁，提醒我用 K 版本，再次感谢！

### 1、配置软件源
我用的 Ubuntu 16.04，推荐用这个版本学习 Kinetic，坑比较少。

先来配置软件源，要求你会点 Linux 系统基础：
```shell
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

设置 Keys：
```shell
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```
如果出错，可以用下面这个：
```shell
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
```

### 2、安装
更新软件源：
```shell
sudo apt-get update
```

安装 Desktop-Full 版本，包含常用的所有工具，安装完后几乎一劳永逸：
```shell
sudo apt-get install ros-kinetic-desktop-full
```
等待一段时间，最好选择一个快一点的网，下载量好像是 500M 左右。

### 3、初始化 rosdep
在运行 ROS 系统之前，需要初始化 rosdep，主要功能可以简单理解为在必要的时候安装系统依赖：
```shell
sudo rosdep init
rosdep update
```

### 4、环境变量设置
如果你使用 bash，则键入如下命令：
```shell
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
如果你使用 zsh，使用下面的命令：
```
echo "source /opt/ros/kinetic/setup.zsh" >> ~/.zshrc
source ~/.zshrc
```

如果你安装了多个 ROS 版本，并且想在当前 Shell 终端中切换版本，可以直接 source 你想使用的那个版本的 setup.bash 或者 setup.zsh。

比如你想切换当前 Shell 使用 Kinetic 版本的 ROS：
```
source /opt/ros/kinetic/setup.bash
```

### 5、安装编译包的依赖
这一步主要是安装一些工具来管理你的 ROS 工作空间，直接键入下面的命令：
```shell
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

### 6、运行 ROS
在 Shell 中键入：
```shell
roscore
```
出现以下信息即启动成功啦：

<div  align="center">
<img src="{{ site.url }}/images/ros/ros_install/kinetic.png"/>
</div>

然后 Ctrl + C 即可退出 ROS，大功告成！

> {{ site.prompt }}

<div  align="center">
<img src="{{ site.url }}/images/wechart.jpg" width = "200" height = "200"/>