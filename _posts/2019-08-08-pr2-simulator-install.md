---
title: PR2 Simulator 安装
date: 2019-08-07 20:00:00
---
# PR2_Simulator
***
> 版权声明：本文为 {{ site.name }} 原创文章，可以随意转载，但必须在明确位置注明出处！

学习完 ROS Wiki 的基础教程后，我下一步选择学习 PR2 模拟器：
```
http://wiki.ros.org/pr2_simulator/Tutorials
```

于是就按照官网找到这个教程，但是要先安装 PR2 和 gazebo_ros 包才可以，这里记录下自己的安装过程和踩过的坑。
## 一、安装 gazebo_ros_pkgs
运行教程中的第一行命令：
```
$ roslaunch gazebo_ros empty_world.launch
[empty_world.launch] is neither a launch file in package [gazebo_ros] nor is [gazebo_ros] a launch file name
The traceback for the exception was written to the log file
```
这个原因是 gazebo_ros_pkgs 未安装，那就按照 gazebo 给的安装[教程](http://gazebosim.org/tutorials?tut=ros_installing)来安装它：
```
sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control
```
可是期间出现了依赖问题，我就换 Aptitude 安装来自动安装依赖关系：
```
sudo apt install aptitude
```
然后重新安装 gazebo-ros 包：
```
sudo aptitude install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control
```
安装完后，又出现 rosrun 命名找不到，我猜想应该是安装过程中 aptitude 勿删除了，不过没关系，重新安装下 ROS_Base 就行了：
```
sudo apt-get install ros-kinetic-ros-base
```
测试下：
```
source ~/catkin_ws/devel/setup.bash
roscore &
rosrun gazebo_ros gazebo
```
出现 GUI 界面即安装成功：

<div  align="center">
<img src="{{ site.url }}/images/ros/PR2/gazebo.png"/>
</div>

## 二、安装 PR2 模拟器
因为 zsh 不支持通配符安装，所以要先在 .zshrc 末尾加上 `setopt nomatch`，然后就可以使用下面的方式安装 PR2 了：
```
sudo apt-get install ros-kinetic-pr2-*
```
安装需要挺久，建议找个好点的网速下载。
## 三、测试 PR2
先启动 roscore：
```
roscore
```
一次性加载 PR2 到 Gazebo：
```
roslaunch pr2_gazebo pr2_empty_world.launch
```
加载成功如下：

<div  align="center">
<img src="{{ site.url }}/images/ros/PR2/empty_pr2.png"/>
</div>

然后启动键盘控制节点，测试是否能够控制 PR2 移动和旋转：
```
roslaunch pr2_teleop teleop_keyboard.launch
```
利用键盘把机器人移动到面前，搞定了！

<div  align="center">
<img src="{{ site.url }}/images/ros/PR2/key_pr2.png"/>
</div>

> {{ site.prompt }}

<div  align="center">
<img src="{{ site.url }}/images/wechart.jpg" width = "200" height = "200"/>