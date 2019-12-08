---
title: ROS 初级 - 如何使用 rqt_console 调试日志？
date: 2019-12-08 12:00:00
---
# ROS 初级 - 如何使用 rqt_console 调试日志？
***
> 版权声明：本文为 {{ site.name }} 原创文章，可以随意转载，但必须在明确位置注明出处！

这篇文章主要介绍一个日志查看工具 rqt_console 和 roslaunch 命令的使用，在开始之前需要安装一个包：

```sh
sudo apt-get install ros-kinetic-rqt ros-kinetic-rqt-common-plugins ros-kinetic-turtlesim
```

把其中的 kinetic 替换为你自己的 ROS 版本，不过官方教程建议用 Kinetic 版本。

## 1、使用 rqt_console 调试日志

在 ROS 中当我们调试程序的时候经常会使用到日志 log，ROS 提供了 rqt_console 来输出一个节点的信息，我们还可以使用 rqt_logger_level 来改变日志的显示级别（由低到高）：Debug，Warn，Info，Error，Fatal。

比如当我们设置日志等级为 Warn，那么 ROS 就会输出 Warn，Info，Error，Fatal 这 4 种类型的日志，其他的同理，我记得 Android 和 ROS 的日志管理貌似有点类似。

下面来看下如何使用日志系统，我们先分别在 2 个终端中启动 ROS 的日志程序和 logger_level：

```sh
# rosrun 命令用来运行一个包中的节点
rosrun <package> <executable>
```

```sh
rosrun rqt_console rqt_console
```

<div  align="center">
<img src="https://dlonng.com/images/ros/8log_console.png"/>
</div>

再开启新的终端「Ctrl + Alt + T」：

```sh
rosrun rqt_logger_level rqt_logger_level
```

<div  align="center">
<img src="https://dlonng.com/images/ros/8log_level.png"/>
</div>

接着我们就可以启动小乌龟节点，并查看节点输出的日志消息了，默认的日志等级是 Info：

```sh
# 开启新的终端运行节点
rosrun turtlesim turtlesim_node
```

查看 rqt_console 程序，可以看到小乌龟节点输出的 x，y 坐标信息：

<div  align="center">
<img src="https://dlonng.com/images/ros/8log_xy.png"/>
</div>

我们来改变下输出的日志等级，左下角 Reflesh 一下，选择 Loggers 将 Info 改为 Warn，看看会不会输出新的日志：

<div  align="center">
<img src="https://dlonng.com/images/ros/8log_warn.png"/>
</div>

为了让小乌龟节点产生 Warn 的警告输出，我们使用 rostopic 命令来向小乌龟节点发送指令，让小乌龟撞到墙壁上去：

```sh
rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
```

这是可以看到 rqt_console 程序输出了 Warn 日志信息：

<div  align="center">
<img src="https://dlonng.com/images/ros/8log_wall.png"/>
</div>

是不是很容易呢？那尝试改变其他的日志等级试试吧。

> {{ site.prompt }}

<div  align="center">
<img src="https://dlonng.com/images/wechart.jpg" width = "200" height = "200"/>