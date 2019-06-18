---
title: ROS 入门 - 理解 ROS 节点
date: 2019-06-11 20:00:00
---
# ROS 入门 - 理解 ROS 节点
***
> 版权声明：本文为 {{ site.name }} 原创文章，可以随意转载，但必须在明确位置注明出处！

## Nodes
ROS Nodes 是一个使用 ROS 系统与其他节点通信的可执行文件。

Nodes 可以使用 ROS 客户端库来与其他节点通信，可以发布和订阅主题，还也可以提供和使用服务。

## 客户端库
ROS 的客户端库允许使用不同编程语言编写的节点能够互相通信，常用的有 2 个库：
- rospy = python client library
- roscpp = c++ client library

## 运行节点
前面说过，ROS Nodes 实际上是 ROS 系统的一个可执行文件，就类似我们 Window 系统中的 exe 文件一样，所以自然在 ROS 中就可以运行节点。

使用 rosrun 来运行一个节点，格式如下：
```shell
rosrun [package_name] [node_name]
```

来试着运行一下小乌龟节点：
```shell
rosrun turtlesim turtlesim_node
```
出现了：

<div  align="center">
<img src="{{ site.url }}/images/ros/nodes/node.png"/>
</div>
运行成功，我们可以使用 rosnode 来列出当前系统中运行着的节点：
```shell
rosnode list
```
输出如下：
```
/rosout
/turtlesim
```

> {{ site.prompt }}

<div  align="center">
<img src="{{ site.url }}/images/wechart.jpg" width = "200" height = "200"/>
