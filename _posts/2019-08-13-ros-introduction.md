---
title: ROS 基础概念 - Introduction
date: 2019-08-12 20:00:00
---
# ROS 基础概念 - Introduction
***
> 版权声明：本文为 {{ site.name }} 原创文章，可以随意转载，但必须在明确位置注明出处！

## 1、什么是 ROS ?
ROS（Robot Operating System）是一个开源的机器人操作系统，提供比如：硬件抽象，底层设备控制，进程间消息通信以及包管理等功能，ROS 还提供一些用于编写，编译，运行代码的工具和库文件。

就像开发上层手机应用的 Android 框架概念类似，你也可以把 ROS 理解为「机器人框架」，使用这个框架可以方便我们学习机器人技术，不用重复造轮子。

业界类似于 ROS 这样的机器人框架还有挺多的，比如：Player，YARP，Orocos 等等。

在实时性方面，ROS 不是一个实时框架，但是可以将实时性代码集成到 ROS 框架中，比如 PR2 机器人使用 pr2_etherCAT 系统来保证 PR2 的实时性。

## 2、ROS 的目标是什么？
ROS 的目标不是成为最好的，具有最多功能的框架。相反，ROS 的主要目标是支持机器人研究和开发中的代码重用，提高效率。

ROS 系统的可执行文件（Node）可以单独设计并解耦，这种设计使得 ROS 成为了一种分布式的框架，因为 ROS 的程序包耦合度比较小，所以可以轻松地共享和分发，这也符合 ROS 的主要目标 - 代码重用。

## 3、ROS 支持哪些系统？
ROS 目前仅在基于 Unix 平台的机器上运行，并且 ROS 的软件主要在 Ubuntu 和 Mac OS X 上进行过测试。

不过 ROS 是开源的，ROS 社区也在不断发展，目前很多开发人员也正在为 Fedora，Gentoo，Arch Linux 等平台努力提供系统支持，相信 ROS 以后的发展会越来越好。


> {{ site.prompt }}

<div  align="center">
<img src="{{ site.url }}/images/wechart.jpg" width = "200" height = "200"/>