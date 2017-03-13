---
layout:     post
title:      "LinuxKernel-基本概念1"
subtitle:   "S5PV210 Kernel"
date:       2017-03-13 11:00:00
author:     "陈登龙"
header-img: "img/post-bg-linuxkernel.jpg"
catalog: true
tags:
    - LinuxKernel
---

# LinuxKernel-基本概念1

**一，Linux内核特性**

Linux是一款开源的操作系统，具有很强的可移植性，支持多种硬件平台，有很强的网络协议功能，并且是一种具有多任务，多用户的处理能力的系统，整个Linux内核采用模块化的设计。

这是Linux的总体框架：
![Frame][1]


**二，Linux内核子系统**

Linux内核有**5大子系统**：
1.进程管理子系统

2.内存管理子系统

3.虚拟文件系统

4.网络协议子系统

5.设备管理子系统

如下图：
![SubSys][2]

我们在**驱动开发阶段，最常接触的是设备管理子系统**。

**三，如何获取Linux内核**

1.直接从官网下载：www.kernel.org

2.从购买的芯片厂商拿到提供的内核源码，一般都是已经移植好的。


**四，Linux内核源码目录**

我们拿到一个Linux源码后，首先需要了解的就是目录的结构，分析每个目录的具体作用，帮助我们进一步理解Linux的组成。

**1.平台相关目录树**

Linux中与**平台相关**的目录都在**arch**目录下：
mach-xxx：某块开发板
plat-xxx：某个芯片平台

**2.平台无关目录树**

在Linux中，**除了arch目录之外都是平台无关目录**，常用的目录有下面这些：

block：块设备的算法

kernel：内核相关

crypto：加密算法，可以直接应用在裸机，不依赖第三方库

Documentation：官方文档

firmware：固件

drivers：所有的驱动

init：系统初始化目录

ipc：进程间通信

mm：内存

net：网络协议

**3.内核源码开发常用的头文件include格式**

内核代码中常见有下面几种引用头文件的方式：

``` c
//cpu体系结构相关的头文件
#include <asm/xxx.h>

//平台无关的头文件
#include <linux/xxx.h>

//一块芯片平台相关的头文件
#include <plat/xxx.h>

//一块开发板相关的头文件
#include <mach/xxx.h>
```

来看由一个头文件引出的问题：
**下面的两个include分别到那个目录去查找对应的xxx.h头文件？**

``` c
#include <linux/xxx.h>
```
答案：查找的目录是 **include/linux/xxx.h**

``` c
#include <mach/xxx.h>
```
答案：查找的目录是 **arch/arm/mach-某个开发板/include/mach/xxx.h**


在内核中有这么多的文件，难免有重复的文件名，在C语言中重复的文件在编译是是会报错的，那么内核如何解决这个问题呢？
内核的解决方法：内核相当于一个代码仓库，根据用户的配置选择，**每次编译时只会选择编译一个硬件平台的相关代码**，其余平台的代码就不会被编译，因此就不会出现重复文件的问题，这些编写规则是由内核的编译系统来决定的，关于内核编译系统，我们后面介绍。

  [1]: https://cheng-zhi.github.io/img/LinuxKernel/post-2017-3-13-LinuxFrame.png
  [2]: https://cheng-zhi.github.io/img/LinuxKernel/post-2017-3-13-LinuxSubSys.png
