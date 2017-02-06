---
layout:     post
title:      "Linux Driver Preparation"
subtitle:   " \"Linux Driver\""
date:       2017-02-04 12:00:00
author:     "Orange"
header-img: "img/post-bg-2015.jpg"
catalog: true
tags:
    - 生活
---

﻿**嵌入式Linux-驱动开发准备工作**

**一, 驱动学习步骤**
**1.理解驱动模型**
    开发一种驱动之前,首先需要找一个demo程序进行分析,然后总结出开发过程的思维导图, 再根据导图编写代码, 最后得到驱动程序的基本框架.

**2.实现硬件操作**
    我们需要先复习对应的裸机驱动,然后将裸机驱动代码移植到Linux驱动中.

**3.测试驱动程序**
    编写完成驱动后, 我们还需要编写应用程序进行测试, 这是必须的工作.
    
**注意:** 驱动学习的过程中, **不应该过多的去读内核源码**, 应该在驱动学习半年之后再读源码,
因为此时你已经对内核模块有了一定的了解, 可以理解内核代码的功能.
    
    
**二, 驱动程序访问硬件的特殊性**
驱动程序可以通过DMA和IO子系统来访问硬件.
**1.DMA**
概念: DMA是存在于外设中的一个硬件控制器, 它不需要CPU协助, 就可以在内存和外设之间搬运数据.
配置: 通过配置DMA Controller, 告诉DMA可以访问的内存地址, 然后CPU将要传给外设的数据写到该内存地址中, 最后DMA Controller就可以从该内存地址中取得数据.可以总结为下面两个步骤,约定DMA操作的内存地址,设置CPU的DMA Interrupt.

**2.IO子系统**
概念: 在嵌入式系统中, IO子系统是控制外设的有效手段,通过对端口进行0和1的操作,可以发送指令或者传递数据给外设.

使用IO子系统基本过程: 
 1. IO初始化
 2. 申请GPIO
 3. 设置输出状态
 4. 设备GPIO端口值
 5. 释放GPIO

**3.Linux 硬件访问技术**
访问流程: 访问硬件, 就是访问硬件设备的寄存器.

地址映射:  
静态映射, 即用户在内核启动前事先指定映射关系, 内核启动后会自动映射.查看CPU.c中: **struct map_desc**结构
动态映射, 使用**void* ioremap(PA, size)**来映射, 实际开发经常使用动态映射.

寄存器读写: Linux内核提供一套寄存器读写API.

    unsigned ioread8(void *addr)
    unsigned ioread16(void *addr)
    unsigned ioread32(void *addr)
    unsigned readb(address)
    unsigned readw(address)
    unsigned readl(address)


    unsigned iowrite8(u8 value, void *addr)
    unsigned iowrite16(u8 value, void *addr)
    unsigned iowrite32(u8 value, void *addr)
    unsigned writeb(unsigned value, address)
    unsigned writew(unsigned value, address)
    unsigned writel(unsigned value, address)

 
**三, Linux 驱动分类**
常规分类法:
**1.字符设备**
一般是以串行(字节)顺序访问,例如: 触摸屏, 鼠标, 按键, LED等.

**特点:** 以文件的形式访问

内核相关结构: **struct cdev**

**2.块设备**
一般是以扇区, 块等为单位进行读写访问, 例如: cdrom, flash等.Linux允许设备传送任意数目的字节, 因此**块和字符设备的区别仅仅是驱动程序和内核接口不同**.

**特点:** 
通过/dev/下的文件访问
与字符设备的不同在于管理数据的方式不同
有专门的接口, 块设备的接口必须支持mount
App一般通过fs来访问块设备

内核相关结构: **struct block_device**

**3.网络设备**
主要是以太网类的设备ethernet devices, 网络接口负责发送和接收数据报文.

**特点:**
通过**单独的网络接口**来访问
任何一个网络事务都通过一个网络接口, 即一个能够和其他主机交换数据的设备, 例如: 网卡, 回环(loopback).
内核调用一套和数据包传输相关的函数

内核相关结构: **struct net_device**

**4.杂项设备**
例如: 复合设备, 既有字符属性, 又有块属性等.

总线分类法:
 1. USB设备
 2. PCI设备
 3. 平台总线设备

**四, 内核基本调试方法**
 1. printk
 2. oops
 3. kprobe
 4. kcore
 使用方法会在后续更新.
