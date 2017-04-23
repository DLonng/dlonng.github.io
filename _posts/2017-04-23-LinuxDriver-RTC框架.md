---
layout:     post
title:      "LinuxDriver-RTC框架"
subtitle:   "LinuxDriver RTC s5pv210"
date:       2017-04-23 11:00:00
author:     "陈登龙"
header-img: "img/post-bg-unix-linux.jpg"
catalog: true
tags:
    - LinuxDriver
---


# LinuxDriver-RTC 框架
***
> 版权声明：本文为 cheng-zhi 原创文章，可以随意转载，但必须在明确位置注明出处！ 

RTC 驱动是一个学习 Linux 下 Platform-Device-Bus 的一个很好的学习模块，虽然网上已经有许多比较好的介绍 RTC 的文章，但是这里为了记录我的学习过程，还是写下来吧。

#### RTC 驱动模型框图

![RTC框架.png](http://upload-images.jianshu.io/upload_images/4613385-3cd47d569d3279d3.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)

RTC 框架的核心是中间的 RTC 核心层，该核心层对 App 提供了字符设备的操作接口，对驱动层提供了注册的 class 类，相当与 App 和驱动的一个中间抽象层，在计算机编程中，许多问题都可以通过增加一个**中间抽象层**的思想来解决。

#### Linux 内核中的 RTC 相关文件

在Linux内核中，RTC 驱动有 3 层实现代码，他们分别对应上面 RTC 框图的 3 层架构：
* 用户接口层：rtc-dev.c，rtc-sysfs.c，rtc-proc.c
* 中间层(适配层)：rtc-lib.c，interface.c，class.c
* 底层驱动层：rtc-xxx(s3c).c

它们均在 drivers/rtc/ 目录下。

#### 文件作用
用户接口层
* rtc-dev.c：定义了字符设备的操作接口，例如，read，open 等。
* rtc-sysfs.c：定义了操作 sysfs 目录下文件的操作接口，例如，rtc_sysfs_show_name 等。
* rtc-proc.c：定义了操作 proc 文件系统下文件的操作接口，例如，rtc_proc_show

中间层
* rtc-lib.c：提供了操作 rtc 时间的一些工具函数，这些函数在 interface.c 中会被使用，例如，rtc_tm_to_ktime 等。
* interface.c：这里定义了操作 RTC 的一些函数，主要用于在 ioctl 中调用。
* class.c：为底层的具体 RTC 驱动提供注册接口，例如，底层的 RTC 驱动的 probe 函数会调用 class.c 中的 rtc_device_register 函数在将自己注册到 RTC 核心中。

底层驱动层：
* rtc-(xxx)s3c.c：具体平台的 RTC 驱动代码，在代码中的 xxx_rtc_probe 函数中，将自己注册到 RTC 核心层中。

#### 理解的最佳实践
强烈建议你使用 SourceInsight 来查看某一个平台的 RTC 相关的文件，以此来更加深刻的理解 RTC 的模型。

#### RTC 在 Android 中的体现
Android 中有一个基于 RTC 的 Alarm 设备驱动，目前能力不够，后续学到再记录吧。

#### About Me
* cheng-zhi：自动化专业，喜欢 Android，不只是 App。
* Github：[cheng-zhi](https://github.com/cheng-zhi)
* 个人主页：[cheng-zhi](https://cheng-zhi.github.io/)