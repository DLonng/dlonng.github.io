---
layout:     post
title:      "原理图分析-NandFlash"
subtitle:   "Tiny210 PCB NandFlash"
date:       2017-03-02 23:00:00
author:     "陈登龙"
header-img: "img/post-bg-apple-event-2015.jpg"
catalog: true
tags:
    - 原理图分析
---


# 原理图分析-NandFlash

这次我们简单分析下S5PV210的NandFlash原理图和时序要求。

**一，NandFlash原理图**

如下，S5PV210使用的NandFlash型号是**K9K8G08U0B**：
![NandFlash][1]


**二，NandFlash引脚介绍**

在K9K8G08U0B的数据手册上有引脚的详细介绍，这里就翻译下：

如图：
![PIN][2]


**IO0 - IO7**：可以发送数据，地址，命令

**CLE**：COMMAND LATCH ENABLE 命令锁存使能，当CLE为高时，IO0 - IO7发送命令

**ALE**：ADDRESS LATCH ENABLE 地址锁存使能，当ALE为高时，IO0 - IO7发送地址

**如何发送数据**：当CLE和ALE都为低时，IO0 - IO7发送数据

**CE**：片选，低电平有效

**RE**：读使能，低电平有效,当为低电平时，从NandFlash中读取数据

**WE**：写使能，低电平有效当为低电平时，向NandFlash中写入数据

**WP**：写保护，低电平有效
     
**R/B**：状态引脚，B低电平有效。 高电平Ready就绪，低电平Busy忙碌。


**三，NandFlash的基本操作**

这里以SOC(S5PV210)为中心来操作NandFlash。

这里只是简单介绍下操作的时序，没有太严格要求，在我们编写NandFlash的操作函数的时候，我们需要看手册后面详细的步骤。

**1.发命令给NandFlash**

时序图：
![SendCMD][3]

CE选中 -> CLE拉高 -> IO[0 - 7]放入命令 -> WE上升沿发出命令



**2.发地址SOC给NandFlash**

时序图：
![SendAddr][4]

CE选中 -> ALE拉高 -> IO[0 - 7]放入地址 -> WE上升沿发出地址
时序图：



**3.写数据到NandFlash**

时序图：
![WriteData][5]
CE选中 -> WE拉低(ALE,CLE拉低) -> IO[0 - 7]放入数据 -> WE上升沿写入数据到NandFlash

**2.从NandFlash读数据**

时序图：
![ReadData][6]
CE选中 -> RE拉低(ALE,CLE拉低) -> IO[0 - 7]放入数据 -> RE上升沿读入数据到SOC


**四，NandFlash的时序要求**

S5PV210发出的信号必须满足NandFlash手册上的时序要求，才能操作NandFlash，如何设置时序呢？

下面是一种**设置寄存器中时序**的**基本思路**：

**1.**看SOC(例如S5PV210)手册，查找要设置的寄存器描述中时序参数的名称，然后搜索它，并弄清该时序参数的含义。

**2.**看芯片外设手册，确定这个参数的取值范围。

**3.**根据寄存器中的计算方法来计算要配置的时序的值，必须满足芯片手册的取值范围，不能过大或者过小。

在这个过程中，请利用PDF阅读器的**搜索**功能，可以提高效率。




  [1]: https://cheng-zhi.github.io/img/PCB/NandFlash/post-2017-03-02-NandFlash.png
  [2]: https://cheng-zhi.github.io/img/PCB/NandFlash/post-2017-03-02-PIN.png
  [3]: https://cheng-zhi.github.io/img/PCB/NandFlash/post-2017-03-02-SendCMD.png
  [4]: https://cheng-zhi.github.io/img/PCB/NandFlash/post-2017-03-02-SendAddr.png
  [5]: https://cheng-zhi.github.io/img/PCB/NandFlash/post-2017-03-02-WriteData.png
  [6]: https://cheng-zhi.github.io/img/PCB/NandFlash/post-2017-03-02-ReadData.png