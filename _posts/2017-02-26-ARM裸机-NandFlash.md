---
layout:     post
title:      "ARM裸机-NandFlash"
subtitle:   "ARM Tiny210 NandFlash"
date:       2017-02-26 20:00:00
author:     "陈登龙"
header-img: "img/post-bg-unix-linux.jpg"
catalog: true
tags:
    - ARM裸机
---


# ARM裸机-NandFlash

**一，Flash 简介**

**Flash**类似PC上的**硬盘**，用来**保存数据**，与RAM不同的是，Flash可以**永久保存**。

常见的Flash有：**Nor Flash，Nand Flash**，这两者有许多的不同点，但是在使用的时候你需要了解下面的两个不同：

1.**Nor Flash** 一般用来**存储程序**，而**Nand Flash** 一般用来**存储数据**。

2.**Nor Flash** 上常用**jffs2**文件系统，而**Nand Flash** 上常用**yaffs**文件系统。

本次介绍Nand Flash相关操作。

**二，Nand Flash的访问方式**

1.下面是**Tiny210**上的Nand Flash的原理图：
![Nand Flash][1]
可以看出，NandFlash只有8个数据IO，那么地址和命令如何发送呢？
Nand Flash有它的特殊性，为了减少芯片的引脚个数，并且使得系统很容易升级到更大的容量，**NandFlash的命令，地址，数据都通过8个IO口输入和输出**。在操作NandFlash时，**先传输命令**，**然后传输地址**，**最后读写数据**，期间要**检查Flash的状态**。

2.Nand Flash的**命令表**
![CMD][2]

**3.Nand Flash的地址序列**

我们在**发送地址的时候会用到这个地址序列**。
![AddrSeq][3]
不同型号的NandFlash的地址序列可能不同，我使用的是**K9K8G08U0A**，具有**5个地址序列**，你可以查看你的NandFlash的数据手册。


**三，Nand Flash控制器**

Nand Flash的访问比较复杂，但是通常我们的开发板都会带有**Nand Flash 控制器**来简化对它的访问。Nand Flash控制器提供了一系列的寄存器，例如**NFCONF，NFCONT，NFDATA等等**来简化访问，我们只需要操作这些寄存器即可控制NandFlash来进行相应的读写等操作了。
![RegMap][4]


**四，编写NandFlash模块函数**

在学习Nand Flash的时候，我们最好从头到尾编写操作NandFlash的函数，可以帮助我们提高学习底层的能力。
在编写Nand Flash API的时候，我们需要有对应型号的**Nand Flash数据手册，开发板数据手册，开发板原理图**，这3个文档必须要有，你需要有良好的**英文阅读能力**哦，所以重视英语吧。

在编写的过程中，主要是看Nand Flash数据手册，里面提供了所有操作的时序，是我们编写函数的基础，然后通过在开发板数据手册中找到对应Nand Flash控制寄存器的地址来控制Nand Flash，我们编写的**思路**就是：**利用Nand Flash数据手册提供的操作的时序来转换成C语言的函数。**

只要按照这个思路，一步一步即可编写出对应的C函数，下面是我编写的nand.c和nand.h，可以参考。
1.[nand.h][5]
2.[nand.c][6]

在编写的过程中，你一定要学会方法，**方法和思路是底层编程很重要的一点**。



  [1]: https://cheng-zhi.github.io/img/post-2017-02-26-NandFlash.png
  [2]: https://cheng-zhi.github.io/img/post-2017-02-26-NandFlashCMD.png
  [3]: https://cheng-zhi.github.io/img/post-2017-02-26-NandFlashAddrSeq.png
  [4]: https://cheng-zhi.github.io/img/post-2017-02-26-NandFlashRegMap.png
  [5]: https://cheng-zhi.github.io/code/NandFlashCtl/nand.h
  [6]: https://cheng-zhi.github.io/code/NandFlashCtl/nand.c
