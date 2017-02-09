---
layout:     post
title:      "LinuxDriver-字符设备驱动"
subtitle:   "LinuxDriver"
date:       2017-02-09 12:00:00
author:     "陈登龙"
header-img: "img/post-bg-unix-linux.jpg"
catalog: true
tags:
    - LinuxDriver
---


**LinuxDriver-字符设备驱动**

**一, Linux设备模型简介**

Linux下的驱动非常的多, 前辈已经为我们总结出了一套驱动模型, 我们在开发一种类型的驱动时, 应该先了解这种类型的驱动模型, 理解这种类型驱动的基本框架, 再进行开发就会有清晰的思路.

字符设备是Linux中非常常见的设备,例如触摸屏,LED等等..., 我们应该首先学会开发字符设备驱动的基本模式.

**二, 先试着运行一个字符设备驱动.**

1.以内核模块的形式编译字符驱动:

源文件: [memdev.c][1]

Makefile: [Makefile][2]

编译出的驱动模块.ko文件, 之后我们需要将这个文件下载到开发板中并插入到内核中运行: [memdev.ko][3]

2.下载文件到Tiny210开发板

我使用的是嵌入式的开发环境: ubuntu 16.10 + Tiny210(Linux-3.0.8) + UBoot + tftp +nfs.你需要根据你的实际环境来测试.

首先, 我们通过nfs服务器将上面的memdev.ko文件下载RootFs中, 如果你使用的是Windows, 则可以使用CRT串口连接工具,并使用rz命令将ko模块加载到内核.

插入命令
``` nginx
	insmod memdev.ko
```

然后, 我们需要创建字符设备文件
``` gradle
	mknod /dev/memdev c 主设备号 次设备号 
```
memdev是字符设备文件名, 可以自定义.

如何查看驱动程序的主设备号?
``` nimrod
	cat /proc/devices
```
次设备号可以自己**取小于255的非负数**, 这里取**次设备号为0**


3.编写应用测试驱动

向设备文件写入数据: [write_mem.c][4]
从设备文件读取数据: [read_mem.c][5]

注意: 如果你的RootFs里面没有安装必要的C库, 则你应该使用**静态编译**上面的两个C文件,否则会出现如下的错误:
``` stata
	/bin/sh:no found
```

静态编译的命令如下:

``` swift
	arm-linux-gcc -static write_mem.c -o write_mem
```

**还有一种解决方案**: 因为产生这个错误的原因是程序在运行时缺少必要的库,因此你可以查找要运行的应用需要的库,使用下面的命令:

``` stata
	arm-linux-readrf -d app_name
```
找到缺少的库的名称, 然后去找这个库文件并复制到下面的位置:

``` gradle
	/usr/lib/
```



  [1]: https://cheng-zhi.github.io/code/LinuxDriverMode/Driver/memdev.c
  [2]: https://cheng-zhi.github.io/code/LinuxDriverMode/Driver/Makefile
  [3]: https://cheng-zhi.github.io/code/LinuxDriverMode/Driver/memdev.ko
  [4]: https://cheng-zhi.github.io/code/LinuxDriverMode/App/write_mem.c
  [5]: https://cheng-zhi.github.io/code/LinuxDriverMode/App/read_mem.c
