---
layout:     post
title:      "LinuxKernel-bootargs"
subtitle:   "S5PV210 Kernel bootargs "
date:       2017-03-15 10:00:00
author:     "陈登龙"
header-img: "img/post-bg-linuxkernel.jpg"
catalog: true
tags:
    - LinuxKernel
---

# LinuxKernel-bootargs

**一，嵌入式系统启动流程**

这是总体的启动路线：
**bootloader(uboot) -> LinuxKernel(uImage) -> RootFs(Init) -> App**

我们这次讨论的启动参数是uboot的环境变量，**uboot会将bootargs参数传递给kernel**，然后由kernel来解析，并根据参数的配置来启动Linux。

**二，重要的启动参数**

**1.bootargs**

bootargs必须要正常配置，它的主要功能是**挂载rootfs**，**配置ip**，**运行第一个程序**，**配置终端**等，如果配置错误，kernel会启动出错。

例如：

``` nix
bootargs=root=/dev/nfs nfsroot=192.168.1.1:/home/RootFs ip=192.168.1.2 init=/linuxrc console=ttySAC0,115200
```
**root**：表示根文件系统类型，如果是nfs，还需要指定nfs的路径，**如果RootFs在Flash上**，我们的启动参数要配置**mtdparts**字段，并且配置**内核需要支持mtd(menuconfig)**
		

``` aspectj
mtdparts=s5pv210-nand:1M(boot),5M(kernel),80M(rootfs),426M(usrfs)
```


**ip**：要给linux设置的ip

**init**： 第一个运行的程序

**console**：初始化控制台和bps

**mem**：限制linux内核使用的最大内存，没有指定就使用全部内存
	
``` ini
mem=128M
```


**2.bootcmd**

是uboot的默认启动参数，常见从tftp下载uImage启动或者从Flash中读取uImage到RAM中启动。在uboot中运行boot命令默认使用bootcmd作为uboot的启动命令。

``` ini
bootcmd=tftp 0x20008000 uImage ; bootm 0x20008000
```


**注意**：uboot的启动参数是bootcmd，linux的启动参数是bootargs

其他的uboot命令比较简单，可以参考[这篇文章][1]。


  [1]: http://blog.csdn.net/qq_22075977/article/details/54347651
