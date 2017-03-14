---
layout:     post
title:      "LinuxKernel-内核配置路线"
subtitle:   "S5PV210 Kernel Kconfig Makefile Menuconfig"
date:       2017-03-14 15:00:00
author:     "陈登龙"
header-img: "img/post-bg-linuxkernel.jpg"
catalog: true
tags:
    - LinuxKernel
---


# LinuxKernel-内核配置路线

**一，简介**

上次我们简要介绍了内核源码树的基础，它可以说是我们通向内核的**第一张地图**，本次我们来介绍了解内核的**第二张**地图：**内核的基本配置路线**。

**二，Kconfig**

内核提供了一套编译系统，由Kconfig来配置make menuconfig的菜单，菜单会自动配置主目录下的.config，然后Makefile根据这个.config来编译内核。

主Kconfig如下：

``` bash
  1 #
  2 # For a description of the syntax of this configuration file,
  3 # see Documentation/kbuild/kconfig-language.txt.
  4 #
  5 mainmenu "Linux/$ARCH $KERNELVERSION Kernel Configuration"
  6 
  7 config SRCARCH
  8     string
  9     option env="SRCARCH"
 10 
 11 source "arch/$SRCARCH/Kconfig"
```
介绍3个Kconfig的主要命令：

**1.source**

通过source命令包含具体架构下的Kconfig，$SRCARCH = arm，source命令类似于include

**2.menu endmenu**

定义一个菜单条目

**3.config**	

配置这个菜单的信息

我们来手动添加一个菜单：我们在**arch/arm/Kconfig**中添加了一个**System info**，并添加了一个**名称为ABC的config**，如下：

``` mel
 220 menu "System Info"
 221 config ABC
 222     bool "This is a test config"
 223     help
 224         Support test info
 225 endmenu
```

然后我们再次进行：make menuconfig，可以看到我们新添加的System Info了：
![SysInfo][1]

进入SystemInfo:
![SysInfo2][2]

我们按**空格将它选择编译到内核**，然后**打开主目录下的.config**，搜素System Info, 可以看到已经被旋转编译到内核了：

``` vala
#
# System Info
#
CONFIG_ABC=y
```

通过这次的配置，我们可以发现，Kconfig，make menuconfig, .config, Makefile是存在相互联系的。


**三，如何在内核中查找配置信息**

**1.如何由make menuconfg中的配置信息如何找到Kconfig和Makeflie中对应的信息？**

例如我们如何根据Samsung SoC serial support的提示信息，找到对应的Kconfig和Makefile呢？
![Soc][3]
我们注意开头：**CONFIG_SERIAL_SAMSUNG -> config SERIAL_SAMSUNG**

下面我们在主目录下查找含有Samsung SoC serial support的文件：

``` gradle
grep -nR "Samsung SoC serial support"
```
可以找到它在： **drivers/tty/serial/Kconfig:441:tristate "Samsung SoC serial support"** 这个文件中，我们打开这个文件，查找SERIAL_SAMSUNG:
![SerialSam][4]


这就是它的Kconfig,找到了Kconfig，我们就可以利用SERIAL_SAMSUNG这个宏开关，在同级目录的Makefile中定位到编译的文件了，当然你通过menuconfig的帮助信息也可以直接看到，这里只是帮你加深印象：
![Samsung][5]



当然也可以根据Makefile中的宏来找到对应的Kconfig信息，如果你找不到对应的Kconfig信息，那么这个宏应该是作用在对应的.c文件中或者.h文件中用来进行预处理，例如DEBUG类的宏定义，也就是说CONFIG_XXX的宏的来源位置是下面这2个地方：

**1.具体的Kconfig中的定义**

**2.对应的C文件中的预处理定义，常见用于debug**



  [1]: https://cheng-zhi.github.io/img/LinuxKernel/post-2017-03-14-SysInfo.png
  [2]: https://cheng-zhi.github.io/img/LinuxKernel/post-2017-03-14-SysInfo2.png
  [3]: https://cheng-zhi.github.io/img/LinuxKernel/post-2017-03-14-SamSoc.png
  [4]: https://cheng-zhi.github.io/img/LinuxKernel/post-2017-03-14-SerialSam.png
  [5]: https://cheng-zhi.github.io/img/LinuxKernel/post-2017-03-14-Samsung.png
