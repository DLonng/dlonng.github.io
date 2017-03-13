---
layout:     post
title:      "LinuxKernel-内核配置编译"
subtitle:   "S5PV210 Kernel"
date:       2017-03-13 19:00:00
author:     "陈登龙"
header-img: "img/post-bg-linuxkernel.jpg"
catalog: true
tags:
    - LinuxKernel
---

# LinuxKernel-内核配置编译

**一，简介**

本次，我们为S5PV210编译可运行的内核，同时熟悉内核编译的基本流程。

**二，内核下载**

我们从 www.kernel.org 下载linux3.0.8的内核源码，然后解压，注意最好使用root用户解压：

``` nginx
tar -xjvf kernel_name
```

**三，内核配置**

**1.** 在配置内核之前，我们先对内核的编译系统有一个大致的了解。

内核总的编译流程如下：

``` haskell
Kconfig -> make menuconfig -> .config -> Makefile
```

**Kconfig**：是配置信息的清单

**make menuconfig**：是根据Kconfig的信息启动菜单配置界面

**.config**：是当前内核使用的配置文件

**主目录Makefile**：在主目录下的Makefile是核心Makefile，比较复杂，平台无关，通用，但是又必须指定的体系结构，因此，主目录的Makefile必须跟arch/具体体系结构$(hdr-arch)/Makefile相关。

我们打开主Makefile，查看下面这几行：

``` ruby
#指定当前编译架构，默认x86
195 ARCH        ?= $(SUBARCH)
	
#指定当前交叉编译工具链
196 CROSS_COMPILE   ?= $(CONFIG_CROSS_COMPILE:"%"=%)
	
#当前源码架构，就是ARCH的值
200 SRCARCH     := $(ARCH)

#当前Linux运行的硬件平台
232 hdr-arch  := $(SRCARCH)

#Linux的包含目录
360 LINUXINCLUDE    := -I$(srctree)/arch/$(hdr-arch)/include \
361                    -Iarch/$(hdr-arch)/include/generated -Iinclude \
362                    $(if $(KBUILD_SRC), -I$(srctree)/include) \
363                    -include include/generated/autoconf.h

#Linux引用的Makefile的目录
484 include $(srctree)/arch/$(SRCARCH)/Makefile
```


**子目录Makefile**：比较简单，就是定义了**obj-y, obj-m, obj-空**，这些编写条件

**2.** 环境配置

编辑主目录下的Makefile，修改架构平台和工具链

``` ruby
#指定当前编译架构，默认x86
195 ARCH        ?= arm
	
#指定当前交叉编译工具链
196 CROSS_COMPILE   ?= arm-linux-
```



**3.** 内核具体配置

我们先选取默认配置文件，在Linux中提供了一些硬件平台的默认配置文件，他们在下面的目录：

``` gradle
arch/arm/configs/xxx_defconfig
```

我们的硬件是S5PV210,因此我们选择：**s5pv210_defconfig**，将它拷贝主目录(修改内核文件，最好进行备份)：

``` gradle
cp arch/arm/configs/s5pv210_defconfig ./.config
```
实际工作过程中，还要在xxx_deficonfig之上进行修改，使得它适用你的实际产品	

然后运行下面的命令来启动配置菜单：

``` go
make menuconfig
```
如果你没有安装**ncurses**库，那么你需要使用apt安装，否则不能启动配置菜单，安装方法百度有。如果运行成功会出现下面的界面：
![MenuConfig][1]

这3步完成后，执行下面的命令编译内核：

``` go
make uImage
```

**不建议直接使用make命令**，应为Linux的make命令相当与下面的2条命令：

``` vala
#编译(obj-y),
make Image
#编译(obj-m)
make modules
```
我们这里只编译uImage，编译过程不算很长，最后你会出现提示**缺少mkimage**这个错误，这个是用来将zImage转换成uImage的工具，在**uboot/tools**目录下有，你可以拷贝到**/bin/**下以便于以后直接使用。
拷贝之后，重新make uImage即可，最后你会在**arch/arm/boot/**下得到**uImage**文件。

这里简单介绍下vmlinux, Image,zImage,uImage的区别，因为他们在编译内核过程中都会生成。

**vmlinux**：ELF文件，不能在开发板执行

**Image**：由vmlinux通过objcopy生成，文件较大，不适合下载执行

**zImage**：压缩过的Image，带自解压功能, 但是需要启动条件，引导比较麻烦

**uImage**：经过mkimage工具处理，是uboot可以引导的镜像

**4.** 运行内核

将uImage拷贝到/tftpboot下，启动开发板，进入uboot，使用tftp下载uImage：

``` nginx
tftp 0x20000000 uImage
```

然后直接执行内核：

``` nginx
bootm 0x20000000
```
可以**看到启动信息即可表明配置成功，即使会卡住**，这就说明默认的s5pv210_defconfig并不能完全适用于我们当前的开发板，所以后期需要**对.config进行修改**，但是通过这次实验我们至少熟悉了内核配置和编译的具体流程，为以后内核的移植和驱动开发打下基础。


  [1]: https://cheng-zhi.github.io/img/LinuxKernel/post-2017-03-13-MenuConfig.png
