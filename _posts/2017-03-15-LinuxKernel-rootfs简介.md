---
layout:     post
title:      "LinuxKernel-rootfs简介"
subtitle:   "S5PV210 Kernel rootfs "
date:       2017-03-15 18:00:00
author:     "陈登龙"
header-img: "img/post-bg-linuxkernel.jpg"
catalog: true
tags:
    - LinuxKernel
---

# LinuxKernel-rootfs简介

**一，rootfs简介**

**1.什么是rootfs呢？**

rootfs就是内核启动后**挂载的第一个文件系统**，包含Linux系统完整启动所需要的**目录结构**和**配置文件**，另外rootfs也是其他的文件系统**挂载**到Linux中使用的**载体**。

**2.为什么需要rootfs？**

因为Linux完整启动需要进入用户态操作环境，用户的数据都保存在文件系统上。
因为Linux内核提供了VFS机制，所以有了rootfs就可以挂载其他类型的fs。

**二，rootfs的形式**

**1.要求**

**(1).** rootfs必须能够被kernel找到并加载，rootfs可以保存在介质中。

**(2).** rootfs的格式必须能被kernel识别，也就是说内核需要在make menucofig中配置使用的fs类型。

**(3).** rootfs必须包含基本的目录结构和启动配置文件。

**2.rootfs的基本结构**

``` nimrod
/bin 存放二进制可执行文件
/dev 存放设备文件
/etc  存放系统配置文件
/home 用户主目录
/lib  存放动态库
/sbin  存放系统管理员的可执行文件
/tmp  存放公用的临时文件
/root 系统管理员主目录
/mnt 文件系统挂载目录
/proc 虚拟文件系统，可以直接访问这个目录中的文件来获取系统信息
/var 某些大的文件的溢出区域
/usr 最大的目录，app和文件几乎都在这个目录
```
一般来说：**只有/bin, /dev, /etc, /lib, /proc, /var, /usr这些目录是必须的**，其他是非必须的。

**三，制作rootfs**

你可以参考我写的[这篇文章][1]


  [1]: http://blog.csdn.net/qq_22075977/article/details/54577018