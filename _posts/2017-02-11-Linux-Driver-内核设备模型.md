---
layout:     post
title:      "LinuxDriver-内核设备模型"
subtitle:   "LinuxDriver"
date:       2017-02-11 11:00:00
author:     "陈登龙"
header-img: "img/post-bg-unix-linux.jpg"
catalog: true
tags:
    - LinuxDriver
---


**LinuxDriver-内核设备模型**

在内核驱动的编程过程中，经常会遇到一些内核概念，比如kobject，kset，这些变量常常定义在内核驱动模型结构体中，我们应该了解这些内核设备基础概念，这对于我们理解内核驱动的框架有很好的作用，下面就一一介绍这些基本概念。

**一，kobject**

struct kobject，先看看这个结构体的定义：

``` c
struct kobject
	/* 设备名称 */
	const char *name;
	/* 连接到kset建立层次结构 */
	struct list_head entry;
	/* kobject父节点 */
	struct kobject *parent;
	/* 这个kobject所属的kset */
	struct kset *kset;
	/* kobject类型 */
	struct kobj_type *ktype;
	/* 在sysfs中的目录 */
	struct sysfs_dirent *sd;
	/* 这个对象的引用计数 */
	struct kref kref;
	unsigned int state_initialized:1;
	unsigned int state_in_sysfs:1;
	unsigned int state_add_uevent_sent:1;
	unsigned int state_remove_uevent_sent:1;
	unsigned int uevent_suppress:1;
};
```

内核设计出这个结构的起因是为了实现**智能电源管理**，再后来因为这个结构的优点就在内核中使用起来。
kobject有几个重要的特点：
**1.一个内核设备一般有会有一个kobject对象**，如果该设备功能复杂，也可以包含多种类型的kobject。例如下面的字符设备的cdev结构：

``` c
struct cdev {
    /* 标识该设备的kobject对象 */
	struct kobject kobj;
	struct module *owner;
	/* 设备操作集 */
	const struct file_operations *ops;
	struct list_head list;
	/* 设备号 */
	dev_t dev; 
	/* 设备数目 */
	unsigned int count; 
};
```

**2.一个kobject对象一般是sysfs目录中的一个路径，是内核空间用来和用户空间交互的接口。**

**3.kobject有一个引用计数kref**，引用计数这个概念在内核中使用的非常多，你应该至少能够知道它有什么作用。引用计数经常使用在管理资源的释放方面，在一个对象被复制时，该对象的引用计数会自动加1, 在该对象一份拷贝释放后，该对象的引用计数会减1, 当该对象的**引用计数递减为0**时，内核**自动释放该对象**的内存。

与kobject操作相关的API，这里介绍2个：

``` c
/*
 * kobj : kobject指针
 * ktype : 定义kobject类型结构体的指针
 * 该函数用来分配一个kobject对象
 */
void kobject_init(struct kobject *kobj, struct kobj_type *ktype)  
```


``` c
/*
 * 该函数用来注册已经分配的kobject对象到内核中的Linux设备层次
 */
int kobject_add(struct kobject *kobj,  struct kobject *parent, const char *fmt, ...) 
```

在内核中的资源很多都需要**先分配，再注册**到内核中。



**二，kset**

先看看 struct kset 的定义：

``` c
struct kset {
	struct list_head list;
	spinlock_t list_lock;
	/* kobj对象 */
	struct kobject kobj;
	/* kset里面所有kobject的处理函数*/
	const struct kset_uevent_ops *uevent_ops;
};
```

kset是一组有类似性质的**kobject的集合**，一般来说，一个kset其实就是一个**子系统**，它是**sysfs的一个顶层目录**的特征。比如block子系统，各种总线bus子系统。



**三，sysfs**

sysfs是一个虚拟文件系统，但是与VFS是两个不同的概念，是**kobject的一个完整视图**，她提供丰富的内核与用户交互的接口，sysfs有取代proc的趋势。

下面是syfs的主要目录(不包括全部的目录)：

``` 
block：块设备，独立于连接的总线.
device：被内核识别的硬件设备，依照连接他们的总线进行组织.
bus：系统中用于连接设备的总线.
drivers：在内核中注册的设备驱动程序.
class：系统中设备的类型(声卡，网卡，显卡等)，同一类可能包含由不同总线练连接的设备，于是由不同的驱动程序驱动.
power：处理一些硬件设备电源状态的文件.
firmware：处理一些硬件设备固件的文件
```

你可以在一台Linux下，用下面的命令查看sysfs目录：

``` 
	ls /sys/
```


**四，总结**

这三个模型是构成sysfs目录的底层结构，了解他们对驱动开发有一定的帮助。下面是kobject，kset，ktype的一张关系图，sysfs目录就是由这样的关系一层一层构成的。
[sysfs底层结构图][1]

  [1]: https://cheng-zhi.github.io/img/post-2017-02-11-kernel-devices-mode.jpg
