---
layout:     post
title:        "LinuxDriver-字符设备驱动模型"
subtitle:   "LinuxDriver"
date:        2017-02-10 12:00:00
author:     "陈登龙"
header-img: "img/post-bg-unix-linux.jpg"
catalog: true
tags:
    - LinuxDriver
---


**LinuxDriver-字符设备驱动模型**

**一, Linux设备模型简介**

Linux下的驱动非常的多, 前辈已经为我们总结出了一套驱动模型, 我们在开发一种类型的驱动时, 应该先了解这种类型的驱动模型, 理解这种类型驱动的基本框架, 再进行开发就会有清晰的思路.

字符设备是Linux中非常常见的设备,例如触摸屏,LED等等..., 我们应该首先学会开发字符设备驱动的基本框架.

**二, 先试着运行一个字符设备驱动.**

1.以内核模块的形式编译字符驱动:

源文件: [memdev.c][1]

Makefile: [Makefile][2]

编译出的驱动模块.ko文件, 之后我们需要将这个文件下载到开发板中并插入到内核中运行: [memdev.ko][3], 右键保存这个文件吧.

2.下载文件到Tiny210开发板

我使用的是嵌入式的开发环境: ubuntu 16.10 + Tiny210(Linux-3.0.8) + UBoot + tftp +nfs.你需要根据你的实际环境来测试.

首先, 我们通过nfs服务器将上面的memdev.ko文件下载RootFs中, 如果你使用的是Windows, 则可以使用**CRT**串口连接工具,并使用**rz**命令将ko模块加载到内核.

插入命令

``` 
	insmod memdev.ko
```

然后, 我们需要创建字符设备文件

``` 
	mknod /dev/memdev c 主设备号 次设备号 
```
memdev是字符设备文件名, 可以自定义.

如何查看驱动程序的主设备号?

``` 
	cat /proc/devices
```

次设备号可以自己**取小于255的非负数**, 这里取**次设备号为0**


3.编写应用测试驱动

向设备文件写入数据: [write_mem.c][4]
从设备文件读取数据: [read_mem.c][5]

注意: 如果你的RootFs里面没有安装必要的C库, 则你应该使用**静态编译**上面的两个C文件,否则会出现如下的错误:

``` 
	/bin/sh:no found
```

静态编译的命令如下:

``` 
	arm-linux-gcc -static app.c -o app
```

**还有一种解决方案**: 因为产生这个错误的原因是程序在运行时缺少必要的库,因此你可以查找要运行的应用需要的库,使用下面的命令:

``` 
	arm-linux-readrf -d app_name
```

找到缺少的库的名称, 然后去找这个库文件并复制到下面的位置:

``` 
	/usr/lib/
```

成功插入模块后,先运行write_mem向设备文件中写入数据,再运行read_mem读取数据:

``` 
	./write_mem
	./read_mem
```

如果运行成功,则会打印下面的信息: 

``` 
	dst is 2017
```

**三, 字符设备驱动模型**

1.设备描述结构**cdev**
在任何一种驱动模型中,设备都会用内核中的一种结构来描述。我们的字符设备在内核中使用struct cdev来描述:

``` c
#include <linux/dev.h>

struct cdev {
	struct kobject kobj;
	struct module *owner;
	//设备操作集
	const struct file_operations *ops;
	struct list_head list;
	//设备号
	dev_t dev; 
	//设备数目
	unsigned int count; 
};
```
cdev中比较重要的是:**设备操作集合ops, 设备号dev**, 下面就主要介绍他们的作用.

**设备操作集合ops**

``` c
#include <linux/fs.h>

/*
 * NOTE:
 * all file operations except setlease can be called without
 * the big kernel lock held in all filesystems.
 */
struct file_operations {
	struct module *owner;
	loff_t (*llseek) (struct file *, loff_t, int);
	ssize_t (*read) (struct file *, char __user *, size_t, loff_t *);
	ssize_t (*write) (struct file *, const char __user *, size_t, loff_t *);
	ssize_t (*aio_read) (struct kiocb *, const struct iovec *, unsigned long, loff_t);
	ssize_t (*aio_write) (struct kiocb *, const struct iovec *, unsigned long, loff_t);
	int (*readdir) (struct file *, void *, filldir_t);
	unsigned int (*poll) (struct file *, struct poll_table_struct *);
	long (*unlocked_ioctl) (struct file *, unsigned int, unsigned long);
	long (*compat_ioctl) (struct file *, unsigned int, unsigned long);
	int (*mmap) (struct file *, struct vm_area_struct *);
	int (*open) (struct inode *, struct file *);
	int (*flush) (struct file *, fl_owner_t id);
	int (*release) (struct inode *, struct file *);
	int (*fsync) (struct file *, int datasync);
	int (*aio_fsync) (struct kiocb *, int datasync);
	int (*fasync) (int, struct file *, int);
	int (*lock) (struct file *, int, struct file_lock *);
	ssize_t (*sendpage) (struct file *, struct page *, int, size_t, loff_t *, int);
	unsigned long (*get_unmapped_area)(struct file *, unsigned long, unsigned long, unsigned long, unsigned long);
	int (*check_flags)(int);
	int (*flock) (struct file *, int, struct file_lock *);
	ssize_t (*splice_write)(struct pipe_inode_info *, struct file *, loff_t *, size_t, unsigned int);
	ssize_t (*splice_read)(struct file *, loff_t *, struct pipe_inode_info *, size_t, unsigned int);
	int (*setlease)(struct file *, long, struct file_lock **);
	long (*fallocate)(struct file *file, int mode, loff_t offset, loff_t len);
};

```
文件操作集合中都是一系列的函数指针， 这些函数指针在调用对应的系统调用时会被底层代码调用，比如：read系统调用会最终调用一个驱动层的read，该驱动层的read就是编写时指定的file_operations中的read函数指针。我们在编写字符驱动的过程中，基本都要实现ops中的**open, release, read, write, llseek**函数，因为这几个函数是操作一个设备文件的基础。


**设备号**

设备号分为：**主设备号和次设备号**
主设备号用来关联一个设备文件和驱动程序， 次设备号用来确定该驱动程序管理的多个设备，比如一个串口驱动程序可以管理多个串口，而这多个串口是通过次设备号来确定。

设备号的相关操作：
设备号用dev_t dev来表示：

``` 
	dev_t dev = MKDEV(主设备号， 次设备号);
```

从dev_t中提取主设备号：

``` 
	主设备号 = MKJOR(dev_t dev);
```

从dev_t中提取次设备号：

``` 
	次设备号 = MINOR(dev_t dev);
```

如何申请设备号呢？设备号可以静态申请或者动态申请。

静态申请API：

``` c
<linux/fs.h>

/*
 * first : 是你要分配的起始设备编号,first 的次编号部分常常是 0, 但是没有要求是那个效果.
 * count : 是你请求的连续设备编号的总数
 * name : 是应当连接到这个编号范围的设备的名字
 */
int register_chrdev_region(dev_t first, unsigned int count, char *name);
```
静态申请不常用， 因为可能会分配到一个已经存在的设备号， 在实际开发中**经常使用动态申请**， API如下：

``` c
/* 需要的头文件 */
#include <linux/fs.h>

/* 全局devnum */
dev_t devnum;

/*
 * dev : 全局dev编号的指针
 * minor : 要分配的次设备号，一般设备为0
 * count : 要分配的设备数
 * name : 与设备号关联的设备文件名称
 */
int alloc_chrdev_region(dev_t *dev, unsigned int minor, unsigned int count, char *name)；
```

在驱动模块编程中，任何在模块初始化中申请的资源，**都要在模块退出时释放资源**， 使用下面的API来释放已经设备号：

``` c
/* 需要的头文件 */
#include <linux/fs.h>

/* 全局devnum */
dev_t devnum;

/*
 * devnum : 全局设备号
 * count : 设备个数
 */
void unregister_chrdev_region(dev_t devnum, unsigned int count);
```


**四，字符设备编程基本框架**

内核驱动编程框架已经比较成熟， 有些基本驱动都有可用的框架，在编程之前，先搭建好基本的框架，然后根据实际需求来增加功能。

**第一步：驱动初始化**

**分配cdev**，可以使用静态或者动态分配。

静态分配：

``` c
/* 需要的头文件 */
#include <linux/dev.h>

/* 静态分配的cdev */
struct cdev mdev;
```

动态分配：

``` c
/* 需要的头文件 */
#include <linux/dev.h>

/* pdev全局指针 */
struct cdev *pdev = NULL;

/* 动态申请 */
pdev = cdev_alloc();
```

**初始化cdev**：

``` c
/* 需要的头文件 */
#include <linux/dev.h>

/*
 * pcdev : cdev指针
 * fops : 文件操作集合指针
 */
cdev_init(struct cdev *pcdev, const struct file_operations *fops)
```

**注册cdev**：

``` c
/* 需要的头文件 */
#include <linux/dev.h>

/*
 * pcdev : cdev指针
 * devnum : 设备号
 * count : 设备数
 */
cdev_add(struct cdev *pcdev, dev_t devnum, unsigned count)
```

**之后需要根据芯片手册和具体的硬件平台完成其他的初始化工作**


**第二步 : 实现设备操作集合**

不同的驱动程序响应不同的系统调用，例如read驱动程序响应read系统调用，也就是说你在应用中使用read系统调用后会调用到最后的read驱动程序。

在介绍这个函数之前，我们需要先了解2个结构体：

**struct file**
``` c
/*
* 在Linux系统中,每一个打开的文件,在内核中都会关联一个struct file,它由内核在打开文件时创建, 在文件关闭后释放
* 下面是两个比较重要的成员。
*/
struct file {
	/* 文件读写指针 */
	loff_t f_pos;
	
	/* 该文件所对应的操作集合 */
	struct file_operations *f_op;
};
```

**struct inode**

``` c
/*
 * 每一个存在于文件系统里面的文件都会关联一个inode 结构,该结构主要用来记录文件物理上的信息。
 * 因此, 它和代表打开文件的file结构是不同的。
 * 一个文件没有被打开时不会关联file结构,file但是却会关联一个inode *结构。
 */
struct inode {
	/* 设备号 */
	dev_t i_rdev:
};
```

在了解了上面的2个结构之后，下面的几个设备操作基本上是字符设备都要实现的函数，你可以根据实际情况来取舍。

打开设备文件，响应open系统调用：

``` c
#include <linux/fs.h>

int (*open) (struct inode *inode, struct file * pfile);
```

关闭设备文件，响应close系统调用：

``` c
#include <linux/fs.h>

int (*release) (struct inode *inode, struct file *pfile);
```

用户空间从内核空间读取数据，响应read系统调用：

``` c
#include <linux/fs.h>

/*
 * pfile : 文件指针
 * buf : 存储读取到的内核数据
 * size : 读取的数目
 * pos : 读取的位置
 */
ssize_t (*read) (struct file *pfile, char __user *buf, size_t size, loff_t* pos);
```

用户空间向内核空间写入数据，响应write系统调用：

``` c
#include <linux/fs.h>

/*
 * pfile : 文件指针
 * buf : 存储读取到的内核数据
 * size : 读取的数目
 * pos : 读取的位置
 */
ssize_t (*write) (struct file *pfile, const char __user *buf, size_t size, loff_t *pos);
```

文件定位函数llseek，响应lseek系统调用

``` c
#include <linux/fs.h>

/*
 * pfile : 文件指针
 * offset : 偏移数量
 * where : 偏移位置
 */
loff_t (*llseek) (struct file *pfile, loff_t offset, int where);
```


**第三步 : 驱动注销**

删除cdev

``` c
/* 需要的头文件 */
#include <linux/dev.h>

/*
 * pdev : 要释放的cdev指针
 */
void cdev_del(struct cdev *pdev);
```

删除设备号

``` c
/* 需要的头文件 */
#include <linux/dev.h>

/*
 * devnum : 要释放的设备号变量
 * count : 设备数
 */
void unregister_chrdev_region(dev_t devnum, unsigned int count);
```

**四，总结**

驱动开发还是需要多多实践，你可以下载我的代码运行，然后尝试自己编写出来，在后面还会继续更新驱动的相关文章，敬请关注。










  [1]: https://cheng-zhi.github.io/code/LinuxDriverMode/Driver/memdev.c
  [2]: https://cheng-zhi.github.io/code/LinuxDriverMode/Driver/Makefile
  [3]: https://cheng-zhi.github.io/code/LinuxDriverMode/Driver/memdev.ko
  [4]: https://cheng-zhi.github.io/code/LinuxDriverMode/App/write_mem.c
  [5]: https://cheng-zhi.github.io/code/LinuxDriverMode/App/read_mem.c
