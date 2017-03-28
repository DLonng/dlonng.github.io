---
layout:     post
title:      "LinuxDriver-Asio"
subtitle:   "LinuxDriver Asio"
date:       2017-03-28 15:00:00
author:     "陈登龙"
header-img: "img/post-bg-unix-linux.jpg"
catalog: true
tags:
    - LinuxDriver
---

# LinuxDriver-Asio

**一，Asio简介**

Asio是异步IO的简称，在内核驱动编程过程中，如果我们想在获取**获取不到某种资源**能够**立刻返回**而不是阻塞时，我们就应该使用Asio。

**二，同步IO和异步IO的区别**

同步IO在访问资源的时候，如果访问的资源不能立刻获取，该进程就会阻塞直到获取到资源，即**同步IO = 阻塞IO**。

异步IO在访问资源的时候，如果访问的资源不能立刻获取，该进程就会立刻返回，即**异步IO = 非阻塞IO**。

**三，如何在app和driver中使用异步IO**

我们需要对应用程序打开文件的标记做点手脚，就像下面这样：

``` c
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>


int main(void)
{
	int fd = 0;

	/* Using NONBLOCK flag to open file. */
	fd = open("/dev/Tiny210_LED", O_RDWR | O_NONBLOCK);


	if (fd < 0) {
		printf("Failed to open!\n");
		return -1;
	}
	
	while(1);

	return 0;
}
```

我们在open的flag上加了一个**NONBLOCK**标记，该标记表示在打开这个文件的时候，该进程使用异步发方式打开，但是这个设备文件需要支持异步打开的功能，下面我们来看下驱动程序的框架：

``` c
int led_open(struct inode *pnode, struct file *filp)
{
	/*
	 * 我们在打开文件的时候判断文件是否有NONBLOCK的标记
	 * 如果有标记，获取不到信号量来返回-EBUSY到用户空间，即为非阻塞IO访问
	 * 如果没有标记，获取不到信号量，该进程就会睡眠，即为阻塞IO访问
	 */
	if (filp->f_flags & O_NONBLOCK) {
		/* Try to get sem, don`t get sem then return -EBUSYY. */
		if (down_trylock(&sem))
			return -EBUSY;
	} else {
		/* if BLOCK IO, then get sem directly, don`t get then sleep. */
		down(&sem);
	}

	return 0;
}


int led_release(struct inode *pnode, struct file *filp)
{
	/* 关闭文件，释放信号量 */
	up(&sem);
	return 0;
}

```

我们还可以使用这种方法来实现在read中的异步读取，只要将open中的if else框架放到read中即可，原理都是相同的。


**四，总结**

阻塞和非阻塞的概念在应用层编程使用的频率也很高，我们经常听过**SendMessage(同步发送消息)和PosyMessage(异步发送消息)**这两个发送消息的函数，他们就是阻塞和非阻塞的很好体现，在内核中我们再次对他们有了一次更加深刻的了解，进而可以帮助我们理解这种概念的原理。















