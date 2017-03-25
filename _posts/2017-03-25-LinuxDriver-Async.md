---
layout:     post
title:      "LinuxDriver-Async"
subtitle:   "S5PV210 LinuxDriver Async"
date:       2017-03-25 17:00:00
author:     "陈登龙"
header-img: "img/post-bg-unix-linux.jpg"
catalog: true
tags:
    - LinuxDriver
---


# LinuxDriver-Async

**一，Async简介**

Async简称异步通知，是驱动程序在有数据到来的时候通知应用程序的一种基于信号(signal)的机制, 因为是异步通知，所以应用程序可以做更多的事情，而不用像poll那样不断的轮询，但是对于多于一个驱动程序可以异步通知应用程序，那么应用程序仍然需要借助poll或者select来确定输入的来源。

**二，Async的使用**

**1.应用程序如何使用Async**

在app中，你只需要通知驱动程序，**本进程要接受你发送的信号**，然后**设置对应的信号处理程序**即可，只需要**这两步**，代码也很简单。

**app.c**
``` c
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <signal.h>


int g_fd = 0;
int g_press_cnt[4] = { 0 };


/*
 * Signal Callback
 * 在信号处理函数做你该做的任务
 */
void my_signal_fun(int signum)
{
/*
	int i = 0;
	read(g_fd, g_press_cnt, sizeof(g_press_cnt));
	for (i = 0; i < sizeof(g_press_cnt) / sizeof(g_press_cnt[0]); i++) {
		if (g_press_cnt[i])
			printf("Key%d has been pressed %d times!\n", i + 1, g_press_cnt[i]);
		}

	printf("\n");
}
*/



int main(void)
{
	int oflag = 0;
	/* 为进程注册一个SIGIO信号 */
	signal(SIGIO, my_signal_fun);

	/* 打开驱动的文件 */
	g_fd = open("/dev/Tiny210_KEY_ASYNC", O_RDWR);
	if (g_fd < 0) {
		printf("Can`t open /dev/Tiny210_KEY_ASYNC!\n");
		return -1;
	}

	/* Set PID and receive SIGIO and SIGUSR signal 设置该进程接受驱动发送的SIGIO信号 */
	fcntl(g_fd, F_SETOWN, getpid());
	
	/* Get file flags 得到驱动文件的状态标志 */
	oflag = fcntl(g_fd, F_GETFL);
	
	/* 
	 * Set file flags with FASYNC 
	 * 为驱动文件的状态标志加上FASYNC属性
	 * 文件打开时，默认清楚FASYNC标志，一旦设置该标志，驱动程序中的fasync函数就会自动被调用
	 */
	fcntl(g_fd, F_SETFL, oflag | FASYNC);

	/* 进程休眠，等待驱动的信号 */
	while (1) {
		sleep(1000);	
	}

	close(g_fd);

	return 0;
}
```


**2，驱动程序如何使用Async**

在驱动程序中，我们需要实现**fasync**函数，来设置要发送信号的进程，以及使用**kill_fasync来发送信号**。

你需要在驱动程序中定义一个结构：

``` c
/* async使用的数据结构，类似等待队列 */
static struct fasync_struct *key_async;
```


还需要添加fops中的fasync函数
``` c
/* 该函数在app中设置了FASYNC后会被自动调用 */
int key_fasync(int fd, struct file *filp, int on)
{
	printk(KERN_DEBUG "key fasync!\n");
	/* app的PID在filp->owner中 */
	return fasync_helper(fd, filp, on, &key_async);
}
```

在合适的地方，发送信号给app，这里使用中断的方式
``` c
/* 这个按键的中断程序调用kill_async来发送信号给app */
irqreturn_t key_handler(int irq, void *dev_id)
{
	volatile int *tmp_press_cnt = (volatile int *)dev_id;
	*tmp_press_cnt += 1;
	ev_press = 1;
	wake_up_interruptible(&key_wait_queue);
	
	/* 发送SIGIO信号给在key_fasync中绑定的用户进程 */
	kill_fasync(&key_async, SIGIO, POLLIN);

	return IRQ_RETVAL(IRQ_HANDLED);
}
```



解除文件的异步通知功能
``` c
/* 在文件关闭后要将文件从异步通知列表中删除 */
static int key_release(struct inode *inode, struct file *filp)
{
    //将文件从异步通知列表中删除
  key_fasync(-1,filp,0);

    return 0;
}
```


不管是轮询，poll还是异步通知，都有各种的适用范围，你应当根据实际需要来选择合适的方法。

