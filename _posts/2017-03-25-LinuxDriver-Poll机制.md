---
layout:     post
title:      "LinuxDriver-Poll机制"
subtitle:   "S5PV210 LinuxDriver Poll"
date:       2017-03-25 16:00:00
author:     "陈登龙"
header-img: "img/post-bg-unix-linux.jpg"
catalog: true
tags:
    - LinuxDriver
---


# LinuxDriver-Poll机制

**一，Poll简介**

Poll(轮询)，从名称上可以看出，是不断查询的意思，但是与while(1) + read不同，这种查询不会占用非常多的CPU资源。

**poll机制：** 在调用进程设置的超时时间内如果有数据到来，驱动程序就唤醒该进程，否则该进程在超时过后就会被自动唤醒，然后重复查询。

**二，poll的实际使用**

我们在实际使用过程中，在app中使用poll系统调用来调用驱动程序中的poll函数，进而实现读取驱动程序数据的功能。
poll的定义：

``` c

#include <poll.h>

/*
 *  fds: 参数fds指向所有要监听的文件描述符对应结构体struct pollfd的数组
 *  参数nfds 表示struct pollfd结构的数目（亦即是要监听的文件描述符的个数）
 *  timeout是超时时间，单位为milliseconds（指定一个负数表示一直阻塞）
 */
int poll(struct pollfd *fds, 　nfds_t nfds,　int timeout);


```




这是一段app的程序实例：

``` c
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/types.h>
#include <sys/stat.h>


int main(void)
{
	int i = 0;
	int ret = 0;
	int fd = 0;
	int press_cnt[4] = { 0 };

	struct pollfd fds[1];

	fd = open("/dev/Tiny210_KEY_POLL", O_RDWR);
	
	if (fd < 0) {
		printf("Can`t open /dev/Tiny210_KEY_POLL!\n");
		return -1;
	}
	
	fds[0].fd = fd;
	fds[0].events = POLLIN;



	while (1) {
		
		/*
		 * 指定5S的超时时间，在5S内，如果驱动中有数据就唤醒该进程，
		 * 如果超时，就自动唤醒该进程，超时会返回0，会打印Time out
		 */
		ret = poll(fds, 1, 5000);

		if (!ret) {
			/* 超时：ret = 0 */
			printf("Time out!\n");
		} else {
		    /* 在超时时间内放会非0，表示可以从驱动读取数据 */
			read(fd, press_cnt, sizeof(press_cnt));
			for (i = 0; i < sizeof(press_cnt) / sizeof(press_cnt[0]); i++) {
				if (press_cnt[i])
					printf("Key%d has been pressed %d times!\n", i + 1, press_cnt[i]);
			}

			printf("\n");

		}
	
	}
	close(fd);

	return 0;
}
```


这是poll对应的驱动程序：

``` c
unsigned int key_poll(struct file *filp, struct poll_table_struct *wait)
{
	unsigned int mask = 0;
	
	/* 
	 * poll进行查询，在第一次查询失败后，该进程才会被加入等待队列中，并进入休眠状态 
	 * 注意：该函数不会立刻让调用者进程进入休眠状态
	 */
	poll_wait(filp, &key_wait_queue, wait);

	if (ev_press)
		mask |= POLL_IN | POLLRDNORM;//设置标记，通知app可以读取驱动的数据
	
	return mask;
}
```

例如在这个键盘中断处理程序中唤醒了在等待队列中睡眠的进程：

``` c
/*
 * Key1 IRQ handler
 */
irqreturn_t key1_handler(int irq, void *dev_id)
{
	volatile int *tmp_press_cnt = (volatile int *)dev_id;
	*tmp_press_cnt += 1;
	
	/* 将在等待队列中睡眠的进程唤醒 */
	ev_press = 1;
	wake_up_interruptible(&key_wait_queue);

	return IRQ_RETVAL(IRQ_HANDLED);
}
```


**三，poll机制分析**

1.从系统调用poll开始：poll -> sys_poll -> do_sys_poll -> poll_initwait，poll_initwait函数注册 __pollwait 回调函数. 

2.之后执行我们自己在驱动程序编写的poll函数(file->f_op->poll), 它会调用poll_wait把调用者进程挂入**进程等待队列**，这个队列是在驱动程序中自己定义的。

3.**唤醒进程**：驱动程序在poll设置的**timeout时间内唤醒**了调用者进程，例如可以通过中断的方式唤醒在等待队列中休眠的进程; 如果驱动**没有在timeout时间内唤醒**进程，那么在timeout时间过后，调用者进程会被**自动唤醒**。