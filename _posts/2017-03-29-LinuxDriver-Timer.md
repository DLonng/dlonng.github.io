---
layout:     post
title:      "LinuxDriver-Timer"
subtitle:   "LinuxDriver Timer"
date:       2017-03-29 21:00:00
author:     "陈登龙"
header-img: "img/post-bg-unix-linux.jpg"
catalog: true
tags:
    - LinuxDriver
---



# LinuxDriver-Timer

**一，内核定时器简介**

定时器是我们在应用层编程经常使用的技术，在内核驱动中我们也会经常使用定时器，例如使用定时器来进行按键的消抖，这次我们就简单了解下Linux内核中的定时器。


**二，内核定时器**

内核使用下面的结构来描述一个定时器：

``` c
struct timer_list {
    struct list_head entry; 
	/* 定时时间 */
    unsigned long expires;
	/* 定时器回调函数 */
    void (*function)(unsigned long);
	/* 传递给定时器回调函数的参数 */
    unsigned long data; 
    struct tvec_base *base;
    /* ... */
};
```
我们只需要关心加上注释的3个参数即可。


**三，使用内核定时器来消除按键抖动框架**

我们知道普通的机械按键在按下的过程中会存在抖动现象，会导致一个按键的中断被触发多次，我们可以通过定时器延时来解决这个问题。
下面是一个按键消抖的框架，其中在**模块init函数中注册定时器到内核**，按键的**中断处理来更新并启动定时器**，**定时器回调函数来处理实际的按键逻辑**，在**模块exit函数中删除定时器**，同时需要一个全局**指针来传递**中断处理函数和定时器回调函数的**参数**。


``` c
/* 定时器所在的头文件 */
#include <linux/timer.h>

/* 动态定义一个定时器结构 */
struct timer_list my_timer;
/* 声明定时器的回调函数 */
void timer_fun(unsigned long data);

/* 参数指针 */
static volatile int *timer_data = NULL;

/*
 * 按键中断处理，将定时器延时10ms后执行回调函数
 */
irqreturn_t key1_handler(int irq, void *dev_id)
{
	printk(KERN_DEBUG "key1_handler\n");
	/* 接受中断的参数 */
	timer_data = (volatile int *)dev_id;
	
	/* Delay 10ms, jiffies一秒内增加的值为HZ, 所有HZ / 100就相当与1000 / 100 = 10ms，即延时时间是10ms，你也可以自行设定 */
	mod_timer(&my_timer, jiffies + HZ / 100);

	return IRQ_RETVAL(IRQ_HANDLED);
}

/*
 * Timer function
 */
void timer_fun(unsigned long data)
{
	/* 在模块init函数中第一次add_timer会自动调用一次定时器的回调函数，这里进行相应的处理 */
	if (NULL == timer_data)
	{
		printk(KERN_DEBUG "******First time_fun******\n");
		return;
	}	
	
	printk(KERN_DEBUG "------timer_fun------\n");
	
	/* 下面的逻辑即为中断处理中的逻辑 */
	volatile int *tmp_press_cnt = timer_data;
	*tmp_press_cnt += 1;

	ev_press = 1;
	wake_up_interruptible(&key_wait_queue);
	
	kill_fasync(&key_async, SIGIO, POLLIN);

	return IRQ_RETVAL(IRQ_HANDLED);
}


/*
 * Module init
 */
static int __init mykey_int_init(void)
{
	key_create_device();

	/* 初始化定时器 */
	init_timer(&my_timer);
	/* 设置回调函数 */
	my_timer.function = timer_fun;
	
	/* expires 没有配置时默认值是0， 因为我们是进行按键的消抖动，所以这里不需要赋值 */
	
	/* 向内核注册定时器 */
	add_timer(&my_timer);

	printk(KERN_DEBUG DEV_NAME "\tmykey_int_init ok!\n");
	return 0;
}



/*
 * Module exit
 */
static void __exit mykey_int_exit(void)
{
	key_destroy_device();
	/* 删除定时器 */
	del_timer(&my_timer);
	printk(KERN_DEBUG DEV_NAME "\tmykey_int_exit ok!\n");
}

```


**四，总结**

当你将这个框架应用到实际的代码中时，当你按下按键时如果按键**发生了抖动**，那么在**中断处理程序中的打印信息会打印多次**，而**定时器处理中的打印信息只会打印一次**，说明我们的按键消抖成功，即实际的**中断处理的逻辑代码只会被调用一次**。





















