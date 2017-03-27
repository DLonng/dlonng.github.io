---
layout:     post
title:      "LinuxKernel-Semaphore"
subtitle:   "LinuxKernel Semaphore"
date:       2017-03-27 12:00:00
author:     "陈登龙"
header-img: "img/post-bg-linuxkernel.jpg"
catalog: true
tags:
    - LinuxKernel
---

# LinuxKernel-Semaphore

**一，Semaphore简介**

信号量(**semaphore**)是OS中最典型的用于同步和互斥的手段，对OS中的PV概念相对应。

**二，内核中的信号量API**

**1.定义sem**

``` c
/* 动态分配，然后初始化 */
struct semaphore sem;
void sema_init(struct semaphore *sem, int val);

//or 

/* 静态定义 */
static DEFINE_SEMAPHORE(sem);
```

**2.操作sem**

``` c
/* 获取sem， 不能获取的时候会进入睡眠状态，并且不能被信号打断 */
void down(struct semaphore *sem);

/* 获取sem， 不能获取的时候会进入睡眠状态，可以被信号打断 */
void down_interruptible(struct semaphore *sem);

/* 
 *尝试获取sem，如果能够立刻获取，就返回0, 否则立刻返回非0值，
 * 在使用的时候不能获取一般返回-ERESTARTSYS 
  */
void down_trylock(struct semaphore *sem);

/* 释放sem，并唤醒睡眠的进程 */
void up(struct semaphore *sem);

```

**三，信号量的使用**

sem在使用的时候要注意：**获取不到信号量的进程会进入休眠状态而不是在想spin_loc那样原地等待。**

例如：我们在open中获取sem，在release中释放sem来**保证只有一个进程可以操作该驱动设备文件**。

``` c

/* 静态定义一个信号量 */
static DEFINE_SEMAPHORE(sem);

int led_open(struct inode *pnode, struct file *filp)
{
	/*
	* 第一个进程可以成功获取信号量，而第二个进程不能获取到信号量则就会进入睡眠状态
	*/
	down(&sem);
	return 0;
}


int led_release(struct inode *pnode, struct file *filp)
{
	/*
	 * 在第一个进程释放了信号量后，第二个进程就会被唤醒。
	 */
	up(&sem);
	return 0;
}
```


**四，总结**

**信号量的适用范围**：sem可以保护**临界区**的代码，只有的到sem的进程才可以进入临界区，另外如果你遇到了具体的**生产者和消费者**的问题，使用sem是最合适的方法。

