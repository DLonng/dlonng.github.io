---
layout:     post
title:      "LinuxDriver-原子变量"
subtitle:   "LinuxDriver Atomic"
date:       2017-03-26 21:00:00
author:     "陈登龙"
header-img: "img/post-bg-unix-linux.jpg"
catalog: true
tags:
    - LinuxDriver
---


# LinuxDriver-原子变量

**一，原子变量简介**

原子变量听起来很高大上，其实就是一个变量而已，只是CPU在访问这个变量的时候，增加了一些对这个变量读写的限制，你可以理解为，**CPU在访问这个变量的时候限制了其他的的实体对该变量的并发访问**，在ARM中使用**LDREX**和**STREX**指令来限制，简单来说就是将要保护的代码放在LDREX和STREX之间，这两个指令通常配对使用。

**二，内核中的原子变量**

在内核中有一套已经实现好的原子变量的操作API，我们在理解了原子变量基础上需要学会使用他们，如果有兴趣可以深入了解源码。

**1.初始化Atomic**

``` c
/* 设置原子变量的值为i */
void atomic_set(atomic_t *v, int i);

/* 静态初始化一个值为i的原子变量 */
atomic_t = ATOMIC_INIT(i);
```

**2.获取Atomic**

``` c
/* 返回原子变量的值 */
atomic_read(atomic_t *v);
```

**3.操作原子变量**

``` c
/* 原子变量增加i */
void atomic_add(int i, atomic_t *v);

/* 原子变量减少i */
void atomic_sub(int i, atomic_t *v);

/* 原子变量自增i */
void atomic_inc(atomic_t *v);

/* 原子变量字减i */
void atomic_inc(atomic_t *v);
```

**4.操作并测试**

``` c
/* 下面3个函数，如果测试原子变量为0就返回true，否则返回false */
int atomic_inc_and_test(atomic_t *v);
int atomic_dec_and_test(atomic_t *v);
int atomic_sub_and_test(int i, atomic_t *v);
```

**5.操作并返回**

``` c
/* 下面的操作对原子变量进行自增加或者自减少操作，并返回新的值 */
int atomic_add_return(int i, atomic_t *v);
int atomic_sub_return(int i, atomic_t *v);
int atomic_inc_return(atomic_t *v);
int atomic_dec_return(atomic_t *v);
```


内核在对**每一位**的操作上也提供了一系列的原子操作函数


``` c
/* 1.设置一个位为1 */
void set_bit(nr, void *addr);

/* 2.清楚位 */
void clear_bit(nr, void *addr);

/* 3.改变位 */
void change_bit(nr, void *addr);

/* 4.测试位 */
test_bit(nr, void *addr);

/* 5.测试并操作位 */
int test_and_set_bit(nr, void *addr);
int test_and_clear_bit(nr, void *addr);
int test_and_clange_bit(nr, void *addr);
```

**三，在驱动中如何使用原子变量**

我们以一个驱动为例：我们使用原子变量**使得一个设备文件只能被一个进程**打开。



``` c
/* 1.定义原子变量, 初始化设置为1 */
static atomic_t atomic_open = ATOMIC_INIT(1);

static int my_open(struct inode *inode, struct file *filp)
{
	/*
	 * 在第一次打开文件的时候将原子变量减去1, 当有第二个进程
	 * 打开本文件的时候，下面的if语句将会返回false，即不能再次打开文件。 
	 */
	if (!atomic_dev_and_test(atomic_open))
	{
		atomic_inc(&atomic_open);
		return -EBUSY;
	}
	return 0;
}

static int my_release(struct inode *inode, struct file *filp)
{
	/* 
	 * 只有当该文件被成功打开后，才有被释放的机会
	 * 因为在open的时候将原子变量减去1,所以为了其他的进程能够
	 * 重新打开这个文件，在文件关闭的时候需要将原子变量恢复到
	 * 初始值，即为1.
	 */
	atomic_inc(&atomic_open);
	return 0;
}
```


**四，总结**

关于内核驱动中防止并发访问的方法还有几种，例如：信号量，自旋锁，互斥量等，在后面的文章中会进行简单的介绍。
























