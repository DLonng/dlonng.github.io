---
title: LinuxKernel - Semaphore
date:  2016-12-12 12:00:00
---

# LinuxKernel - Semaphore
***
> 版权声明：本文为 {{ site.name }} 原创文章，可以随意转载，但必须在明确位置注明出处！ 

## Semaphore 简介

信号量（**semaphore**）是 OS 中典型的用于同步和互斥的手段，这次记录下 `sem` 在 `LinuxKernel` 中的体现。

## 内核信号量 API

#### 动态分配
``` c
/* 动态分配，然后初始化 */
struct semaphore sem;
void sema_init(struct semaphore *sem, int val);
```

#### 静态分配
```c
/* 静态定义 */
static DEFINE_SEMAPHORE(sem);
```


#### 不可被打断的获取 sem

``` c
/* 获取 sem，不能获取的时候会进入睡眠状态，并且不能被信号打断 */
void down(struct semaphore *sem);
```
#### 可被打断的获取 sem

```c
/* 获取 sem，不能获取的时候会进入睡眠状态，可以被信号打断 */
void down_interruptible(struct semaphore *sem);
```

#### 异步获取 sem

```c
/* 
 * 尝试获取 sem，如果能够立刻获取，就返回 0, 否则立刻返回非 0 值
 * 在使用的时候不能获取一般返回 -ERESTARTSYS 
 */
void down_trylock(struct semaphore *sem);
```

#### 释放 sem

```c
/* 释放 sem，并唤醒睡眠的进程 */
void up(struct semaphore *sem);
```

## 信号量的使用

`sem` 在使用的时候要注意：获取不到信号量的进程会进入**休眠状态**而不是像 `spin_lock` 那样原地等待。例如：我们在 `open` 中获取 `sem`，在 `release` 中释放 `sem` 来**保证只有一个进程可以操作驱动设备文件**。

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


## 总结

**信号量的适用范围**：`sem` 可以保护**临界区**的代码，只有得到 `sem` 的进程才可以进入临界区，如果遇到了类似的**生产者和消费者**的问题，使用 `sem` 是比较合适的方法。

> 信号量要成对使用
