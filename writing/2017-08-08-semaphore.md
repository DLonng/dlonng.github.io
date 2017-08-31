---
title: Linux 高级编程 - 信号量 semaphore
date : 2016-06-06 12:00:00
---

# Linux 高级编程 - 信号量 semaphore
***
> 版权声明：本文为 {{ site.name }} 原创文章，可以随意转载，但必须在明确位置注明出处！ 

## 信号量 semaphore
信号量（semaphore）与之前介绍的管道，消息队列的等 IPC 机制不同，**信号量是一个计数器**，用来为多个进程或线程提供对共享数据的访问。

## 信号量的原理
常用的信号量是**二值信号量**，它控制单个共享资源，初始值为 1，操作如下：
1. 测试该信号量是否可用
2. 若信号量为 1，则当前进程使用共享资源，并将信号量减 1（加锁）
3. 若信号量为 0，则当前进程不可以使用共享资源并休眠，必须等待信号量为 1 进程才能继续执行（解锁）

要注意因为是使用信号量来保护共享资源，所以信号量本身的操作不能被打断，即必须是原子操作，因此由内核来实现信号量。

## 信号量的基本操作
Linux 内核提供了一套对信号量的操作，包括获取，设置，操作信号量，下面就来学习具体的 API。

### 1. 获取信号量
使用 `semget` 来创建或获取一个与 key 有关的信号量。

```c
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/sem.h>

/*
 * key：返回的 ID 与 key 有关系
 * nsems：信号量的值
 * semflg：创建标记
 * return：成功返回信号量 ID，失败返回 -1，并设置 erron
 */
int semget(key_t key, int nsems, int semflg);
```
关于参数的详细解释参考 `man semget`


### 2. 操作信号量
使用 `semop` 可以对一个信号量加 1 或者减 1：
```c
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/sem.h>

/*
 * semid：信号量 ID
 * sops：对信号量的操作
 * nsops：要操作的信号数量
 * return：成功返回 0，失败返回 -1，并设置 erron
 */
int semop(int semid, struct sembuf *sops, size_t nsops);
```

`sembuf` 表示了对信号量操作的属性：
```c
struct sembuf {
	unsigned short sem_num;  /* semaphore number */
	short          sem_op;   /* semaphore operation */
	short          sem_flg;  /* operation flags */
};
```

详细解释参考 `man semop`。


### 3. 设置信号量
使用 `semctl` 可以设置一个信号量的初始值：
```c
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/sem.h>

/*
 * semid：要设置的信号量 ID
 * semnum：要设置的信号量的个数
 * cmd：设置的属性
 */
int semctl(int semid, int semnum, int cmd, ...);
```

详细解释参考 `man semctl`。


## 例子：使用信号量进行进程间的同步





















