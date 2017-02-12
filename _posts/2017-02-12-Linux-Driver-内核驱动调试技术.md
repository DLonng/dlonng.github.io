---
layout:     post
title:      "LinuxDriver-内核驱动调试技术"
subtitle:   "LinuxDriver"
date:       2017-02-12 17:00:00
author:     "陈登龙"
header-img: "img/post-bg-unix-linux.jpg"
catalog: true
tags:
    - LinuxDriver
---

**LinuxDriver-内核驱动调试技术**

**一，调试简介**

写过程序的人都知道，**写程序的时间远远小于调试程序的时间**。如果你调试过应用程序，你可能非常熟练，但是内核驱动的调试方法却跟调试应用有很大不同，下面就介绍几种在内核中调试**常用**的技术。



**二，第一种调试方法：printk**

printk可以在控制台输出日志信息，它是**调试内核驱动最有效的手段**，在调试过程中**95%用到**的都是printk。

在使用printk过程中，有些基本功能必须了解。

**1.printk输出的日志级别**

日志级别用**宏定义**表示，日志级别宏展开为一个字符串，在编译时由预处理器将它和消息文本拼接成一个字符串，因此printk函数中**日志级别宏和格式字符串间不能有逗号**。像下面这样的格式：

``` c
	printk(KERN_DEBUG "This is a debug msg!\n");
```

前面的KERN_DEBUG就是日志的打印级别，在内核中有如下几种级别：

``` c
	/* 日志级别宏在linux/kernel.h中定义 */
	#define   KERN_EMERG "<0>"
	#define   KERN_ALERT "<1>"
	#define   KERN_CRIT "<2>"
	#define   KERN_ERR "<3>"
	#define   KERN_WARNING "<4>"
	#define   KERN_NOTICE "<5>"
	#define   KERN_INFO "<6>"
	#define   KERN_DEBUG "<7>"
```

日志级别有如下**特点**：

1.日志级别范围是：**0 - 7**

2.没有指定日志级别的printk语句**默认级别是4**, 表示当**级别高于4**(0 - 3, **编号越小，级别越高**)的日志信息，将会在终端上打印。


``` c
	/* 定义在kernel/printk.c中 */
	#define DEFAULT_MESSAGE_LOGLEVEL 4
```

3.通过如下命令可以使所有打印信息输出到终端：

``` 
	echo 8 > /proc/sys/kernel/printk
```


**2.dmesg**

dmesg通过查看**/proc/kmsg**文件可以输出**所有由printk打印的内核信息**。可以在Linux中运行如下的命令查看所有的printk日志：

``` 
	dmesg
```


**3.如何在驱动中使用printk？**

在驱动开发代码中，不要直接使用printk进行调试，应该设置一个**printk宏开关**，用来控制和打开printk功能。可以参考下面的printk宏开关：

``` c
/* 1表示打开调试功能 ，0表示关闭调试功能*/
#define DEBUG 1

#ifdef DEBUG
/* args表示可以打印带参数的调试信息 */
#define TS_DEBUG(fmt,args...) printk(fmt, ##args)
#else
#define TS_DEBUG(fmt,args...)
#endif
```

但是printk有一个**缺点**：打印需要时间，不适合调试实时性很高的程序。


**三，第二种调试方法：oops**

oops：**内核级的段错误**，内核发生崩溃(panic)时，调用**panic函数**所打印的信息。


oops分析方法
先看一段内核oops：

``` 
[ 25.446300] Hi
[ 25.446324] Function A
[ 25.446345] Function B
[ 25.446366] Function C
[ 25.446386] Function D
//错误原因
[ 25.446416] Unable to handle kernel NULL pointer dereference at virtual address 00000000 
//错误页表位置
[ 25.446483] pgd = d8ba8000 
[ 25.446508] [00000000] *pgd=38b60831, *pte=00000000, *ppte=00000000
//内核错误代码817，#1表示该oops只出现了一次，PREEMPT表示当前内核允许抢占
[ 25.452034] Internal error: Oops: 817 [#1] PREEMPT
[ 25.456796] Modules linked in: oops_test(+)
//单核CPU，编号为0，内存没有被污染，括号里是内核版本
[ 25.460959] CPU: 0 Not tainted (3.0.8-FriendlyARM #3)


//发生错误的PC指针，0x20是D函数的长度，0x18表示oops的位置偏移位置
[ 25.466337] PC is at D+0x18/0x20 [oops_test]
//发生错误的上一个函数位置
[ 25.470578] LR is at D+0x10/0x20 [oops_test]


//寄存器地址
[ 25.474823] pc : [<bf000018>] lr : [<bf000010>] psr: 60000013
[ 25.474828] sp : d8b75eb0 ip : 00000007 fp : 0000002e
[ 25.486260] r10: 00000001 r9 : 000000e5 r8 : bf002000
[ 25.491459] r7 : 00000000 r6 : d8b74000 r5 : c07ec280 r4 : bf000154
[ 25.497959] r3 : 00000000 r2 : 00000002 r1 : 60000093 r0 : 0000001d
[ 25.504459] Flags: nZCv IRQs on FIQs on Mode SVC_32 ISA ARM Segment user
[ 25.511564] Control: 10c5387d Table: 38ba8019 DAC: 00000015
[ 25.517283] 
//SP指针附近的内容
[ 25.517285] SP: 0xd8b75e30:
[ 25.521527] 5e30 342e3532 38333634 00205d36 30333634 00205d30 00000000 c080fd58 c080fd5c
[ 25.529673] 5e50 ffffffff d8b75e9c d8b74000 00000000 bf002000 c004212c 0000001d 60000093
[ 25.537819] 5e70 00000002 00000000 bf000154 c07ec280 d8b74000 00000000 bf002000 000000e5
[ 25.545965] 5e90 00000001 0000002e 00000007 d8b75eb0 bf000010 bf000018 60000013 ffffffff
[ 25.554110] 5eb0 00000000 bf002010 00000000 c0037494 00000001 c00c9790 00000007 d8ae0d80
[ 25.562256] 5ed0 00000000 bf000154 bf00010c bf000154 bf00010c 00000001 d8b06d80 0000001c
[ 25.570402] 5ef0 00000001 c008f71c bf000118 d8b75fb0 00000000 c008e7c0 bf000230 0000003f
[ 25.578547] 5f10 d8b74000 e085024c e085024c 001abb66 d863aea0 e0850000 00006a59 e0854ac4


//内核崩溃时的堆栈 [<地址>](函数名)，这个信息对于调试很有帮助
[ 25.825451] [<bf000018>] (D+0x18/0x20 [oops_test]) from [<bf002010>] (myoops_init+0x10/0x1c [oops_test])
[ 25.834894] [<bf002010>] (myoops_init+0x10/0x1c [oops_test]) from [<c0037494>] (do_one_initcall+0x30/0x174)
[ 25.844601] [<c0037494>] (do_one_initcall+0x30/0x174) from [<c008f71c>] (sys_init_module+0xf0/0x195c)
[ 25.853785] [<c008f71c>] (sys_init_module+0xf0/0x195c) from [<c0042640>] (ret_fast_syscall+0x0/0x30)
[ 25.862875] Code: e34b0f00 eb55243c e3a02002 e3a03000 (e5832000) 
[ 25.870708] ---[ end trace 96d22d7f0bc4f118 ]---
Segmentation fault
```

在分析oops时，有下面几个主要步骤：

**1.错误原因**

错误原因一般是在第一行，例如上面的oops的错误原因是：

``` 
Unable to handle kernel NULL pointer dereference at virtual address 00000000 
```

**2.函数调用栈**

内核崩溃时的堆栈 [<地址>](函数名)，这个信息对于调试很有帮助，最后被调用的函数栈在最上面，例子的函数调用栈：

``` 
[ 25.825451] [<bf000018>] (D+0x18/0x20 [oops_test]) from [<bf002010>] (myoops_init+0x10/0x1c [oops_test])
[ 25.834894] [<bf002010>] (myoops_init+0x10/0x1c [oops_test]) from [<c0037494>] (do_one_initcall+0x30/0x174)
[ 25.844601] [<c0037494>] (do_one_initcall+0x30/0x174) from [<c008f71c>] (sys_init_module+0xf0/0x195c)
[ 25.853785] [<c008f71c>] (sys_init_module+0xf0/0x195c) from [<c0042640>] (ret_fast_syscall+0x0/0x30)
```

**3.寄存器**

分析常见的**PC指针，SP指针**等，如上面的oops的寄存器指针信息：

``` 
//发生错误的PC指针，0x20是D函数的长度，0x18表示oops的位置偏移位置
[ 25.466337] PC is at D+0x18/0x20 [oops_test]
//发生错误的上一个函数位置
[ 25.470578] LR is at D+0x10/0x20 [oops_test]
//寄存器地址
[ 25.474823] pc : [<bf000018>] lr : [<bf000010>] psr: 60000013
[ 25.474828] sp : d8b75eb0 ip : 00000007 fp : 0000002e
[ 25.486260] r10: 00000001 r9 : 000000e5 r8 : bf002000
[ 25.491459] r7 : 00000000 r6 : d8b74000 r5 : c07ec280 r4 : bf000154
[ 25.497959] r3 : 00000000 r2 : 00000002 r1 : 60000093 r0 : 0000001d
[ 25.504459] Flags: nZCv IRQs on FIQs on Mode SVC_32 ISA ARM Segment user
[ 25.511564] Control: 10c5387d Table: 38ba8019 DAC: 00000015
[ 25.517283] 
```

查看到对应的PC指针地址后，**反汇编驱动模块文件并输出到文件查看对应地址的错误代码**：

``` 
	arm-linux-objdump -D -S *.ko > log
```

之后就可以分析这个log文件了。


**四，第三种调试方法：kprobe**

kprobe用于调试正在**运行中的内核**代码, 在要调试的API前面或者后面加上调试信息，可以动态调试内核(通过insmod)。

使用方法：在PC机测试，**kprobe在ARM上的实现不完整**。编写一个调试模块插入到内核代码中，实现动态调试内核。

**kprobe.c**

``` c
#include <linux/module.h>
#include <linux/kprobes.h>
#include <linux/kallsyms.h>
#include <linux/sched.h>


struct kprobe kp;

/*
 * 加载到要调试到函数的头部的信息
 */
int handler_pre(struct kprobe *p, struct pt_regs *regs) {
	printk(KERN_INFO "pt_regs: %p, pid: %d, jiffies: %ld\n", regs, current->tgid, jiffies);
	return 0;
}

/*
 * 调试模块入口
 * 通过调试模块动态调试do_execve函数
 */
static int __init init_kprobe_sample(void) {
	printk(KERN_INFO "Init kprobe.\n");
	kp.symbol_name = "do_execve";
	kp.pre_handler = handler_pre;

	register_kprobe(&kp);

	return 0;
}


/*
 * 调试模块退出
 */
static void __exit cleanup_kprobe_sample(void) {
	printk(KERN_INFO "Exit kprobe.\n");
	unregister_kprobe(&kp);
}


MODULE_LICENSE("GPL"); 
module_init(init_kprobe_sample);
module_exit(cleanup_kprobe_sample);
```

**Makefile:**

``` 
obj-m := kprobe.o
CURRENT_PATH := $(shell pwd)
LINUX_KERNEL := $(shell uname -r)
LINUX_KERNEL_PATH := /usr/src/linux-headers-$(LINUX_KERNEL)
all:
  make -C $(LINUX_KERNEL_PATH) M=$(CURRENT_PATH) modules
clean:
  make -C $(LINUX_KERNEL_PATH) M=$(CURRENT_PATH) clean

```

在make之后，插入到正在运行的内核中：

``` 
	insmod kprobe.ko
```

通过ls命令可以间接调用到**do_execve**函数，所以我们**执行一次ls**命令，然后用**dmesg**命令查看printk的日志就可以查看到**handler_pre中打印的信息**。


**五，第四种调试方法：kcore**

kcore调试当前正在**运行内核**的**内存映像**文件：

``` 
	/proc/kcore
```

**gdb可以引用kcore进行调试**。


**六，总结**

在实际驱动开发过程中，**用的最多的是printk**，其他的调试技术也必须了解。
