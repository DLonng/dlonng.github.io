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

oops：**内核级的段错误**，内核发生崩溃(panic)时，调用**panic函数**所打印的信息。下面是panic.c的代码：

``` c
/**
 *panic - halt the system
 *@fmt: The text string to print
 *
 *Display a message, then perform cleanups.
 *
 *This function never returns.
 */
NORET_TYPE void panic(const char * fmt, ...)
{
  static char buf[1024];
  va_list args;
  long i, i_next = 0;
  int state = 0;


  /*
   * It's possible to come here directly from a panic-assertion and
   * not have preempt disabled. Some functions called from here want
   * preempt to be disabled. No point enabling it later though...
   */
  preempt_disable();


  console_verbose();
  bust_spinlocks(1);
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  printk(KERN_EMERG "Kernel panic - not syncing: %s\n",buf);
#ifdef CONFIG_DEBUG_BUGVERBOSE
  dump_stack();
#endif


  /*
   * If we have crashed and we have a crash kernel loaded let it handle
   * everything else.
   * Do we want to call this before we try to display a message?
   */
  crash_kexec(NULL);


  kmsg_dump(KMSG_DUMP_PANIC);


  /*
   * Note smp_send_stop is the usual smp shutdown function, which
   * unfortunately means it may not be hardened to work in a panic
   * situation.
   */
  smp_send_stop();


  atomic_notifier_call_chain(&panic_notifier_list, 0, buf);


  bust_spinlocks(0);


  if (!panic_blink)
     panic_blink = no_blink;


  if (panic_timeout > 0) {
     /*
      * Delay timeout seconds before rebooting the machine.
      * We can't use the "normal" timers since we just panicked.
      */
     printk(KERN_EMERG "Rebooting in %d seconds..", panic_timeout);


     for (i = 0; i < panic_timeout * 1000; i += PANIC_TIMER_STEP) {
       touch_nmi_watchdog();
       if (i >= i_next) {
          i += panic_blink(state ^= 1);
          i_next = i + 3600 / PANIC_BLINK_SPD;
       }
       mdelay(PANIC_TIMER_STEP);
     }
     /*
      * This will not be a clean reboot, with everything
      * shutting down.  But if there is a chance of
      * rebooting the system it will be rebooted.
      */
     emergency_restart();
  }
#ifdef __sparc__
  {
     extern int stop_a_enabled;
     /* Make sure the user can actually press Stop-A (L1-A) */
     stop_a_enabled = 1;
     printk(KERN_EMERG "Press Stop-A (L1-A) to return to the boot prom\n");
  }
#endif
#if defined(CONFIG_S390)
  {
     unsigned long caller;


     caller = (unsigned long)__builtin_return_address(0);
     disabled_wait(caller);
  }
#endif
  local_irq_enable();
  for (i = 0; ; i += PANIC_TIMER_STEP) {
     touch_softlockup_watchdog();
     if (i >= i_next) {
       i += panic_blink(state ^= 1);
       i_next = i + 3600 / PANIC_BLINK_SPD;
     }
     mdelay(PANIC_TIMER_STEP);
  }
}
 
```


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
[ 25.586695] 
[ 25.586697] R5: 0xc07ec200:
[ 25.590939] c200 00000000 00000000 00000000 00000000 00000000 00000000 00000000 00000000
[ 25.599085] c220 00000000 00000000 00000000 00000000 00000000 00000000 00000000 00000000
[ 25.607231] c240 00000004 00000014 00000003 00554e47 0597c863 2d620ffc 7c8e2cf6 854c7216
[ 25.615376] c260 436fabe5 00000000 00000000 00000000 00000000 00000000 00000000 00000000
[ 25.623522] c280 00000000 c0c351a5 00000000 00000000 00000000 c0c35080 c0c35180 00000000
[ 25.631668] c2a0 00000000 00000000 00000000 00000000 00000000 00000000 00000000 00000000
[ 25.639814] c2c0 00000000 00000000 00000000 00000000 00000000 00000000 00000000 00000000
[ 25.647960] c2e0 0000000c 00000000 00000000 00000000 00000000 00000000 00000001 00000000
[ 25.656106] 
[ 25.656108] R6: 0xd8b73f80:
[ 25.660351] 3f80 3fff6841 3fff6c41 3fffd841 3fffdc41 3fff7841 3fff7c41 3fff9841 3fff9c41
[ 25.668497] 3fa0 3fff8841 3fff8c41 00000000 00000000 00000000 00000000 00000000 00000000
[ 25.676643] 3fc0 38830811 38830c11 38831811 38831c11 38832811 38832c11 38833811 38833c11
[ 25.684788] 3fe0 38834811 38834c11 38835811 38835c11 38836811 38836c11 3fffe821 3fffec21
[ 25.692934] 4000 00000000 00000002 00000000 d8b2e080 c07a8bdc 00000000 00000015 d8b2e080
[ 25.701080] 4020 d8b74000 c07a8040 c07a8040 d8b2da00 d8b38380 00000000 d8b75de4 d8b75db8
[ 25.709226] 4040 c0549720 00000000 00000000 00000000 00000000 00000000 00010000 00000000
[ 25.717371] 4060 001be4c0 00000000 00000000 00000000 00000000 00000000 00000000 00000000
[ 25.725520] Process insmod (pid: 1069, stack limit = 0xd8b742f0)
[ 25.731499] Stack: (0xd8b75eb0 to 0xd8b76000)
[ 25.735833] 5ea0:                             00000000 bf002010 00000000 c0037494
[ 25.743980] 5ec0: 00000001 c00c9790 00000007 d8ae0d80 00000000 bf000154 bf00010c bf000154
[ 25.752126] 5ee0: bf00010c 00000001 d8b06d80 0000001c 00000001 c008f71c bf000118 d8b75fb0
[ 25.760272] 5f00: 00000000 c008e7c0 bf000230 0000003f d8b74000 e085024c e085024c 001abb66
[ 25.768418] 5f20: d863aea0 e0850000 00006a59 e0854ac4 e08548f7 e0856974 d8ae0c00 00000244
[ 25.776563] 5f40: 00000394 00000000 00000000 0000002c 0000002d 00000012 00000000 0000000f
[ 25.784708] 5f60: 00000000 00000000 00000000 00000000 00000000 00000000 00000000 c00d49b8
[ 25.792855] 5f80: 00000003 00000000 00000069 be96aea4 00000080 c00427e8 d8b74000 00000000
[ 25.801001] 5fa0: 00000000 c0042640 00000000 00000069 001c6ff8 00006a59 001abb66 7fffffff
[ 25.809146] 5fc0: 00000000 00000069 be96aea4 00000080 be96aea8 001abb66 be96aea8 00000000
[ 25.817292] 5fe0: 00000001 be96ab44 00030010 000095e4 60000010 001c6ff8 fb5fbeff fffbef77


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

内核崩溃时的堆栈 [<地址>](函数名)，这个信息对于调试很有帮助，**最后被调用的函数栈在最上面**，上面的oops的函数调用栈：

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