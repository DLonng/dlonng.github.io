---
layout:     post
title:      "LinuxDriver-Key中断"
subtitle:   "S5PV210 LinuxDriver Key"
date:       2017-03-24 18:00:00
author:     "陈登龙"
header-img: "img/post-bg-unix-linux.jpg"
catalog: true
tags:
    - LinuxDriver
---

# LinuxDriver-Key中断


**一，Key中断简介**

之前我们了解了在app中使用while循环来不断检测Key的状态，这种方法会占用几乎全部的CPU资源，非常的不可取，这次我们来学习下如何在Linux驱动中为Key注册中断处理程序，使用中断来返回按键的次数给用户空间。


**二，Key驱动编写**

与前面的Key驱动的框架大同小异，我们先看 **my_key_int.h:** 详细的注释都在代码中，大家还是以代码为主。
``` c
/* 必要的头文件 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <mach/map.h>
#include <mach/regs-gpio.h>
#include <plat/gpio-cfg.h>
#include <asm/uaccess.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <mach/irqs.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <asm/irq.h>


#define KEY_NUM 4
#define Tiny210_KEY_MAJOR 0
#define Tiny210_KEY_MINOR 0

#define DEV_COUNT 1
#define DEV_NAME  "Tiny210_KEY_INT"

static int key_create_device(void); 
static void key_destroy_device(void);

/* 因为要在打开文件的同时申请中断号，还要将数据送会用户空间，所以我们需要下面3个ops */
size_t key_read(struct file *, char __user *, size_t, loff_t *);
int key_release(struct inode *, struct file *);
int key_open(struct inode *, struct file *);


static int key_major = Tiny210_KEY_MAJOR;
static int key_minor = Tiny210_KEY_MINOR;
static dev_t key_dev_num;
static struct class *key_class = NULL;
static struct cdev key_cdev;
static struct file_operations key_fops = {
	.owner = THIS_MODULE,
	.open = key_open,
	.read = key_read,
	.release = key_release
};

/* 
 * 自定义的Key中断的描述结构，存储了每个Key的中断请求irq，flag，name
 * 这个irq准确来说应该是Key的GPIO，因为在request_irq之前需要将gpio转换为irq才可以。
 */
struct key_irq_desc {
	int irq;
	unsigned long flags;
	char *name;
};


/* 定义一个Key中断的描述数组，方便初始化Key中断 */
static struct key_irq_desc key_irqs[] = {
	/* 按键的GPIO，Key的电平触发方式：下降沿触发，中断名称 */
	{S5PV210_GPH2(0), IRQ_TYPE_EDGE_FALLING, "KEY1"},
	{S5PV210_GPH2(1), IRQ_TYPE_EDGE_FALLING, "KEY2"},
	{S5PV210_GPH2(2), IRQ_TYPE_EDGE_FALLING, "KEY3"},
	{S5PV210_GPH2(3), IRQ_TYPE_EDGE_FALLING, "KEY4"},
};

/* 4个Key的中断处理函数 */
irqreturn_t key1_handler(int, void *);
irqreturn_t key2_handler(int, void *);
irqreturn_t key3_handler(int, void *);
irqreturn_t key4_handler(int, void *);


/* 中断处理函数的函数指针数组 */
static irq_handler_t keys_handler[] = {
	key1_handler,
	key2_handler,
	key3_handler,
	key4_handler,
};

/* 存储4个Key的按下次数的数组 */
static int press_cnt[KEY_NUM] = { 0 };

/* 用于配合工作队列使用，来休眠对应的进程 */
static volatile int ev_press = 0;

/* 定义一个工作队列，主要用来休眠调用者的进程，关于工作队列的详细内容，我们后面介绍，这里会使用即可 */
static DECLARE_WAIT_QUEUE_HEAD(key_wait_queue);
```

**my_key_int.c:** 

在比较重要的地方，我加了很详细的注释。

``` c
#include "my_key_int.h"

/*
 * Key1 IRQ handler
 * 4个中断处理函数都是相同的，这里介绍这一个即可
 */
irqreturn_t key1_handler(int irq, void *dev_id)
{
	/* 得到每个中断的参数 */
	volatile int *tmp_press_cnt = (volatile int *)dev_id;
	/* 将对应Key的按下次数加1，表示该按键被按下一次 */
	*tmp_press_cnt += 1;
	
	/*
	* 将ev_press设置为1, 应为在read函数中，只有在ev_press不等于0的时候，用户空间才能获取数据
	* 这里将ev_press设置为1表示中断发生了，使得read函数可以继续运行下去，将数据copy到用户空间。
	*/
	ev_press = 1;
	/* 唤醒工作队列中正在睡眠的进程 */
	wake_up_interruptible(&key_wait_queue);

	/* 处理成功，返回IRQ_RETVAL(IRQ_HANDLED) */
	return IRQ_RETVAL(IRQ_HANDLED);
}


/*
 * Key2 IRQ handler
 */
irqreturn_t key2_handler(int irq, void *dev_id)
{
	volatile int *tmp_press_cnt = (volatile int *)dev_id;
	*tmp_press_cnt += 1;

	ev_press = 1;
	wake_up_interruptible(&key_wait_queue);

	return IRQ_RETVAL(IRQ_HANDLED);
}


/*
 * Key3 IRQ handler
 */
irqreturn_t key3_handler(int irq, void *dev_id)
{
	volatile int *tmp_press_cnt = (volatile int *)dev_id;
	*tmp_press_cnt += 1;

	ev_press = 1;
	wake_up_interruptible(&key_wait_queue);

	return IRQ_RETVAL(IRQ_HANDLED);
}



/*
 * Key4 IRQ handler
 */
irqreturn_t key4_handler(int irq, void *dev_id)
{
	volatile int *tmp_press_cnt = (volatile int *)dev_id;
	*tmp_press_cnt += 1;

	ev_press = 1;
	wake_up_interruptible(&key_wait_queue);

	return IRQ_RETVAL(IRQ_HANDLED);
}



/*
 * Create /dev/DEV_NAME, /sys/class/DEV_NAME, and alloc dev_num
 */
static int key_create_device(void)
{
	int ret = 0;
	int err = 0;

	cdev_init(&key_cdev, &key_fops);

	if (key_major) {
		key_dev_num = MKDEV(key_major, key_minor);
		err = register_chrdev_region(key_dev_num, DEV_COUNT, DEV_NAME);
		if (err < 0) {
			printk(KERN_ERR "register_chrdev_region() : failed!\n");
			return err;
		}
	} else {
		err = alloc_chrdev_region(&key_cdev.dev, key_minor, DEV_COUNT, DEV_NAME);
		if (err < 0) {
			printk(KERN_ERR "alloc_chrdev_region() : failed!\n");
			return err;
		}

		key_major = MAJOR(key_cdev.dev);
		key_minor = MINOR(key_cdev.dev);
		key_dev_num = key_cdev.dev;
		key_cdev.owner = THIS_MODULE;
	}

	ret = cdev_add(&key_cdev, key_dev_num, DEV_COUNT);
	key_class = class_create(THIS_MODULE, DEV_NAME);
	device_create(key_class, NULL, key_dev_num, NULL, DEV_NAME);
	
	return ret;
}

/*
 * Free /dev/DEV_NAME, /sys/class/DEV_NAME, and dev_num
 */
static void key_destroy_device(void)
{
	device_destroy(key_class, key_dev_num);

	if (key_class)
		class_destroy(key_class);

	unregister_chrdev_region(key_dev_num, DEV_COUNT);
}




/*
 * read fun, copy press_cnt array to usespace.
 */
size_t key_read(struct file *filp, char __user *buf, size_t count, loff_t *ppos)
{
	int err = 0;
	
	/* 
	* 当Key没有按下时，ev_press = 0
	* 这行代码在ev_press为0的时候，会将调用者进程加到key_wait_queue这个进程等待队列中     
	*/
	wait_event_interruptible(key_wait_queue, ev_press);
	/* 
	* 如果按键按下了，在IRQ handler中会将ev_press设置为1, 为了避免进程不断的触发中断，这里人为将ev_press设置为0
	* 也就是说，这句话的作用就是防止调用者进程重复触发中断。
	*/
	ev_press = 0;

	/* 将存储每个Key按下次数的数组copy到用户空间去 */
	err = copy_to_user(buf, (const void*)press_cnt, min(sizeof(press_cnt), count));

	return err ? -EFAULT : 0;
}


/*
 * Open /dev/DEV_NAME
 * request_irq: GPH2[0 - 3]
 */
int key_open(struct inode *pnode, struct file *filp)
{
	int i = 0;
	int err = 0;
	
	int tmp_irq = 0;

	for (i = 0; i < sizeof(key_irqs) / sizeof(key_irqs[0]); i++) {
		/* 将每个Key的GPIO转换为对应的irq */
		tmp_irq = gpio_to_irq(key_irqs[i].irq);
		/* 注册每个Key的中断，将press_cnt中对应位置的元素地址作为对应中断的dev_id, 也可以看做是中断的参数 */
		err = request_irq(tmp_irq, keys_handler[i], key_irqs[i].flags, key_irqs[i].name, (void*)&press_cnt[i]);
		if (err) {
			printk(KERN_ERR "request_irq err, i = %d, irq = %d\n", i, tmp_irq);
			break;
		}
		printk(KERN_DEBUG "KEY1 request_irq success = %d\n", tmp_irq);
	}

	/* 如果发生错误，就释放所有的中断 */
	if (err) {
		for(--i; i >= 0; i--) {
			tmp_irq = gpio_to_irq(key_irqs[i].irq);
			free_irq(tmp_irq, (void*)&press_cnt[i]);
		}
		return -EBUSY;
	}

	return 0;
}


/*
 * Release irq.
 */
int key_release(struct inode *pnode, struct file *filp)
{
	int i = 0;

	int tmp_irq = 0;

	/* 释放所有的中断请求 */
	for (i = 0; i < sizeof(key_irqs)/ sizeof(key_irqs[0]); i++) {
		tmp_irq = gpio_to_irq(key_irqs[i].irq);
		free_irq(tmp_irq, (void*)&press_cnt[i]);
	}

	return 0;
}



/*
 * Module init
 */
static int __init mykey_int_init(void)
{
	key_create_device();
	printk(KERN_DEBUG DEV_NAME "\tmykey_int_init ok!\n");
	return 0;
}



/*
 * Module exit
 */
static void __exit mykey_int_exit(void)
{
	key_destroy_device();
	printk(KERN_DEBUG DEV_NAME "\tmykey_int_exit ok!\n");
}


MODULE_LICENSE("GPL");

module_init(mykey_int_init);
module_exit(mykey_int_exit);


```


**Makefile:**

不要忘记更改内核路径
``` makefile
obj-m := my_key_int.o

KDIR := /home/orange/Desktop/linux-3.0.8

all:
	make -C $(KDIR) M=$(PWD) modules CROSS_COMPILE=arm-linux- ARCH=arm

clean:
	rm -f *.o *.ko *.order *.symvers *.mod.*
```

make之后会生成：**my_key_int.ko**。

**三，测试驱动**

**ket_int_test.c**

``` c
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>

int main(void)
{
	int i = 0;
	int ret = 0;
	int fd = 0;
	int press_cnt[4] = { 0 };

	/* 打开我们创建的设备文件 */
	fd = open("/dev/Tiny210_KEY_INT", O_RDWR);
	
	/* 以防万一，加上检查 */
	if (fd < 0) {
		printf("Can`t open /dev/Tiny210_KEY_INT!\n");
		return -1;
	}

	while (1) {
		/*
		* 因为我们在驱动的read函数中使用了工作队列，所以在按键没有按下的时候，这个进程就会进入休眠状态
		* 当有按键按下的时候，这个进程就会被唤醒，接着就会打印按键的按下次数。
		*/
		ret = read(fd, press_cnt, sizeof(press_cnt));

		/* 循环读出内核空间记录按键次数的数组，然后打印每个按键按下的次数 */
		for (i = 0; i < sizeof(press_cnt) / sizeof(press_cnt[0]); i++) {
			if (press_cnt[i])
				printf("Key%d has been pressed %d times!\n", i + 1, press_cnt[i]);
		}

		printf("\n");
	
	}
	close(fd);

	return 0;
}
```

静态编译：

``` swift
arm-linux-gcc -static key_int_test.c -o key_int_test
```

在开发板上测试可以发现，当模块加载到内核后，我们查看当前的内核中断的使用：

``` nimrod
cat /proc/interrupts
```
可以发现有我们注册的4个Key中断，然后运行key_int_test,每按下一次按键就会在控制台上打印4个Key的按下次数，说明实验成功。


**四，总结**

仔细观察会发现**按键会有抖动**，这个是硬件的原因，不过可以使用**内核定时器**来消除，这个问题我们后面再来解决，后面会介绍字符设备的高级操作，例如**poll，select以及内核定时器**等，敬请关注。



