---
layout:     post
title:      "LinuxDriver-轮询Key"
subtitle:   "S5PV210 LinuxDriver Key"
date:       2017-03-24 15:00:00
author:     "陈登龙"
header-img: "img/post-bg-unix-linux.jpg"
catalog: true
tags:
    - LinuxDriver
---

# LinuxDriver-轮询Key

**一，Key驱动简介**

Key也是属于字符设备，编程模型跟LED的几乎相同，唯一不同的是，我们的**LED是设置GPIO为输出状态**，但是我们的的**Key的GPIO要设置成输入状态**，因为我们要从用户空间读取Key的输入。这次介绍的是轮询Key的方法，这种方法在实际项目中根本不会用到，这里作为学习来说只是简单的介绍下，为后面的Key的中断实现做个铺垫，用来做下对比，帮助大家在以后更好的理解中断的优势。

**二，编写Key驱动**

**1.编写字符设备基本框架**

字符设备的基本框架相信大家都能够写出来吧，无非就是:模块初始化函数，模块退出函数，字符设备fops，字符设备的设备号的申请，以及相关的头文件的include。

**2.完整的Key驱动**

**my_key.h:**

``` c
/* 一些必要的头文件 */
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

#define KEY_NUM 4
#define Tiny210_KEY_MAJOR 0
#define Tiny210_KEY_MINOR 0

#define DEV_COUNT 1
#define DEV_NAME  "Tiny210_KEY"

static int key_create_device(void); 
static void key_destroy_device(void);
static int key_init_gpio(void);

/* 用户空间只需要read键盘的输入 */
size_t key_read(struct file *, char __user *, size_t, loff_t *);

static unsigned char key_state_array[KEY_NUM] = { 0 };

static int key_major = Tiny210_KEY_MAJOR;
static int key_minor = Tiny210_KEY_MINOR;
static dev_t key_dev_num;

static struct class *key_class = NULL;

static struct cdev key_cdev;

/* key的设备操作集合，只需要实现read即可 */
static struct file_operations key_fops = {
	.owner = THIS_MODULE,
	.read = key_read
};

```


**my_key.c:**

``` c
#include "my_key.h"

/*
 * 动态申请设备号，创建设备文件和class节点
 */
static int key_create_device(void)
{
	int ret = 0;
	int err = 0;
	
	cdev_init(&key_cdev, &key_fops);
	
	/* 默认的key_major为0, 所以我们使用动态申请 */
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
 * 初始化Key的GPIO
 * S5PV210上有8个按键，我这里只初始化前面4个。
 */
static int key_init_gpio(void)
{
	int err = 0;
	int i = 0;

	err = gpio_request(S5PV210_GPH2(0), "KEY0");
	if (err) {
		printk(KERN_ERR "Failed to request GPH2_0 for KEY0!\n");
		return err;
	}

	err = gpio_request(S5PV210_GPH2(1), "KEY1");
	if (err) {
		printk(KERN_ERR "Failed to request GPH2_1 for KEY1!\n");
		return err;
	}

	err = gpio_request(S5PV210_GPH2(2), "KEY2");
	if (err) {
		printk(KERN_ERR "Failed to request GPH2_2 for KEY2!\n");
		return err;
	}

	err = gpio_request(S5PV210_GPH2(3), "KEY3");
	if (err) {
		printk(KERN_ERR "Failed to request GPH2_3 for KEY3!\n");
		return err;
	}

	/* 设置4个GPIO为输入状态 */
	for (i = 0; i < KEY_NUM; i++) 
		gpio_direction_input(S5PV210_GPH2(i));
	
	return 0;
}

/*
 * read函数直接返回存储4个Key状态的数组给用户空间
 */
size_t key_read(struct file *filep, char __user *buf, size_t count, loff_t *ppos)
{
	int i = 0;
	for (i = 0; i < KEY_NUM; i++)
		key_state_array[i] = gpio_get_value(S5PV210_GPH2(i));

	if (copy_to_user(buf, key_state_array, count)) 
		return -EFAULT;
	else	
		return count;
}


/*
 * 释放设备文件和class节点以及设备号
 */
static void key_destroy_device(void)
{
	device_destroy(key_class, key_dev_num);

	if (key_class)
		class_destroy(key_class);
	
	unregister_chrdev_region(key_dev_num, DEV_COUNT);
}

/*
 * Key模块初始化函数
 */
static int __init mykey_init(void)
{
	key_create_device();
	key_init_gpio();
	printk(KERN_DEBUG DEV_NAME "\tmykey_init ok!\n");
	return 0;
}

/*
 * Key模块退出函数
 */
static void __exit mykey_exit(void)
{
	int i = 0;
	
	key_destroy_device();

	/* 释放Key的4个GPIO */
	for (i = 0; i < 4; i++)
			gpio_free(S5PV210_GPH2(i));

	printk(KERN_DEBUG DEV_NAME "\tmykey_exit!\n");
}

MODULE_LICENSE("GPL");

module_init(mykey_init);
module_exit(mykey_exit);
```


**Makefile:**

注意：你需要更改你的内核路径。

``` makefile
obj-m := my_key.o

KDIR := /home/orange/Desktop/linux-3.0.8

all:
	make -C $(KDIR) M=$(PWD) modules CROSS_COMPILE=arm-linux- ARCH=arm

clean:
	rm -f *.o *.ko *.order *.symvers *.mod.*
```

在这三个目录下直接：make即可编译该模块，**生成my_key.ko**。下面我们编写Linux App来测试我们的Key驱动。


**三，编写Linux App测试驱动**

**key_test.c:**

``` c
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>


int main(void)
{
	int fd_led = 0;
	int fd_key = 0;
	int cmd = 0;
	int arg = 0;
	unsigned char key_state[4] = { 0 };

	/* 打开LED和Key的设备文件 */
	fd_led = open("/dev/Tiny210_LED", 0);
	fd_key = open("/dev/Tiny210_KEY", 0);

	/*
	* 死循环，非常耗费CPU
	*/
	while (1) {
		/* 从内核空间读取4个Key的状态到key_state数组中 */
		read(fd_key, key_state, sizeof(key_state));
 
 		/* 判断每个Key的状态，如果按下就点亮对应的LED，你也可以直接打印信息*/
		if (!key_state[0])
			ioctl(fd_led, 1, 0);
			
		if (!key_state[1])
			ioctl(fd_led, 1, 1);
		
		if (!key_state[2])
			ioctl(fd_led, 1, 2);
		
		if (!key_state[3])
			ioctl(fd_led, 1, 3);
	}

	close(fd_led);
	close(fd_key);

	return 0;
}
```

**静态编译：**

``` swift
arm-linux-gcc -static key_test.c -o key_test
```

将my_key.ko以及my_led.ko通过NFS在开发板上插入到内核中，然后运行key_test, 按下一个按键，可以发现LED被点亮，那么实验就成功了。



