---
layout:     post
title:      "LinuxDriver-LED"
subtitle:   "S5PV210 LinuxDriver LED Char"
date:       2017-03-23 10:00:00
author:     "陈登龙"
header-img: "img/post-bg-unix-linux.jpg"
catalog: true
tags:
    - LinuxDriver
---


# LinuxDriver-LED

**一，LED简介**

LED驱动可以说是我们开发驱动的第一个例子，LED属于字符设备驱动，应用我们之前介绍的字符设备驱动的编程模型，本次借鉴内核在gpio上提供的API来实现使用Linux App控制LED的亮灭。

**二，LED驱动框架搭建**

按照我们之前的约定，写出下面的LED驱动的框架。
**my_led.h**
``` c
#include <linux/init.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <mach/map.h>
#include <mach/regs-gpio.h>
#include <plat/gpio-cfg.h>
#include <asm/uaccess.h>

#define DEV_NAME "Tiny210_LED"
#define DEV_COUNT 1

#define Tiny210_LED_MAJOR 0
#define Tiny210_LED_MINOR 0

#define PARAM_SIZE 3

#define LED_ON  0
#define LED_OFF 1

#define LED_NUM 4

#define USER_SPACE_LED_ON  '1'
#define USER_SPACE_LED_OFF '0'
 
/* Store 4 Led state`s array. */
static unsigned char led_state_array[LED_NUM] = { 0 };
/* 4 Led init state, this set LED_ON. */
static int led_init_state = LED_ON;

/* 
 * Tiny210_LED device MAJOR and MINOR
 * if major = 0, using alloc_chrdev_region, else using register_chrdev_region.
 */
static int major = Tiny210_LED_MAJOR;
static int minor = Tiny210_LED_MINOR;

/* Using in test array params to this module. */
static int param_size = PARAM_SIZE;
static char *params[] = {"string1", "string2", "string3"};

/* LED dev number */
static dev_t dev_num;

/* LED cdev */
static struct cdev led_cdev;

/* LED class pointer, create DEV_NAME node in /sys/class/DEV_NAME. */
static struct class *led_class = NULL;

/* Function declaration */ 
size_t tiny210_led_write(struct file *pfile, const char __user *buf, size_t count, loff_t *offset);
long tiny210_led_unlocked_ioctl(struct file *pfile, unsigned int cmd, unsigned long arg);


/* file operations */
static struct file_operations cdev_fops = {
	.write = tiny210_led_write,	
	.unlocked_ioctl = tiny210_led_unlocked_ioctl,
};
```




**my_led.c**

``` c
#include "my_led.h"


/*
 * Tiny210 led write.
 * Using: echo 1111 > /dev/Tiny210_LED to start 4 led, and echo 0000 > /dev/Tiny210_LED to close 4 led.
 */
size_t tiny210_led_write(struct file *pfile, const char __user *buf, size_t count, loff_t *offset)
{
	 
	return 0;
	
}


/*
 * Tiny210 led unlocked ioctl.
 */
long tiny210_led_unlocked_ioctl(struct file *pfile, unsigned int cmd, unsigned long arg)
{
 
	return 0;
}

 

/*
 * Led module init function.
 */
static int __init myled_init(void)
{
 
	return 0;
}

 


/*
 * Led module exit function.
 */
static void __exit myled_exit(void)
{
 
}


MODULE_LICENSE("GPL");

/*
 * S_IRUGO | S_IWUSR : set this param at /sys/module/, and only root can modify this param.
 */
module_param(led_init_state, int, S_IRUGO | S_IWUSR);
module_param_array(params, charp, &param_size, S_IRUGO | S_IWUSR);

module_init(myled_init);
module_exit(myled_exit);
```

之后，我们需要实现上面的这几个函数，并且在 **sys/class/** 下增加我们的LED设备的节点，所以我们需要创建class。在这次的LED驱动中，我使用的是内核已经为s5pv210封装的gpio，不用自己使用实际的led地址然后映射到虚拟地址了，大大简化了驱动的开发，我们应该学会使用内核提供了功能。

在填充了每个函数后，像下面这样，其中我增加了几个初始化的函数，主要是为了使得代码比较清晰，并且代码上都有详细的注释，就不多解释了。

**my_led.h**

``` c
#include <linux/init.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <mach/map.h>
#include <mach/regs-gpio.h>
#include <plat/gpio-cfg.h>
#include <asm/uaccess.h>

#define DEV_NAME "Tiny210_LED"
#define DEV_COUNT 1

#define Tiny210_LED_MAJOR 0
#define Tiny210_LED_MINOR 0

#define PARAM_SIZE 3

#define LED_ON  0
#define LED_OFF 1

#define LED_NUM 4

#define USER_SPACE_LED_ON  '1'
#define USER_SPACE_LED_OFF '0'

/* Ioctl cmd */
#define LED_MAGIC 'L'
#define LED_ON_CMD  _IOW(LED_MAGIC, 0, int)
#define LED_OFF_CMD _IOW(LED_MAGIC, 1, int)



/* Store 4 Led state`s array. */
static unsigned char led_state_array[LED_NUM] = { 0 };
/* 4 Led init state, this set LED_ON. */
static int led_init_state = LED_ON;

/* 
 * Tiny210_LED device MAJOR and MINOR
 * if major = 0, using alloc_chrdev_region, else using register_chrdev_region.
 */
static int major = Tiny210_LED_MAJOR;
static int minor = Tiny210_LED_MINOR;

/* Using in test array params to this module. */
static int param_size = PARAM_SIZE;
static char *params[] = {"string1", "string2", "string3"};

/* LED dev number */
static dev_t dev_num;

/* LED cdev */
static struct cdev led_cdev;

/* LED class pointer, create DEV_NAME node in /sys/class/DEV_NAME. */
static struct class *led_class = NULL;

/* Function declaration */
static int led_create_device(void);
static int led_init_gpio(int led_default); 
static void led_destroy_device(void);
size_t tiny210_led_write(struct file *pfile, const char __user *buf, size_t count, loff_t *offset);
long tiny210_led_unlocked_ioctl(struct file *pfile, unsigned int cmd, unsigned long arg);


/* file operations */
static struct file_operations cdev_fops = {
	.write = tiny210_led_write,	
	.unlocked_ioctl = tiny210_led_unlocked_ioctl,
};
```


**my_led.c**

``` c
#include "my_led.h"


/*
 * Tiny210 led write.
 * Using: echo 1111 > /dev/Tiny210_LED to start 4 led, and echo 0000 > /dev/Tiny210_LED to close 4 led.
 */
size_t tiny210_led_write(struct file *pfile, const char __user *buf, size_t count, loff_t *offset)
{
	unsigned int tmp = count;
	unsigned int i = 0;
	//memset(led_state_array, 0, 4);

	if (count > LED_NUM)
		tmp = LED_NUM;
	
	/* Get userspace input. */
	if (copy_from_user(led_state_array, buf, tmp)){
		return -EFAULT;
	} else {
		for (i = 0; i < LED_NUM; i++) {
			if (USER_SPACE_LED_ON == led_state_array[i])
				gpio_set_value(S5PV210_GPJ2(i), LED_ON);
			else
				gpio_set_value(S5PV210_GPJ2(i), LED_OFF);
		}
		return count;
	}
}


/*
 * Tiny210 led unlocked ioctl.
 */
long tiny210_led_unlocked_ioctl(struct file *pfile, unsigned int cmd, unsigned long arg)
{
	//printk(KERN_DEBUG "cmd = %d\n", cmd);	
	switch (cmd) {
	//case LED_OFF_CMD:
	//case LED_ON_CMD:
	case 0:
	case 1:
		/* arg : 0 - 3 */
		if (arg > (LED_NUM - 1))
			return -EINVAL;
		//printk(KERN_DEBUG "cmd = %d\n", cmd);	
		/* Open or close led depend cmd. */
		//if (LED_ON_CMD == cmd)
		if (1 == cmd)
			gpio_set_value(S5PV210_GPJ2(arg), LED_ON);
		else
			gpio_set_value(S5PV210_GPJ2(arg), LED_OFF);

		return 0;
	}	
	return 0;
}


/*
 * Create DEV_NAME in /dev/DEV_NAME
 */
static int led_create_device(void)
{
	int ret = 0;
	int err = 0;

	/* Init cdev struct and fops */
	cdev_init(&led_cdev, &cdev_fops);

	/* Because major = 0, we use alloc_chrdev_region. */
	if (major > 0) {
		dev_num = MKDEV(major, minor);
		err = register_chrdev_region(dev_num, DEV_COUNT, DEV_NAME);
		if (err < 0) {
			printk(KERN_ERR "register_chrdev_region() : failed!!!\n");
			return err;
		}
	} else {
		/* Alloc cdev, minor also = 0. */
		err = alloc_chrdev_region(&led_cdev.dev, minor, DEV_COUNT, DEV_NAME);
		if (err < 0) {
			printk(KERN_ERR "alloc_chrdev_region() : failed!!!\n");
			return err;
		}

		/* Get major, minor and dev_num. */
		major = MAJOR(led_cdev.dev);
		minor = MINOR(led_cdev.dev);
		dev_num = led_cdev.dev;
	}

	/* Register led cdev to linux kernel. */
	ret = cdev_add(&led_cdev, dev_num, DEV_COUNT);

	/* Create /sys/class/Tiny210_LED node. */
	led_class = class_create(THIS_MODULE, DEV_NAME);

	/* Create /dev/Tiny210_LED device file. */
	device_create(led_class, NULL, dev_num, NULL, DEV_NAME);
	return ret;
}


/*
 * Request 4 led GPIO and set default output value.
 * LED0 - GPJ2_0
 * LED1 - GPJ2_1
 * LED2 - GPJ2_2
 * LED3 - GPJ2_3
 */
static int led_init_gpio(int led_default) 
{
	int err = 0;
	int i = 0;

	/* Request 4 led GPIO. */
	err = gpio_request(S5PV210_GPJ2(0), "LED0");
	if (err) {
		printk(KERN_ERR "Failed to request GPJ2_0 for led0!!!\n");
		return err;
	}

	err = gpio_request(S5PV210_GPJ2(1), "LED1");
	if (err) {
		printk(KERN_ERR "Failed to request GPJ2_1 for led1!!!\n");
		return err;
	}

	err = gpio_request(S5PV210_GPJ2(2), "LED2");
	if (err) {
		printk(KERN_ERR "Failed to request GPJ2_2 for led2!!!\n");
		return err;
	}

	err = gpio_request(S5PV210_GPJ2(3), "LED3");
	if (err) {
		printk(KERN_ERR "Failed to request GPJ2_3 for led3!!!\n");
		return err;
	}

	/* Set GPIO module is output, and close all led. */
	for (i = 0; i < 4; i++) {
		gpio_direction_output(S5PV210_GPJ2(i), 1);
		gpio_set_value(S5PV210_GPJ2(i), led_default);
	}

	return 0;

}


/*
 * Led module init function.
 */
static int __init myled_init(void)
{
	/* Create /dev/DEV_NAME = /dev/Tiny210_LED node. */
	int ret = led_create_device();
	
	/* Init led gpio, close 4 led. */
	led_init_gpio(led_init_state);
	
	/* Print something msg. */
	printk(KERN_DEBUG DEV_NAME "\tinitialized\n");
	printk(KERN_DEBUG "led_init_state\t%d\n", led_init_state);
	printk(KERN_DEBUG "param0\t%s\n", params[0]);
	printk(KERN_DEBUG "param1\t%s\n", params[1]);
	printk(KERN_DEBUG "param2\t%s\n", params[2]);
	
	return ret;
}


/*
 * Delete /dev/DEV_NAME and /sys/class/DEV_NAME 
 */
static void led_destroy_device(void)
{
	/*
	 * NOTE : We create /dev/Tiny210_LED second, but we must destroy it first!
	 */
	device_destroy(led_class, dev_num);

	/* We create /sys/class/Tiny210_LED first, but we need destroy it second! */
	if (led_class) 
		class_destroy(led_class);

	/* Delete cdev struct from linux kernel. */
	unregister_chrdev_region(dev_num, DEV_COUNT);

	return;
	
}


/*
 * Led module exit function.
 */
static void __exit myled_exit(void)
{
	int i = 0;

	/* Free something that alloc in led_create_device function. */
	led_destroy_device();

	/* Free 4 GPIO. */
	for (i = 0; i < 4; i++) 
		gpio_free(S5PV210_GPJ2(i));
	
	/* Moduel exit ok!*/
	printk(KERN_DEBUG DEV_NAME "\texit!!!\n");
}


MODULE_LICENSE("GPL");

/*
 * S_IRUGO | S_IWUSR : set this param at /sys/module/, and only root can modify this param.
 */
module_param(led_init_state, int, S_IRUGO | S_IWUSR);
module_param_array(params, charp, &param_size, S_IRUGO | S_IWUSR);

module_init(myled_init);
module_exit(myled_exit);
```

这里没有使用Linux的标准ioctl的命令来控制led，因为加上后会出现问题，待下次解决。

编写完成后，我们需要Makefile:

``` makefile
obj-m := my_led.o

#My kernel path
KDIR := /home/orange/Desktop/linux-3.0.8 

all:
	make -C $(KDIR) M=$(PWD) modules CROSS_COMPILE=arm-linux- ARCH=arm

clean:
	rm -f *.o *.ko *.order *.symvers *.mod.*
```

编写完Makefile，直接进行make即可生成my_led.ko

**三，编写Linux app测试**

**test_ioctl.c**

``` c
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>

/* Ioctl cmd */
#define LED_MAGIC 'L'
#define LED_ON_CMD  _IOW(LED_MAGIC, 0, int)
#define LED_OFF_CMD _IOW(LED_MAGIC, 1, int)

int main(int argc, char ** argv)
{
	int fp = 0;
	int cmd = 0;
	int arg = 0;

	if (argc < 4) {
		printf("Usage : ioctl <dev_file> <cmd> <arg>");
		return 0;
	}

	cmd = atoi(argv[2]);
	arg = atoi(argv[3]);

	printf("dev:%s\n", argv[1]);
	printf("cmd:%d\n", cmd);
	printf("arg:%d\n", arg);

	fp = open(argv[1], 0);

	ioctl(fp, cmd, arg);

	/*
	if (1 == cmd)
		ioctl(fp, LED_ON_CMD, arg);
	else
		ioctl(fp, LED_OFF_CMD, arg);
	*/
	close(fp);

	return 0;	
}
```
根据用户的命令行输入，开启或者熄灭对应的LED，例如：**熄灭索引为1的LED，也就是第二个LED**。

``` gradle
test_ioctl /dev/Tiny210_LED 0 1
```

将我们的模块和app都下载NFS中，然后在开发板上insmod my_led.ko，运行上面的测试命令可以发现第二个LED被熄灭了说明测试成功。
我们可以看到这个目录:**/sys/class/Tiny210_LED**，这就是我们在驱动中创建的LED在sys/class下的节点文件。
查看 **/dev/Tiny210_LED** , 这个就是我们在驱动程序中动态创建的设备文件(内核建议设备文件都使用动态创建)。


**四，总结**

内核驱动的编写有很多需要注意的地方，这次的LED驱动只能算是学习使用，如果用于实际的项目还有许多需要改进的地方，我们下次再见。

