---
layout:     post
title:      "LinuxDriver-Platform_Led"
subtitle:   "LinuxDriver Platform"
date:       2017-04-05 21:00:00
author:     "陈登龙"
header-img: "img/post-bg-unix-linux.jpg"
catalog: true
tags:
    - LinuxDriver
---


# **一，Platform简介**

Platform是Linux2.6内核之后添加的一套**设备驱动模型**，该模型和已经存在的IIC，SPI等平台设备具有相似的结构，只不过Platform是一种**虚拟的总线**，挂接在SoC内存空间的外设等依附与这种虚拟的总线，挂接在这种总线上的设备称为**platform_device**，而相应的设备驱动称之为**platform_driver**。

在开发挂接在Platform上的驱动时，我们需要遵循下面的步骤：

**1.编写driver -> 注册driver**

**2.编写device -> 注册device**


**二，Platform_Led**

这次我们来编写一个简单的Platform Led来熟悉下Platform设备驱动编写方法，这个Led的例子只是为了帮助我们理解Platform设备驱动的编程方法，并没有在实际的项目中应用。



这个代码中相比与普通的字符设备驱动最大的不同就是**多了platform_driver结构体**，并且在**模块init和exit中是对这个结构体进行注册和反注册**。
``` c
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>


#define DEV_NAME "Tiny210_LED"
#define DEV_COUNT 1

#define Tiny210_LED_MAJOR 0
#define Tiny210_LED_MINOR 0

/* 
 * Tiny210_LED device MAJOR and MINOR
 * if major = 0, using alloc_chrdev_region, else using register_chrdev_region.
 */
static int major = Tiny210_LED_MAJOR;
static int minor = Tiny210_LED_MINOR;

/* LED dev number */
static dev_t dev_num;

/* LED cdev */
static struct cdev led_cdev;

/* LED class pointer, create DEV_NAME node in /sys/class/DEV_NAME. */
static struct class *led_class = NULL;

/* Function declaration */
static int led_create_device(void);
static void led_destroy_device(void);

static volatile unsigned long *gpio_con = NULL;
static volatile unsigned long *gpio_dat = NULL;

static int led_probe(struct platform_device *pdev);
static int led_remove(struct platform_device *pdev);
static int led_open(struct inode *inode, struct file *file);
static ssize_t led_write(struct file *file, const char __user *buf, size_t count, loff_t * ppos);
static int led_create_device(void);


static int pin = 0;


/* Led fops */
static struct file_operations led_fops = {
	.owner = THIS_MODULE,
	.open  = led_open,
	.write = led_write,
};


/* Led platform driver ,这是相比普通字符设备驱动不同的地方，我们需要一个这样来的结构来注册平台驱动 */
struct platform_driver led_drv = {
	.probe  = led_probe,
	.remove = led_remove,
	.driver = {
	     /* 这个名称必须和device中的名称相同，否则设备和驱动不能匹配 */
		.name = "my_led",
	},
};


/*
 * Create DEV_NAME in /dev/DEV_NAME
 */
static int led_create_device(void)
{
	int ret = 0;
	int err = 0;

	/* Init cdev struct and fops */
	cdev_init(&led_cdev, &led_fops);

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
 * Led driver probe function 
 * 驱动探测函数，在设备初始化中调用已经初始化的driver中的probe函数来检测注册的驱动是否真实存在，如果存在就初始化相应的驱动 
 */
static int led_probe(struct platform_device *pdev)
{
	struct resource *res = NULL;

	/* 得到索引为0的设备的MEM资源地址 */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	/* 将MEM的地址映射到虚拟地址空间 */
	gpio_con = ioremap(res->start, res->end - res->start + 1);
	gpio_dat = gpio_con + 1;

	/* 创建设备号等操作 */
	led_create_device();	
	
	/* 我们操作第一个LED */
	pin = 0;
	
	return 0;
}


/* Led remove */
static int led_remove(struct platform_device *pdev)
{
	led_destroy_device();
	iounmap(gpio_con);
	return 0;
}

/* Led open */
static int led_open(struct inode *inode, struct file *file)
{
	unsigned int value = ioread32(gpio_con);
	value &= ~(0xF << (pin * 4));
	value |= (0x1 << (pin * 4));
	iowrite32(value, gpio_con);
	
	return 0;
}

/* Led write */
static ssize_t led_write(struct file *file, const char __user *buf, size_t count, loff_t * ppos)
{
	int usr_value = 0;

	unsigned int led_value = ioread32(gpio_dat);

	copy_from_user(&usr_value, buf, count);

	
	if (1 == usr_value)
		led_value |= (0x1 << (pin *4));
	else
		led_value &= ~(0x1 << (pin * 4));

	iowrite32(led_value, gpio_dat);

	return 0;
}

/* Module init */
static int __init my_led_drv_init(void)
{
	/*注册平台驱动 */
	platform_driver_register(&led_drv);	
	return 0;
}


/* Module exit */
static void __exit my_led_drv_exit(void)
{
	/* 反注册平台驱动 */
	platform_driver_unregister(&led_drv);
}


MODULE_LICENSE("GPL");
module_init(my_led_drv_init);
module_exit(my_led_drv_exit);

```



对应的Platform Device：

``` c
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/platform_device.h>

#define	MY_LED_DEV_NAME	"my_led"
#define	MY_LED_DEV_ID	-1

/* S5PV210上LED的控制寄存器首地址 */
#define GPJ2CON	0xE0200280

/* Led resource array */
static struct resource led_resource[] = {
	[0] = {
		.start  = GPJ2CON,
		/* 控制寄存器和数据寄存器加起来一共是8个字节, 所以这里的MEM一共寻址8个字节 */
		.end   = GPJ2CON + 8 - 1,
		.flags = IORESOURCE_MEM,
	},
};


/* 
 * Led release 
 * Every kobject must be have a release function 
 * 注意：在platform设备模型中必须要有release函数，因为platform反注册需要调用这个函数
 */
static void led_release(struct device *dev)
{

}


/* Led platform device 平台设备的结构体 */
static struct platform_device led_dev = {
	/* 这个名称必须和platform driver中的name相同，否则device不能匹配到相应的driver */
	.name = MY_LED_DEV_NAME,
	.id   = MY_LED_DEV_ID,
	.num_resources = ARRAY_SIZE(led_resource),
	.resource = led_resource,
	.dev = {
		.release = led_release,
	},
};


/* Led device init */
static int __init my_led_dev_init(void)
{
	/* 调用这个函数完成设备的注册 */
	platform_device_register(&led_dev);
	return 0;
}

/* Led device exit */
static void __exit my_led_dev_exit(void)
{
	/* 反注册函数 */
	platform_device_unregister(&led_dev);
}


MODULE_LICENSE("GPL");

module_init(my_led_dev_init);
module_exit(my_led_dev_exit);
```



用户测试程序test.c：

``` c
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>

/* led_test on
 * led_test off
 */
int main(int argc, char **argv)
{
	int fd;
	int val = 1;
	fd = open("/dev/Tiny210_LED", O_RDWR);
	if (fd < 0)
	{
		printf("can't open!\n");
	}
	if (argc != 2)
	{
		printf("Usage :\n");
		printf("%s <on|off>\n", argv[0]);
		return 0;
	}

	/* 比较用户的输入是on函数off，来打开或者关闭第一个LED */
	if (strcmp(argv[1], "on") == 0)
	{
		val  = 0;
	}
	else
	{
		val = 1;
	}
	
	write(fd, &val, 4);
	return 0;
}
```


**三，总结**

在了解了简单的platform的开发框架之后，我们再去开发LCD，USB等复杂的驱动就会有一个大致的思路，因为这些复杂的驱动使用的也是这种框架，变换的只是每种驱动的原理以及对寄存器的相关操作，你在开发一种驱动之前，必须要了解驱动的原理，通常查看内核已经实现的驱动代码是一个非常好的方法。








