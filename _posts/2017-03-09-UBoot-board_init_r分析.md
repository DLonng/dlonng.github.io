---
layout:     post
title:      "UBoot-board_init_r分析"
subtitle:   "S5PV210 UBoot board_init_r"
date:       2017-03-09 17:00:00
author:     "陈登龙"
header-img: "img/post-bg-uboot.jpg"
catalog: true
tags:
    - UBoot
---
 
# UBoot-board_init_r分析

**一，board_init_r简介**

上次我们分析了board_init_f函数，这次我们简单分析下board_init_r函数，该函数进行UBoot的板级初始化的后半部分, 在UBoot进行代码重定位后调用。

**二，board_init_r主要函数分析**

**1.enable_caches**

就是输出一段信息，与主流程没有太大关系

arch/arm/lib/cache.c:
``` c

/* 声明一个别名 */
void enable_caches(void) __attribute__((weak, alias("__enable_caches")));
	
/*
 * Default implementation of enable_caches()
 * Real implementation should be in platform code
 */
void __enable_caches(void)
{
	puts("WARNING: Caches not enabled\n");
}
```


**2.board_init**

主要是初始化一个gpio的全局变量

``` c
int board_init(void)
{
	/* Set Initial global variables */
	s5pc110_gpio = (struct s5pc110_gpio *)S5PC110_GPIO_BASE;

	gd->bd->bi_arch_number = MACH_TYPE_GONI;
	gd->bd->bi_boot_params = PHYS_SDRAM_1 + 0x100;

#if defined(CONFIG_PMIC)
	pmic_init();
#endif
	return 0;
}
```

**3.serial_initialize**

主要是注册串口

``` c
void serial_initialize(void)
{
...
#if defined(CONFIG_S5P)
	/* 注册串口，将所有的串口链接成一个链表 */
	serial_register(&s5p_serial0_device);
	serial_register(&s5p_serial1_device);
	serial_register(&s5p_serial2_device);
	serial_register(&s5p_serial3_device);
#endif
... 
	/* 得到默认的串口名称，将全局串口设置为这个串口 */
	serial_assign(default_serial_console()->name);
}
```


**serial_register的定义:**

``` c
void serial_register(struct serial_device *dev)
{
#ifdef CONFIG_NEEDS_MANUAL_RELOC
	dev->init += gd->reloc_off;
	dev->setbrg += gd->reloc_off;
	dev->getc += gd->reloc_off;
	dev->tstc += gd->reloc_off;
	dev->putc += gd->reloc_off;
	dev->puts += gd->reloc_off;
#endif

	dev->next = serial_devices;
	serial_devices = dev;
}

```

**serial_assign的定义：**

``` c
int serial_assign(const char *name)
{
	struct serial_device *s;

	for (s = serial_devices; s; s = s->next) {
		if (strcmp(s->name, name) == 0) {
			/* 将找到的串口赋值给全局的串口变量 */
			serial_current = s;
			return 0;
		}
	}

	return 1;
}
```


**4.mem_malloc_init**

初始化堆的内存

``` c
void mem_malloc_init(ulong start, ulong size)
{
	mem_malloc_start = start;
	mem_malloc_end = start + size;
	mem_malloc_brk = start;

	memset((void *)mem_malloc_start, 0, size);
}
```


**5.mmc_initialize**

MMC与主流程关系不大，有兴趣可以深入

``` c
int mmc_initialize(bd_t *bis)
{
	INIT_LIST_HEAD (&mmc_devices);
	cur_dev_num = 0;

	if (board_mmc_init(bis) < 0)
		cpu_mmc_init(bis);

	print_mmc_devices(',');

	return 0;
}
```


**6.env_relocate**

将环境变量重新初始化， 其实还是使用默认的环境变量
``` c
void env_relocate(void)
{ 
...
	if (gd->env_valid == 0) {
#if defined(CONFIG_ENV_IS_NOWHERE) || defined(CONFIG_SPL_BUILD)
		/* Environment not changable */
		set_default_env(NULL);
#else
		bootstage_error(BOOTSTAGE_ID_NET_CHECKSUM);
		set_default_env("!bad CRC");
#endif
	} else {
		env_relocate_spec();
	}
}
```
**set_default_env的定义：**

``` c
void set_default_env(const char *s)
{
	/*
	 * By default, do not apply changes as they will eventually
	 * be applied by someone else
	 */
	int do_apply = 0;
	if (sizeof(default_environment) > ENV_SIZE) {
		puts("*** Error - default environment is too large\n\n");
		return;
	}

	if (s) {
		if (*s == '!') {
			printf("*** Warning - %s, "
				"using default environment\n\n",
				s + 1);
		} else {
			/*
			 * This set_to_default was explicitly asked for
			 * by the user, as opposed to being a recovery
			 * mechanism.  Therefore we check every single
			 * variable and apply changes to the system
			 * right away (e.g. baudrate, console).
			 */
			do_apply = 1;
			puts(s);
		}
	} else {
		/* 注意：我们的串口上会打印这句话 */
		puts("Using default environment\n\n");
	}
	
	/* 将default_environment中的所有的环境变量做成一张Hash表格，方便以后加快速度查找 */
	if (himport_r(&env_htab, (char *)default_environment,
			sizeof(default_environment), '\0', 0,
			0, NULL, do_apply) == 0)
		error("Environment import failed: errno = %d\n", errno);

	/* 说明环境变量已经准备完毕 */
	gd->flags |= GD_FLG_ENV_READY;
}
```


**7.stdio_init**

标准输入输出初始化

``` c
int stdio_init (void)
{
 ...

	/* Initialize the list 双向链表的标准实现，初始化stdio_dev的链表 */
	INIT_LIST_HEAD(&(devs.list));

...

	drv_system_init ();
#ifdef CONFIG_SERIAL_MULTI
	serial_stdio_init ();
#endif

...
	return (0);
}
```


**8.drv_system_init**

初始化stdio的操作函数
``` c
static void drv_system_init (void)
{
	struct stdio_dev dev;

	memset (&dev, 0, sizeof (dev));

	strcpy (dev.name, "serial");
	dev.flags = DEV_FLAGS_OUTPUT | DEV_FLAGS_INPUT | DEV_FLAGS_SYSTEM;
	dev.putc = serial_putc;
	dev.puts = serial_puts;
	dev.getc = serial_getc;
	dev.tstc = serial_tstc;
	/* 将局部dev复制到堆中 */
	stdio_register (&dev);

 ...
}

/* serial_putc的定义：这个函数我们之前分析过 */
void serial_putc(const char c)
{
	get_current()->putc(c);
}
```

 **stdio_register的定义：**
```c
int stdio_register (struct stdio_dev * dev)
{
	struct stdio_dev *_dev;
	
	/* 拷贝dev到_dev*/
	_dev = stdio_clone(dev);
	if(!_dev)
		return -1;
	/* 链接到全局变量devs双向链表尾部 */
	list_add_tail(&(_dev->list), &(devs.list));
	return 0;
}

```


**stdio_clone的定义：**

``` c
struct stdio_dev* stdio_clone(struct stdio_dev *dev)
{
	struct stdio_dev *_dev;

	if(!dev)
		return NULL;
	/* 在堆中分配一块大小为sizeof(struct stdio_dev)的空间，并清0 */
	_dev = calloc(1, sizeof(struct stdio_dev));

	if(!_dev)
		return NULL;
	/* 拷贝dev的信息到堆中的_dev */
	memcpy(_dev, dev, sizeof(struct stdio_dev));
	strncpy(_dev->name, dev->name, 16);

	
	/* 返回克隆后的堆中的_dev地址 */
	return _dev;
}
```


**9.serial_stdio_init**

``` c
void serial_stdio_init(void)
{
	struct stdio_dev dev;
	struct serial_device *s = serial_devices;
	/* 遍历串口链表，将每个串口都复制到堆空间的stdio_dev中 */
	while (s) {
		memset(&dev, 0, sizeof(dev));

		strcpy(dev.name, s->name);
		dev.flags = DEV_FLAGS_OUTPUT | DEV_FLAGS_INPUT;
		
		/* 串口驱动与stdio操作函数的对应关系 */
		dev.start = s->init;
		dev.stop = s->uninit;
		dev.putc = s->putc;
		dev.puts = s->puts;
		dev.getc = s->getc;
		dev.tstc = s->tstc;

		stdio_register(&dev);

		s = s->next;
	}
}
```


**10.jumptable_init**

初始化函数的跳转表，比较复杂，有兴趣可以深入。

**11.console_init_r** 

控制台初始化后半部分

``` c
/* Called after the relocation - use desired console functions */
int console_init_r(void)
{
	struct stdio_dev *inputdev = NULL, *outputdev = NULL;
	int i;
	struct list_head *list = stdio_get_list();
	struct list_head *pos;
	struct stdio_dev *dev;

#ifdef CONFIG_SPLASH_SCREEN
	/*
	 * suppress all output if splash screen is enabled and we have
	 * a bmp to display. We redirect the output from frame buffer
	 * console to serial console in this case or suppress it if
	 * "silent" mode was requested.
	 */
	if (getenv("splashimage") != NULL) {
		if (!(gd->flags & GD_FLG_SILENT))
			outputdev = search_device (DEV_FLAGS_OUTPUT, "serial");
	}
#endif

	/* Scan devices looking for input and output devices */
	list_for_each(pos, list) {
		dev = list_entry(pos, struct stdio_dev, list);

		if ((dev->flags & DEV_FLAGS_INPUT) && (inputdev == NULL)) {
			inputdev = dev;
		}
		if ((dev->flags & DEV_FLAGS_OUTPUT) && (outputdev == NULL)) {
			outputdev = dev;
		}
		if(inputdev && outputdev)
			break;
	}

	/* Initializes output console first */
	if (outputdev != NULL) {
		console_setfile(stdout, outputdev);
		console_setfile(stderr, outputdev);
#ifdef CONFIG_CONSOLE_MUX
		console_devices[stdout][0] = outputdev;
		console_devices[stderr][0] = outputdev;
#endif
	}

	/* Initializes input console */
	if (inputdev != NULL) {
		console_setfile(stdin, inputdev);
#ifdef CONFIG_CONSOLE_MUX
		console_devices[stdin][0] = inputdev;
#endif
	}

	gd->flags |= GD_FLG_DEVINIT;	/* device initialization completed */

	stdio_print_current_devices();

	/* Setting environment variables */
	for (i = 0; i < 3; i++) {
		setenv(stdio_names[i], stdio_devices[i]->name);
	}

#if 0
	/* If nothing usable installed, use only the initial console */
	if ((stdio_devices[stdin] == NULL) && (stdio_devices[stdout] == NULL))
		return 0;
#endif

	return 0;
}
```



**console_setfile的定义：**

``` c
static int console_setfile(int file, struct stdio_dev * dev)
{
	int error = 0;

	if (dev == NULL)
		return -1;

	switch (file) {
	case stdin:
	case stdout:
	case stderr:
		/* Start new device */
		if (dev->start) {
			error = dev->start();
			/* If it's not started dont use it */
			if (error < 0)
				break;
		}

		/* Assign the new device (leaving the existing one started) */
		stdio_devices[file] = dev;

		/*
		 * Update monitor functions
		 * (to use the console stuff by other applications)
		 */
		switch (file) {
		case stdin:
			gd->jt[XF_getc] = dev->getc;
			gd->jt[XF_tstc] = dev->tstc;
			break;
		case stdout:
			gd->jt[XF_putc] = dev->putc;
			gd->jt[XF_puts] = dev->puts;
			gd->jt[XF_printf] = printf;
			break;
		}
		break;

	default:		/* Invalid file ID */
		error = -1;
	}
	return error;
}
```

 **stdio_print_current_devices的定义：** 打印当前串口的信息,UBoot中输出的串口信息就是在这里打印的。

``` c
void stdio_print_current_devices(void)
{
#ifndef CONFIG_SYS_CONSOLE_INFO_QUIET
	/* Print information */
	puts("In:    ");
	if (stdio_devices[stdin] == NULL) {
		puts("No input devices available!\n");
	} else {
		printf ("%s\n", stdio_devices[stdin]->name);
	}

	puts("Out:   ");
	if (stdio_devices[stdout] == NULL) {
		puts("No output devices available!\n");
	} else {
		printf ("%s\n", stdio_devices[stdout]->name);
	}

	puts("Err:   ");
	if (stdio_devices[stderr] == NULL) {
		puts("No error devices available!\n");
	} else {
		printf ("%s\n", stdio_devices[stderr]->name);
	}
#endif /* CONFIG_SYS_CONSOLE_INFO_QUIET */
}
```


**12.interrupt_init** 

中断初始化

``` c
int interrupt_init (void)
{
	/*
	 * setup up stacks if necessary
	 */
	IRQ_STACK_START_IN = gd->irq_sp + 8;

	return 0;
}
```


**13.enable_interrupts**

enable_interrupts有两种定义，我们没有定义使用IRQ的宏，所以使用下面的定义

``` c
void enable_interrupts (void)
{
	return;
}
```

**14.main_loop**

``` c
/* main_loop() can return to retry autoboot, if so just run it again. */
	for (;;) {
		main_loop();
	}
```

这些函数就是board_init_r中比较重要的函数了，可以好好分析下，这也是UBoot运行的主流程，再后面就是main_loop的分析了，下次见。




