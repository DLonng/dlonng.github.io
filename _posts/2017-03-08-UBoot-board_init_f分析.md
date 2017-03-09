---
layout:     post
title:      "UBoot-board_init_f分析"
subtitle:   "S5PV210 UBoot board_init_f"
date:       2017-03-08 21:00:00
author:     "陈登龙"
header-img: "img/post-bg-uboot.jpg"
catalog: true
tags:
    - UBoot
---

# UBoot-board_init_f分析


**一，board_init_f简介**

当把整个uboot.bin从SD卡搬移到DDR中后，紧接着就是运行board_init_f这个函数，这个函数完成的主要功能是：**板级初始化前半部分**，
**板级初始化后半部分是在board_init_r**中完成的，我们以后介绍。
这次我们主要介绍board_init_f的主要功能。

**二，board_init_f分析**

board_init_f的分析主要是围绕下面这个**for循环**：

``` c
...
/* init_sequence是一个全局数组，里面存储的是一些初始化函数的指针,这里循环调用每个初始化的函数指针 */
for (init_fnc_ptr = init_sequence; *init_fnc_ptr; ++init_fnc_ptr) {
		/* 如果某个函数返回值不为0，说明该函数初始化错误，因此就挂起UBoot, */
		if ((*init_fnc_ptr)() != 0) {
			hang ();
		}
	}
...
```

arch/arm/lib: **init_sequence的定义**:

``` c
init_fnc_t *init_sequence[] = {
	arch_cpu_init,		/* basic arch cpu dependent setup */

#if defined(CONFIG_BOARD_EARLY_INIT_F)
	board_early_init_f,
#endif
#ifdef CONFIG_OF_CONTROL
	fdtdec_check_fdt,
#endif
	timer_init,		/* initialize timer */
#ifdef CONFIG_BOARD_POSTCLK_INIT
	board_postclk_init,
#endif
#ifdef CONFIG_FSL_ESDHC
	get_clocks,
#endif
	env_init,		/* initialize environment */
	init_baudrate,		/* initialze baudrate settings */
	serial_init,		/* serial communications setup */
	console_init_f,		/* stage 1 init of console */
	display_banner,		/* say that we are here */
#if defined(CONFIG_DISPLAY_CPUINFO)
	print_cpuinfo,		/* display cpu info (and speed) */
#endif
#if defined(CONFIG_DISPLAY_BOARDINFO)
	checkboard,		/* display board info */
#endif
#if defined(CONFIG_HARD_I2C) || defined(CONFIG_SOFT_I2C)
	init_func_i2c,
#endif
	dram_init,		/* configure available RAM banks */
	NULL,
};
```

其中，有些宏被关闭了，则宏中对应的函数就不需要初始化。

下面，我们一一介绍这里面主要的函数初始化，先看第一个：**1.arch_cpu_init**
这个函数是为了**得到当前CPU的ID**，**判断是C100还是C110**。
``` c
int arch_cpu_init(void)
{
	/* 得到CPU的ID*/
	s5p_set_cpu_id();

	return 0;
}

...

static inline void s5p_set_cpu_id(void)
{
	s5p_cpu_id = readl(S5PC100_PRO_ID);
	/* s5p_cpu_id & 0x00FFF000意思是取出中间的3个4位，得到的ID是C100或者是C110 */
	s5p_cpu_id = 0xC000 | ((s5p_cpu_id & 0x00FFF000) >> 12);
}
```


**2.timer_init**

用来初始化定时器，与主流程没有太大关系，有兴趣可以深入。
``` c
int timer_init(void)
{
	/* PWM Timer 4 */
	pwm_init(4, MUX_DIV_2, 0);
	pwm_config(4, 0, 0);
	pwm_enable(4);

	reset_timer_masked();

	return 0;
}

```


**3.env_init**

可以看到有许多的文件有定义env_init函数，是因为当**环境变量定义在不同的地方是就使用不同的配置**，例如当环境变量定义在dataflash时，就使用Env_flash.c中的初始化。默认使用**Env_nowhere.c**中的初始化。

``` c
/*
 * Initialize Environment use
 *
 * We are still running from ROM, so data use is limited
 */
int env_init(void)
{
	/* 使用默认的环境变量数组 */
	gd->env_addr	= (ulong)&default_environment[0];
	gd->env_valid	= 0;

	return 0;
}

...
/* default_environment的定义, 这里定义了每个环境变量的默认值，是通过s5p_goni.h中的宏来设置的 */
const uchar default_environment[] = {
#ifdef	CONFIG_BOOTARGS
	"bootargs="	CONFIG_BOOTARGS			"\0"
#endif
#ifdef	CONFIG_BOOTCOMMAND
	"bootcmd="	CONFIG_BOOTCOMMAND		"\0"
#endif
#ifdef	CONFIG_RAMBOOTCOMMAND
	"ramboot="	CONFIG_RAMBOOTCOMMAND		"\0"
#endif
#ifdef	CONFIG_NFSBOOTCOMMAND
	"nfsboot="	CONFIG_NFSBOOTCOMMAND		"\0"
#endif
#if defined(CONFIG_BOOTDELAY) && (CONFIG_BOOTDELAY >= 0)
	"bootdelay="	MK_STR(CONFIG_BOOTDELAY)	"\0"
#endif
#if defined(CONFIG_BAUDRATE) && (CONFIG_BAUDRATE >= 0)
	"baudrate="	MK_STR(CONFIG_BAUDRATE)		"\0"
#endif
#ifdef	CONFIG_LOADS_ECHO
	"loads_echo="	MK_STR(CONFIG_LOADS_ECHO)	"\0"
#endif
#ifdef	CONFIG_ETHADDR
	"ethaddr="	MK_STR(CONFIG_ETHADDR)		"\0"
#endif
#ifdef	CONFIG_ETH1ADDR
	"eth1addr="	MK_STR(CONFIG_ETH1ADDR)		"\0"
#endif
#ifdef	CONFIG_ETH2ADDR
	"eth2addr="	MK_STR(CONFIG_ETH2ADDR)		"\0"
#endif
#ifdef	CONFIG_ETH3ADDR
	"eth3addr="	MK_STR(CONFIG_ETH3ADDR)		"\0"
#endif
#ifdef	CONFIG_ETH4ADDR
	"eth4addr="	MK_STR(CONFIG_ETH4ADDR)		"\0"
#endif
#ifdef	CONFIG_ETH5ADDR
	"eth5addr="	MK_STR(CONFIG_ETH5ADDR)		"\0"
#endif
#ifdef	CONFIG_ETHPRIME
	"ethprime="	CONFIG_ETHPRIME			"\0"
#endif
#ifdef	CONFIG_IPADDR
	"ipaddr="	MK_STR(CONFIG_IPADDR)		"\0"
#endif
#ifdef	CONFIG_SERVERIP
	"serverip="	MK_STR(CONFIG_SERVERIP)		"\0"
#endif
#ifdef	CONFIG_SYS_AUTOLOAD
	"autoload="	CONFIG_SYS_AUTOLOAD		"\0"
#endif
#ifdef	CONFIG_PREBOOT
	"preboot="	CONFIG_PREBOOT			"\0"
#endif
#ifdef	CONFIG_ROOTPATH
	"rootpath="	CONFIG_ROOTPATH			"\0"
#endif
#ifdef	CONFIG_GATEWAYIP
	"gatewayip="	MK_STR(CONFIG_GATEWAYIP)	"\0"
#endif
#ifdef	CONFIG_NETMASK
	"netmask="	MK_STR(CONFIG_NETMASK)		"\0"
#endif
#ifdef	CONFIG_HOSTNAME
	"hostname="	MK_STR(CONFIG_HOSTNAME)		"\0"
#endif
#ifdef	CONFIG_BOOTFILE
	"bootfile="	CONFIG_BOOTFILE			"\0"
#endif
#ifdef	CONFIG_LOADADDR
	"loadaddr="	MK_STR(CONFIG_LOADADDR)		"\0"
#endif
#ifdef	CONFIG_CLOCKS_IN_MHZ
	"clocks_in_mhz=1\0"
#endif
#if defined(CONFIG_PCI_BOOTDELAY) && (CONFIG_PCI_BOOTDELAY > 0)
	"pcidelay="	MK_STR(CONFIG_PCI_BOOTDELAY)	"\0"
#endif
#ifdef	CONFIG_ENV_VARS_UBOOT_CONFIG
	"arch="		CONFIG_SYS_ARCH			"\0"
	"cpu="		CONFIG_SYS_CPU			"\0"
	"board="	CONFIG_SYS_BOARD		"\0"
#ifdef CONFIG_SYS_VENDOR
	"vendor="	CONFIG_SYS_VENDOR		"\0"
#endif
#ifdef CONFIG_SYS_SOC
	"soc="		CONFIG_SYS_SOC			"\0"
#endif
#endif
#ifdef	CONFIG_EXTRA_ENV_SETTINGS
	CONFIG_EXTRA_ENV_SETTINGS
#endif
	"\0"
};


```

我们配置下**include/configs/s5p_goni.h**,使用**NOWHERE**的环境配置

``` makefile
215 //#define CONFIG_ENV_IS_IN_ONENAND  1
216 #define CONFIG_ENV_IS_NOWHERE   1
```


**4.init_baudrate**

初始化波特率

``` c
static int init_baudrate(void)
{
	/* 取名称为baudrate的值，如果取不到就使用s5p_goni.h中的CONFIG_BAUDRATE = 115200 作为默认配置 */
	gd->baudrate = getenv_ulong("baudrate", 10, CONFIG_BAUDRATE);
	return 0;
}
```

**5.serial_init**

串口初始化，这个初始化**很重要**。

``` c
int serial_init(void)
{
	return get_current()->init();
}
```


get_current的定义：
``` c
static struct serial_device *get_current(void)
{
	struct serial_device *dev;

	if (!(gd->flags & GD_FLG_RELOC) || !serial_current) {
		/*根据前面的gd相关配置，分析条件可知下面的代码会被执行 */
		dev = default_serial_console();

		/* We must have a console device */
		if (!dev)
			panic("Cannot find console");
	} else
		dev = serial_current;
	return dev;
}

```

串口设备结构体，**serial_device描述了一个独立的串口**：

``` c
struct serial_device {
	/* enough bytes to match alignment of following func pointer */
	char name[16];

	int  (*init) (void);
	int  (*uninit) (void);
	void (*setbrg) (void);
	int (*getc) (void);
	int (*tstc) (void);
	void (*putc) (const char c);
	void (*puts) (const char *s);
	/* 将多个串口连接成链表 */
#if CONFIG_POST & CONFIG_SYS_POST_UART
	void (*loop) (int);
#endif

	struct serial_device *next;
};
```

default_serial_console的定义：我们使用串口0
``` c
__weak struct serial_device *default_serial_console(void)
{
/* 在s5p_goni.h中配置了CONFIG_SERIAL0这个宏，所以就返回串口0的地址 */
#if defined(CONFIG_SERIAL0)
	return &s5p_serial0_device;
#elif defined(CONFIG_SERIAL1)
	return &s5p_serial1_device;
#elif defined(CONFIG_SERIAL2)
	return &s5p_serial2_device;
#elif defined(CONFIG_SERIAL3)
	return &s5p_serial3_device;
#else
#error "CONFIG_SERIAL? missing."
#endif
}
```


s5p_serial0_device的定义：

``` c
DECLARE_S5P_SERIAL_FUNCTIONS(0);
struct serial_device s5p_serial0_device =
	INIT_S5P_SERIAL_STRUCTURE(0, "s5pser0");
```
INIT_S5P_SERIAL_STRUCTURE的定义：

``` c
#define INIT_S5P_SERIAL_STRUCTURE(port, name) { \
	name, \    //s5pser0
	s5p_serial##port##_init, \ //s5p_serial0_init
	NULL, \			   //NULL
	s5p_serial##port##_setbrg, \ //s5p_serial0_setbrg
	s5p_serial##port##_getc, \   //s5p_serial0_getc
	s5p_serial##port##_tstc, \   //s5p_serial0_tstc
	s5p_serial##port##_putc, \   //s5p_serial0_putc
	s5p_serial##port##_puts, }   //s5p_serial0_puts

```


DECLARE_S5P_SERIAL_FUNCTIONS(0);的定义：port = 0
``` c
/* Multi serial device functions */
#define DECLARE_S5P_SERIAL_FUNCTIONS(port) \
int s5p_serial##port##_init(void) { return serial_init_dev(port); } \		//serial_init_dev(0)
void s5p_serial##port##_setbrg(void) { serial_setbrg_dev(port); } \		//serial_setbrg_dev(0)
int s5p_serial##port##_getc(void) { return serial_getc_dev(port); } \		//serial_getc_dev(0)
int s5p_serial##port##_tstc(void) { return serial_tstc_dev(port); } \		//serial_tstc_dev(0)
void s5p_serial##port##_putc(const char c) { serial_putc_dev(c, port); } \	//serial_putc_dev(0)
void s5p_serial##port##_puts(const char *s) { serial_puts_dev(s, port); }	//serial_puts_dev(0)


```


**serial_init_dev**等串口函数的定义：

``` c
/*
 * Initialise the serial port with the given baudrate. The settings
 * are always 8 data bits, no parity, 1 stop bit, no start bits.
 */
int serial_init_dev(const int dev_index)
{
	/* 在裸机阶段我们定义过，其实就是UART的串口的寄存器结构体，下面的初始化也就是裸机阶段的对串口寄存器的配置 */
	struct s5p_uart *const uart = s5p_get_base_uart(dev_index);

	/* reset and enable FIFOs, set triggers to the maximum */
	writel(0, &uart->ufcon);
	writel(0, &uart->umcon);
	/* 8N1 */
	writel(0x3, &uart->ulcon);
	/* No interrupts, no DMA, pure polling */
	writel(0x245, &uart->ucon);

	serial_setbrg_dev(dev_index);

	return 0;
}
```

所以实际的串口调用过程：
**get_current() -> init => get_current() -> s5p_serial0_init -> serial_init_dev(0)**
最终调用到了serial_init_dev，这个就UART的驱动初始化函数。

**6.console_init_f** 
``` c
/* Called before relocation - use serial functions */
int console_init_f(void)
{
	/* 设置含有控制台 */
	gd->have_console = 1;

#ifdef CONFIG_SILENT_CONSOLE
	if (getenv("silent") != NULL)
		gd->flags |= GD_FLG_SILENT;
#endif

	print_pre_console_buffer();

	return 0;
}
```


**7.display_banner**

显示具体参数信息,可以对照串口输出的数据查看。

``` c
static int display_banner(void)
{
	printf("\n\n%s\n\n", version_string);
	debug("U-Boot code: %08lX -> %08lX  BSS: -> %08lX\n",
		   _TEXT_BASE,
		   _bss_start_ofs + _TEXT_BASE, _bss_end_ofs + _TEXT_BASE);
	#ifdef CONFIG_MODEM_SUPPORT
		debug("Modem Support enabled\n");
	#endif
	#ifdef CONFIG_USE_IRQ
		debug("IRQ Stack: %08lx\n", IRQ_STACK_START);
		debug("FIQ Stack: %08lx\n", FIQ_STACK_START);
	#endif

	return (0);
}
```

**8.print_cpuinfo**

打印CPU信息

``` c
int print_cpuinfo(void)
{
	char buf[32];
	/* 在串口输出"S5PC110@400MHz" */
	printf("CPU:\t%s%X@%sMHz\n",
			s5p_get_cpu_name(), s5p_cpu_id,
			strmhz(buf, get_arm_clk()));

	return 0;
}
```


**9.checkboard**

``` c
int checkboard(void) 
{
	/* 在串口输出：Board:        Goni 换行*/
	puts("Board:\tGoni\n");
	return 0;
}
```

**10.init_func_i2c**

初始化IIC

``` c
static int init_func_i2c(void)
{
	/* 在串口显示IIC的信息 */
	puts("I2C:   ");
	i2c_init(CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SLAVE);
	puts("ready\n");
	return (0);
}
```

**11.dram_init**

配置DRAM的大小，我们之前有一步就是要修改这个地方，还记得吗？

``` c
int dram_init(void)
{
	gd->ram_size = PHYS_SDRAM_1_SIZE + PHYS_SDRAM_2_SIZE + PHYS_SDRAM_3_SIZE;
	return 0;
}
```

**注意**：DRAM的大小信息以及之后板子打印的信息都是是在board_init_r函数中打印的，borad_init_f只打印到IIC的调试信息，所以这里没有输出DRAM的参数信息。


for循环之后就是对gd的一些配置，到最后需要对uboot代码进行重定位：**relocate_code**,为了提高效率，UBoot使用汇编代码来重定位。

vim .../start.S
``` c
/*
 * void relocate_code (addr_sp, gd, addr_moni)
 *
 * This "function" does not return, instead it continues in RAM
 * after relocating the monitor code.
 *
 */
ENTRY(relocate_code)
	mov	r4, r0	/* save addr_sp */
	mov	r5, r1	/* save addr of gd */
	mov	r6, r2	/* save addr of destination */

	/* Set up the stack						    */
stack_setup:
	mov	sp, r4

	adr	r0, _start
	cmp	r0, r6
	moveq	r9, #0		/* no relocation. relocation offset(r9) = 0 */
	beq	clear_bss		/* skip relocation */
	mov	r1, r6			/* r1 <- scratch for copy_loop */
	ldr	r3, _image_copy_end_ofs
	add	r2, r0, r3		/* r2 <- source end address	    */

copy_loop:
	ldmia	r0!, {r9-r10}		/* copy from source address [r0]    */
	stmia	r1!, {r9-r10}		/* copy to   target address [r1]    */
	cmp	r0, r2			/* until source end address [r2]    */
	blo	copy_loop

	/*
	 * fix .rel.dyn relocations
	 */
	ldr	r0, _TEXT_B没ASE		/* r0 <- Text base */
	sub	r9, r6, r0		/* r9 <- relocation offset */
	ldr	r10, _dynsym_start_ofs	/* r10 <- sym table ofs */
	add	r10, r10, r0		/* r10 <- sym table in FLASH */
	ldr	r2, _rel_dyn_start_ofs	/* r2 <- rel dyn start ofs */
	add	r2, r2, r0		/* r2 <- rel dyn start in FLASH */
	ldr	r3, _rel_dyn_end_ofs	/* r3 <- rel dyn end ofs */
	add	r3, r3, r0		/* r3 <- rel dyn end in FLASH */
fixloop:
	ldr	r0, [r2]		/* r0 <- location to fix up, IN FLASH! */
	add	r0, r0, r9		/* r0 <- location to fix up in RAM */
	ldr	r1, [r2, #4]
	and	r7, r1, #0xff
	cmp	r7, #23			/* relative fixup? */
	beq	fixrel没
	cmp	r7, #2			/* absolute fixup? */
	beq	fixabs
	/* ignore unknown type of fixup */
	b	fixnext
fixabs:
	/* absolute fix: set location to (offset) symbol value */
	mov	r1, r1, LSR #4		/* r1 <- symbol index in .dynsym */
	add	r1, r10, r1		/* r1 <- address of symbol in table */
	ldr	r1, [r1, #4]		/* r1 <- symbol value */
	add	r1, r1, r9		/* r1 <- relocated sym addr */
	b	fixnext
fixrel:
	/* relative fix: increase location by offset */
	ldr	r1, [r0]
	add	r1, r1, r9
fixnext:
	str	r1, [r0]
	add	r2, r2, #8		/* each rel.dyn entry is 8 bytes */
	cmp	r2, r3
	blo	fixloop
	b	clear_bss
_rel_dyn_start_ofs:
	.word __rel_dyn_start - _start
_rel_dyn_end_ofs:
	.word __rel_dyn_end - _start
_dynsym_start_ofs:
	.word __dynsym_start - _start

clear_bss:
	ldr	r0, _bss_start_ofs
	ldr	r1, _bss_end_ofs
	mov	r4, r6			/* reloc addr */
	add	r0, r0, r4
	add	r1, r1, r4
	mov	r2, #0x00000000		/* clear			    */

clbss_l:cmp	r0, r1			/* clear loop... */
	bhs	clbss_e			/* if reached end of bss, exit */
	str	r2, [r0]
	add	r0, r0, #4
	b	clbss_l
clbss_e:

/*
 * We are done. Do not return, instead branch to second part of board
 * initialization, now running from RAM.
 */
jump_2_ram:
/*
 * If I-cache is enabled invalidate it
 */
#ifndef CONFIG_SYS_ICACHE_OFF
	mcr	p15, 0, r0, c7, c5, 0	@ invalidate icache
	mcr     p15, 0, r0, c7, c10, 4	@ DSB
	mcr     p15, 0, r0, c7, c5, 4	@ ISB
#endif
/*
 * Move vector table
 */
#if !defined(CONFIG_TEGRA20)
	/* Set vector address in CP15 VBAR register */
	ldr     r0, =_start
	add     r0, r0, r9
	mcr     p15, 0, r0, c12, c0, 0  @Set VBAR
#endif /* !Tegra20 */

	/* 注意：我们之前从BL1阶段跳转到board_init_f也是使用这种跳转方式，相当优雅 */
	ldr	r0, _board_init_r_ofs
	adr	r1, _start
	add	lr, r0, r1
	add	lr, lr, r9
	/* setup parameters for board_init_r */
	mov	r0, r5		/* gd_t */
	mov	r1, r6		/* dest_addr */
	/* jump to it ... */
	mov	pc, lr

_board_init_r_ofs:
	.word board_init_r - _start
ENDPROC(relocate_code)
```

**在uboot.bin重定位之后，就跳转到board_init_r中去执行**，这个函数我们下次分析。
