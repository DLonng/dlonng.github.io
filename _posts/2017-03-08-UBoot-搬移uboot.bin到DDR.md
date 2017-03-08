---
layout:     post
title:      "UBoot-搬移uboot.bin到DDR"
subtitle:   "S5PV210 UBoot DDR"
date:       2017-03-08 19:00:00
author:     "陈登龙"
header-img: "img/post-bg-uboot.jpg"
catalog: true
tags:
    - UBoot
---


# UBoot-搬移uboot.bin到DDR

**一，为何需要搬移？**

因为BL1阶段的uboot.16k只有16K大小，而实际的uboot有180多K，所以我们需要利用这16K代码来将整个uboot.bin搬移到DDR内存中去，然后**跳转到DDR中运行board_init_f**，即板级初始化前半部分。


**二，如何搬移？**

**1.**我们需要在**board/samsung/goni/**这个目录下添加一个文件：**mmc_relocate.c**，内容如下
下面的代码编写可以参考应用手册：**S5PV210_iROM_ApplicationNote_Preliminary_20091126**

``` c
/* 这个函数的定义在S5PV210的应用手册 */
typedef unsigned int (*copy_sd_mmc_to_mem) (unsigned int channel, unsigned int start_block, unsigned char block_size, unsigned int *trg, unsigned int init);

/*
 * 从SD卡中的第49个扇区开始，拷贝整个uboot.bin到SDRAM中，然后跳转到SDRAM中运行
 */
void copy_code_to_dram(void){
	/* 选择通道0,在应用手册中可以查到 */
	unsigned long ch = 0;
	/* 要拷贝到的内存地址 */
	unsigned long dest = 0x34800000;
	/* 从SD第49个扇区开始 */
	unsigned int sec_no = 49;
	unsigned int ret = 0;
	/* 0xD0037488是IROM中定义的存储通道的地址 */
	ch = *(volatile unsigned int*)(0xD0037488);
	/* D0037F98是IROM中定义的从SD卡拷贝数据到SDRAM的函数指针，这里定义了这个拷贝函数 */
	copy_sd_mmc_to_mem copy_bl2 = (copy_sd_mmc_to_mem)(*(unsigned int*)(0xD0037F98));

	if (ch == 0xEB000000){
		/*
		 *  这里为了对齐，每次拷贝128个扇区 = 128 * 512B = 0x10000 = 64KB。
		 */
		ret = copy_bl2(0, sec_no, 128, (unsigned int*)dest, 0);
		ret = copy_bl2(0, sec_no + 128, 128, (unsigned int*)(dest + 0x10000), 0);
		ret = copy_bl2(0, sec_no + 256, 128, (unsigned int*)(dest + 0x20000), 0);
	}
}
```

**2.我们需要修改同层目录的Makefile，增加对mmc_relocate.c的编译**

``` makefile
...
COBJS-y := goni.o onenand.o mmc_relocate.o
 ...
```


**3.我们在lowlevel_init.S中调用这个C函数，C函数默认就是全局的**

``` 
	/* For copy_sd_to_ddr */
237     bl  copy_code_to_dram
	/* 显示第一个字节的值 */
238     ldr r0, =0x34800000
239     bl display_addr_data  
```

**4.make测试**

我们先看下uboot.bin的第一个字节是什么，使用下面的命令：

``` 
hexdump -C u-boot.bin | less
```

可以看到下面的结果：
![Firtst Byte][1]


我们需要向SD卡烧写2次：

``` 
mkv210 uboot.bin uboot.16k
#将前16K烧写到第一个扇区开始，作为BL1阶段代码
sudo dd iflag=dsync oflag=dsync if=uboot.16k of=/dev/sdb seek=1
#将整个uboot.bin烧写到第49个扇区开始，作为BL2阶段代码
sudo dd iflag=dsync oflag=dsync if=uboot.bin of=/dev/sdb seek=49
```


第一个字节是：**0xEA000014**, 因为我们现在默认配置ARM是小端字节序。如果能够输出这个值，说明我们已经将uboot.bin整体搬移到DDR中了，但是你会发现UBoot现在卡住并没有继续运行，这是因为我们没有为S5PV210初始化uboot.bin在DDR内存中运行的条件，下面我们来进行相关的DDR的配置。

**三，为UBoot配置DDR**

**1.我们首先需要跳转到DDR的board_init_f这个函数去运行**
vim start.S
``` 
164     /* ldr  r0,=0x00000000 */
165     /* bl   board_init_f */
166 
167     ldr r0, =_start
168     ldr r1, _board_init_f_ofs
169 
170     add lr, r0, r1
171     ldr r0, =0x00000000
172     mov pc, lr
173 
174 _board_init_f_ofs:
175     .word board_init_f - _start

```
上面代码的意思是原来的代码还是调用片内的board_init_f，而现在我们要修改代码跳到片外的DDR中的board_init_f中运行，我们利用 
board_init_f - _start 的偏差来跳转到 board_init_f去执行。


**2.修改UBoot中对DDR的配置**

根据数据手册，我们的S5PV210的DRAM Base是0x20000000
``` 
 45 /* DRAM Base */
 46 /* #define CONFIG_SYS_SDRAM_BASE        0x30000000 */
 47 #define CONFIG_SYS_SDRAM_BASE       0x20000000

```

**3.我们只使用第一个Bank，并且S5PV210的DDR一共是512MB**

``` 
193 /* Goni has 3 banks of DRAM, but swap the bank */
194 
195 /* #define CONFIG_NR_DRAM_BANKS 3 */
196 #define CONFIG_NR_DRAM_BANKS    1 
...
200 /* #define PHYS_SDRAM_1_SIZE    (80 << 20) */       /* 80 MB in Bank #0 */
201 #define PHYS_SDRAM_1_SIZE   (512 << 20)     /* 512 MB in Bank #0 */
202 //我们只使用DMC0
203 //#define PHYS_SDRAM_2      0x40000000      /* mDDR DMC1 Bank #1 */
204 //#define PHYS_SDRAM_2_SIZE (256 << 20)     /* 256 MB in Bank #1 */
205 //#define PHYS_SDRAM_3      0x50000000      /* mDDR DMC2 Bank #2 */
206 //#define PHYS_SDRAM_3_SIZE (128 << 20)     /* 128 MB in Bank #2 */

```

**4.我们需要修改串口编号，我们使用串口0做为调试串口**

``` 
 63 //#define CONFIG_SERIAL2            1   /* use SERIAL2 */
 64 #define CONFIG_SERIAL0          1   /* use SERIAL0 */
```

**5.修改goni.c**

``` c
 50 int dram_init(void)
 51 {
 52 /*  gd->ram_size = PHYS_SDRAM_1_SIZE + PHYS_SDRAM_2_SIZE +
 53             PHYS_SDRAM_3_SIZE; */ 
 54
 55     gd->ram_size = PHYS_SDRAM_1_SIZE;
 56     return 0;
 57 }


 59 void dram_init_banksize(void)
 60 {   
 61     gd->bd->bi_dram[0].start = PHYS_SDRAM_1;
 62     gd->bd->bi_dram[0].size = PHYS_SDRAM_1_SIZE;
 63     //gd->bd->bi_dram[1].start = PHYS_SDRAM_2;
 64     //gd->bd->bi_dram[1].size = PHYS_SDRAM_2_SIZE;
 65     //gd->bd->bi_dram[2].start = PHYS_SDRAM_3;
 66     //gd->bd->bi_dram[2].size = PHYS_SDRAM_3_SIZE;
 67 }

```

重新make即可，然后烧写到SD卡，上电之后可以看到**串口终端有打印信息**，说明搬移uboot.bin到DDR成功，并且能够成功运行部分board_init_f的代码，但是最后还是卡住了，原因我们下次分析。





  [1]: https://cheng-zhi.github.io/img/UBoot/post-2017-03-08-UbootFB.png
