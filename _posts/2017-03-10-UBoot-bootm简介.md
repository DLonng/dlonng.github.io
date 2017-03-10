---
layout:     post
title:      "UBoot-bootm简介"
subtitle:   "S5PV210 UBoot bootm"
date:       2017-03-10 22:00:00
author:     "陈登龙"
header-img: "img/post-bg-uboot.jpg"
catalog: true
tags:
    - UBoot
---

# UBoot-bootm简介

**一，bootm简介**

bootm可以将内存中的操作系统运行起来，你只需要**传递内存中OS的启始地址**即可，像下面这样：

这里假设我们的uboot将OS的镜像加载到内存中的**0x30008000**地址处
``` 
bootm 0x30008000
```

**二，使用bootm加载Linux**

下面我们就用之前已经移植成功的UBoot来加载我们的Linux，你需要有一个zImage或者uImage，如果有uImage就更好了，可以直接加载，因为我们需要对zImage进行处理。

uImage就是在zImage的头部加上一个64B的头信息：
**zImage + 64B Header = uImage**

**1.利用zImage制作uImage**

使用uboot目录中的**tools**目录下的：**mkimage**工具，下面的括号中是对应的解释。
``` 
mkimage -n(uImage镜像名称) 'my_kernel' -A(架构) arm -O(OS类型) linux -T(镜像类型) kernel -C(压缩类型) none -a(加载地址) 0x30008000 -e(实际的zImage地址是去掉前面的64B头信息) 0x30008040 -d(指定输入文件) zImage uImage(输出文件)
```

结果如下：

```  
	Image Name:   my_kernel
	Created:      Fri Mar 10 12:14:16 2017
	Image Type:   ARM Linux Kernel Image (uncompressed)
	Data Size:    4824900 Bytes = 4711.82 kB = 4.60 MB
	Load Address: 30008000
	Entry Point:  30008040
```
我们可以分析出uImage的头信息(uImage的头信息是**大端**格式)：

``` 
hexdump -C uImage | less
```
可以看到前64B中有**my_kernel**这个字符串，说明我们制作uImage成功。


**2.烧写uImage到SD卡第1000个扇区开始处**

前面的扇区留给uboot.16K和uboot.bin
``` 
	sudo dd iflag=dsync oflag=dsync if=uImage of=/dev/sdb seek=1000
```
 
**3.将uImage加载到DDR**

**vim board/samsung/goni/mmc_relocate.c**， 之前我们在这个函数中只是将uboot.bin拷贝到DDR中，现在我们需要将整个OS的镜像也拷贝到DDR中，增加最后一个for循环：

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
	int i = 0;
	/* uImage在SD中的扇区号 */
	int uImage_sec_no = 1000;
	/* uImage要拷贝到DDR中的首地址 */
	unsigned long uImage_dest = 0x30008000;
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
		/***************************************下面是增加的拷贝OS到DDR中的部分***********************************************/
		/* 每次拷贝1KB = 2个扇区，到uImage_dest开始处，一共拷贝5MB，我们uImage = 4.6MB */
		for (i = 0; i < 5 * 1024; i++) {
			copy_bl2(0, uImage_sec_no + i * 2, 2, (unsigned int*)(uImage_dest + i * 1024), 0);
		}
	}
}

```

**4.测试**

当UBoot启动后，查看uImage在DDR中的前64B的头信息内存：

``` nginx
md.b 0x30008000
```
可以看到带有**my_kernel的头**被显示出来，然后执行：

``` nginx
bootm 0x30008000 
``` 
即可启动内核，我们这里测试的uImage有问题，不能正常启动，但是只要能够启动Linux就可以说明我们的bootm命令已经能够成功load Linuxkernel了。



