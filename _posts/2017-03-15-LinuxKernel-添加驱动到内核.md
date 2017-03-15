---
layout:     post
title:      "LinuxKernel-添加驱动到内核"
subtitle:   "S5PV210 Kernel CharDriver "
date:       2017-03-15 9:00:00
author:     "陈登龙"
header-img: "img/post-bg-linuxkernel.jpg"
catalog: true
tags:
    - LinuxKernel
---
 
 
# LinuxKernel-添加驱动到内核
 
 
 **一，内核驱动和裸机驱动的区别**
 
 我们在ARM裸机阶段编写过很多的裸机驱动，但是这些驱动能不能直接应用在Linux内核中呢？我们说对于比较简单的驱动，例如点亮一个LED灯是可以应用内核中的，但是当有比较复杂的驱动工作，在内核中运行就要考虑并发访问等问题了，所以编写内核中的驱动跟裸机驱动还是有区别的，但是基本的框架都是一样的，这次我们的**重点是如何将编写好的驱动添加到内核中，主要在于掌握添加的方法**。
 
 
**二，添加字符驱动到内核**

**1.建立文件夹**

我们将我们的驱动文件都放在**drivers/char/mytest**文件夹下：

``` stata
mkdir drivers/char/mytest
```

**2.拷贝驱动程序到mytest下**：我使用的是memdev.c, 你可以选择一个点亮LED的程序

``` stylus
cp .../memdev.c drivers/char/mytest
```

**3.编写当前目录下的Makefile**：可以参考上级目录的Makefile，内核中的很多代码都可以参考
	

``` javascript
obj-$(CONFIG_MYDEVTEST) += memdev.o
```
**注意：**

必须加上CONFIG_XXX

必须+=，如果使用=，则之前的Makefile生曾的.o就会被删除
 
 
**4.编辑上一级别Makefile，将当前目录编译进内核**

vim ../Makefile
``` javascript
obj-$(CONFIG_MYDEVTEST) += mytest/
```
我们可以直接将mytest编译到内核，当我们调试的时候可以这样使用：这样即使我们没有在make menuconfig中配置这个模块，这个模块也会编译进内核中，减少了调试的工作。

``` fix
../Makefile
obj-y += mytest/

./Makefile
obj-y += memdev.o
```


**5.编写Kconfig**

vim ./Kconfig

``` objectivec
menu "my dev device"
config MYDEVTEST #不需要前缀CONFIG_
	bool "Support my dev devices driver"
	help
		Support my dev devices driver for s5pv210
endmenu
```


 **6.将mytest下的Kconfig包含到内核中**：否则make menucofig中没有MYDEVTEST这个菜单项

vim ../Kconfig

``` mel
#source的路径必须是从内核主目录开始
source "drivers/char/myled/Kconfig"
```

**7.make menuconfig**

选中my dev devices的驱动，编译进内核
 
 **Devices Drivers -> Character Devices -> my dev test -> 按空格选中**，[*]表示编译到内核中。
 
 如下图：
 ![TestMenu][1]
 
 
 **8.重新编译内核**
	

``` go
make uImage
```

编译完成后可以在 **driver/char/mytest/** 查看有没有 **memdev.o**

当你编译成功后，driver/char/mytest/中的文件像下面这样，**必须有memdev.o**文件：
![MyTest][2]

**9.拷贝uImage到/tftpboot下**

我们使用tftp从PC上下载uImage，所以将uImage拷贝到/tftpboot下：

``` gradle
cp arch/arm/boot/uImage /tftpboot
```

**三，测试字符设备驱动**

**我们使用tftp + NFS的方式测试**

**1.在开发板上查看内核否运行了memdev驱动：**

``` nimrod
cat /proc/devices
```
从结果中找到memdev对应的主设备号，我的开发板是**253**。


**2.我们没有在memdev驱动中自动创建设备文件，需要手动创建设备文件**
	

``` gradle
mknod /dev/led_name c 253 0
```
led_name必须跟测试程序中fopen打开的文件名相同，这里我的是**memdev0**。

**3.运行测试程序**

我有两个测试程序，writemem, readmem。

**readmem.c**
``` cpp
#include <stdio.h>
#include <sys/types.h>
#include <fcntl.h>

int main(void) {
	int fd = 0;
	int dst = 0;

	/* Open memdev file. */
	fd = open("/dev/memdev0", O_RDWR);
	
	/* Read data wo dst. */
	read(fd, &dst, sizeof(int));

	/* Output to console. */
	printf("dst is %d\n", dst);

	/* Close memdev file. */
	close(fd);
	
	return 0;
}
```


**writemem.c**

``` cpp
#include <stdio.h>
#include <sys/types.h>
#include <fcntl.h>

int main(void) {
	int fd = 0;
	int src = 2017;
	
	/* Open memdev file. */
	fd = open("/dev/memdev0", O_RDWR);

	/* Write data to memedev. */
	write(fd, &src, sizeof(int));

	/* Close memdev file. */
	close(fd);
	return 0;
}
```

使用**静态编译**来编译上面2个程序，因为开发板上的根文件系统没有C库：

``` swift
arm-linux-gcc -static app.c -o app
```

首先运行writemem想设备文件中写入数据
``` gradle
./writemem
```

然后运行readmem读取刚才写入的数据
``` gradle
./readmem
```
结果是2017，我们在writemem中写入是就是2017，结果正确，实验成功。


**四，总结**

这种添加驱动到内核中的方法**必须要掌握**，这是移植内核的基础，通过自己添加Kconfig,Makefile等文件能够更好的帮助我们理解内核的编译路线。


  [1]: https://cheng-zhi.github.io/img/LinuxKernel/post-2017-03-15-TestMenu.png
  [2]: https://cheng-zhi.github.io/img/LinuxKernel/post-2017-03-15-MyTest.png
