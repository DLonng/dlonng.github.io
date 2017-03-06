---
layout:     post
title:      "UBoot-一个简单的myboot程序"
subtitle:   "ARM S5PV210 UBoot"
date:       2017-03-06 19:00:00
author:     "陈登龙"
header-img: "img/post-bg-uboot.jpg"
catalog: true
tags:
    - UBoot
---


# UBoot-简单的myboot程序

**一，myboot**

在了解了UBoot的格式后，我们知道**start.S**和**lowlevel_init.S**是底层初始化的关键的2个文件。这次我们自己编写这两文件来实现点亮一个LED灯。


**1.start.S**

``` 
.global _start
_start:
	#将前面的8个异常向量简单的指向处理到reset处
	b reset
	b reset
	b reset
	b reset
	b reset
	b reset
	b reset
	b reset

reset:
	#初始化LED的GPIO
	bl gpio_init
	#打开LED
	bl led_on
	
1:
	#跳到前面的标号1
	b 1b

gpio_init:
	ldr r11, =0xE0200280
	ldr r12, =0x00001111
	str r12, [r11]

	ldr r11, =0xE0200284
	ldr r12, =0xF
	str r12, [r11]
	mov pc, lr
```

**lowlevel_init.S**
实现开启LED灯的汇编代码
``` 
.global led_on 
led_on:
	ldr r11, =0xE0200284
	ldr r12, [r11]
	bic r12, r12, #(1 << 1)
	str r12, [r11]
	mov pc, lr

```

在编写完这两个文件后，我们需要进行编译连接，但是我们需要一个**链接脚本**文件来规定目标文件在可执行文件中的顺序。
**boot.lds:**
这里规定链接的启始地址是BL1阶段的地址：0xD0020010。
![BL1 Addr][1]


``` 
SECTIONS
{
	. = 0xD0020010;

	.text : {
		start.o
		*(.text)
	}

	.data : {
		*(.data)
	}

	.bss_start = .;
	
	.bss : {
		*(.bss)
	}
	
	.bss_end = .;
}
```
这是链接后的bin格式：
![LDS][2]


这是Makefile：

``` makefile
CC := arm-linux

myboot : start.o lowlevel_init.o
	$(CC)-ld -Tboot.lds $^ -o myboot.elf
	$(CC)-objcopy -O binary -S myboot.elf myboot.bin
	mkv210 myboot.bin myboot.16k

%.o : %.S
	$(CC)-gcc -c $< -o $@

mksd:
	sudo dd iflag=dsync oflag=dsync if=myboot.16k of=/dev/sdb seek=1

clean:
	rm -f *.elf *.o *.bin *.16k
```

编写完后，执行make编译，然后烧写到SD卡中：

``` 
make mksd
```

重启开发板，可以看到LED被点亮。说明我们的boot运行成功，而UBoot代码正是在这两个文件中添加许多的初始化信息，在UBoot中这两个文件通常不在同一个目录，但是没关系，有Makefile帮助我们解决。
所以，你应该理解start.S(首先运行的代码)和lowlevel_init.S(对应开发板的底层初始化代码)这两个文件的作用。





















 
 


  [1]: https://cheng-zhi.github.io/img/UBoot/post-2017-03-06-BL1Addr.png
  [2]: https://cheng-zhi.github.io/img/UBoot/post-2017-03-06-LDS.png
