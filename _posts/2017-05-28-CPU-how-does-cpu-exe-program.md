---
title: How does CPU execute program ?
date : 2017-05-28 18:00:00
---

# How does CPU execute program ?
***
> 版权声明：本文为 cheng-zhi 原创文章，可以随意转载，但必须在明确位置注明出处！ 

## 你知道 CPU 如何执行你的程序吗？
你写过那么多的代码，但是你知道 CPU 是如何执行它们的吗？你不要找借口说业务逻辑不需要我知道这些，别骗你自己了，你只是安于现状而已，了解计算机的底层对我们的开发能力会有一种间接的提高，这就是修炼内功的作用。

下面以一个简单的程序为例，来简单介绍下 CPU 是如何执行我们写的代码的，当然我们写的代码编译出来有非常多的指令，但我这里只分析有几条最为关键的指令，我们只需要把 CPU 整个执行逻辑搞清楚即可。

要分析的程序：计算 50.00 * 0.1 的值
```c
int main(void) {
  double A = 50.00;
  double B = 0.1;
  double C = A * B;
  return 0;
}
```

你写完了代码，如果你没有跟别人交流的话，很可能只有你一个人看了，抱歉 CPU 也不认识你写的啥 ......

## CPU 并不认识你写的程序
不要惊讶，记住：**CPU 只认识二进制代码**，也就是一大串的 0 和 1，至于你写的那些很牛比的代码，CPU 大哥并不买帐，那怎么执行我们的程序呢？这就是编译器存在的原因了，我们的编译器将我们写的程序编译成可供 CPU 执行的二进制代码，并分配指令和内存的地址等复杂的工作。

所以假设上面的这段程序经过编译器编译过后，生成下面的一些信息：


![](http://cheng-zhi.me/images/cpu_table.png)


上面表格中的地址值都可以看做是编译器为我们指定的（假设 CPU 为 32 Bit，则一条指令 4 个字节，一个 int 也是 4 个字节，所以下面的指令地址和变量地址的偏移都为 4）：
- 100，104，108，112 是我们要分析的 4 条指令的内存地址
- 2000，2004，2008 是分别存储 A，B，C 这 3 个变量的内存地址

上面的 `Asm code` 这一列表示的是汇编指令，如果你没有学习过汇编语言，那么这里你可以简单的把一条汇编指令等价于一串具有特定功能的 0 和 1 的序列，因为最后需要通过一个叫做**汇编器**的东西来将汇编指令转换成 0 和 1 的序列，这样一来，CPU 最后执行的就是具有特定功能的二进制代码了。

在了解了 CPU 只会执行二进制代码后，我们需要再来了解一个程序大体的执行流程。

## 程序大体的执行流程
一个程序的执行流程总体上可以分为 3 个步骤：**磁盘 -> RAM 内存 -> CPU**

程序从磁盘拷贝到内存中，CPU 从内存中取指令，然后解码，之后执行，最后将执行结果写回内存，这个过程可以用下面这个流程图来表示：


![](http://cheng-zhi.me/images/cpu_ins_loop.png)

大体的流程了解即可，底层的东西深入比较复杂，有兴趣可以深入，下面来分析 CPU 的执行程序的具体流程。


## 具体流程分析
分析分为许多个小的步骤，只要了解了其中一个 CPU 的**执行循环**，其他的都好理解，下面开始：

#### Step 1
CPU 将**程序计数器**设置为 `LOAD A, 2000` 这条指令的内存地址 100 ，表示现在开始执行这条指令，并把这条指令拷贝到 CPU 内部的指令寄存器 `Instruction Register` 中，把这里的指令都理解为 0 和 1 组成的序列即可。


![Step 1](http://upload-images.jianshu.io/upload_images/4613385-4902a9fdd4d96d44.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)


#### Step 2
CPU 执行指令寄存器中的指令，并按照上面表格第二列的功能将内存地址 2000 位置的值拷贝到寄存器 A 中，最后寄存器 A 中的值就是 50.00 了。

![Step 2](http://upload-images.jianshu.io/upload_images/4613385-4abb5faa9217faa9.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)


#### Step 3
Step 2 后，CPU 已经执行完了一条指令，这时需要将程序计数器中的值更新为**下一条指令的内存地址**，这里就将 `LOAD B, 2004` 这条指令的内存地址 104 拷贝到程序计数器中去了，表示接下来需要执行这条指令的相关工作。

![Step 3](http://upload-images.jianshu.io/upload_images/4613385-3fc4953e83bba75a.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)


#### Step 4
这一步将内存地址 104 处的指令 `LOAD B, 2004` 拷贝到 CPU 的指令寄存器中。


![Step 4](http://upload-images.jianshu.io/upload_images/4613385-79b9919ec66c7233.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)


#### Step 5
CPU 执行指令寄存器中的指令 `LOAD B, 2004`，将内存地址 2004 处的值拷贝到 CPU 的寄存器 B 中。

![Step 5](http://upload-images.jianshu.io/upload_images/4613385-edd41b0d757da644.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)


#### Step 6
CPU 更新程序计数器的值为下一条指令 `Multiphy A, B, C` 的内存地址 108。


![Step 6](http://upload-images.jianshu.io/upload_images/4613385-d52f1d675246ebb7.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)


#### Step 7
CPU 将内存地址 108 处的指令 `Multiphy A, B, C` 拷贝到指令寄存器中。

![Step 7](http://upload-images.jianshu.io/upload_images/4613385-9228a9ff43d69dd8.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)


#### Step 8
CPU 执行当前指令寄存器中的指令 `Multiphy A, B, C`，利用内部的逻辑运算单元（ALU）来计算 A 和 B 的乘积并把计算结果拷贝到寄存器 C 中。

![Step 8](http://upload-images.jianshu.io/upload_images/4613385-2070d6e08df5ae14.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)

#### Step 9
CPU 更新程序计数器的值为下一条指令 `STORE C, 2008` 的内存地址 112。

![Step 9](http://upload-images.jianshu.io/upload_images/4613385-5c62c12a8a5043df.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)


#### Step 10
CPU 将内存地址 112 处的指令 `STORE C, 2008` 拷贝到指令寄存器中。

![Step 8](http://upload-images.jianshu.io/upload_images/4613385-862b8a3eb775afb1.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)

### Step 11
CPU 执行当前指令寄存器中的指令，将寄存器 C 中的值拷贝到内存地址 2008 处，即将计算结果 5.00 拷贝到内存地址 2008 处，这个程序到这里就执行完毕了。


![Step 11](http://upload-images.jianshu.io/upload_images/4613385-bc2c19e655531c70.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)


### Step 12
CPU 是不能停止工作的，因此它将继续重复上面的步骤：**更新程序计数器的值为下一条指令的内存地址 -> 拷贝下一条指令到指令寄存器 -> 执行指令 -> ...**


![Step 12](http://upload-images.jianshu.io/upload_images/4613385-aea4439316eb2b0b.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)


## 小结
仔细分析下，大体的流程不是很难（底层细节还有很多，很难，如果你都理解的差不多，那你可能就是传说中的大神吧 ~），只不过我们平常写的程序太复杂导致我们不敢去分析，相信你看了这个例子之后应该对 CPU 如何执行程序有些眉目了吧，如果你有兴趣踏入计算机的世界，欢迎你入坑...

