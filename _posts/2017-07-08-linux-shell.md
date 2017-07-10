---
title: Linux 命令实现及调用机制  
date : 2017-07-08 12:00:00
---


# Linux 命令实现及调用机制

***
> 版权声明：本文为 {{ site.name }} 原创文章，可以随意转载，但必须在明确位置注明出处！ 

## 为何要了解 Linux 命令机制？
如果你每天都在使用 Linux 命令，那么你了解命令的基本原理吗？你学习 Linux 命令的时候，老师告诉你这些命令在哪里了么？系统是如何调用这些命令的呢？如何写一个命令自己调用呢？如果你有兴趣，请看看我的见解。

##  解析 ls 命令
如果你经常使用 Linux，相信你对 `ls` 这个命令不会陌生，这次就借这个命令来分析 Linux 命令的实现和调用机制。

#### ls 命令在哪里？
在终端键入下面的命令来查找 `ls` 的位置：
```
whereis ls

# 结果
ls: /bin/ls /usr/share/man/man1/ls.1.gz
```
可以看到有一条 `/bin/ls`，说明这个命令在这个目录下，我们 ls 一下 `/bin`：
```
ls /bin
# 结果：包含绿色的可执行文件
```
这些绿色的可执行文件其实有许多就是我们经常使用的命令，现在知道命令在哪里了吧。

#### ls 等命令是谁写得？
Linux 发展到现在，它上面有很多很常用的命令，这些**命令是一些大牛用 C 语言写的**，而且都是开源的，为了方便安装，业界将这些命令整理成一个软件包可供用户使用：[ Coreutils](https://directory.fsf.org/wiki/Coreutils)，该软件包里面就是一些比较常用的命令的源代码，后面我会带着大家一起编译它。


#### 系统如何查找这些命令？
我们在 Windows 下有环境变量 `Path` 这个名词，在 Linux 下也有这个概念。环境变量的作用其实可以用一句话概括：**告诉操作系统你要执行的程序在哪里**，我们先看看 Linux 下的环境变量配置文件：
```
vim /etc/environment

# 机器不同结果可能有些不同
PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
```
可以看到，环境变量其实就是一些**配置的目录**，操作系统在执行你在终端键入的程序时，会去这些配置的目录下查找要执行的程序，然后执行。我们可以看到 `/bin` 目录就在系统环境变量中，而 `ls` 这个程序就在 `/bin` 目录下，所以我们在终端键入 `ls` 可以出效果。因此，想要配置你自己的环境变量，更改这个文件即可，注意需要按照格式来添加目录。

#### 命令是如何执行的？
要解释这个概念，我们需要先了解下**什么是 Linux 的 Shell 解释器**？

`Shell` 的概念源自 `UNIX` 的命令解释器，它**解释用户输入的命令，然后调用内核相应的类库来执行相应的功能**。

常用的 Linux 下的解释器有，`bash`，`sh`，`csh`，`ksh`，在 `ubuntu` 下使用的是 `bash`，这里就以 `bash` 来作为解释器来介绍。

`bash` 其实也是一个程序，那么它在哪里呢？键入下面的命令：
```
whereis bash

# 机器不同结果可能有些不同
bash: /bin/bash /etc/bash.bashrc /usr/share/man/man1/bash.1.gz
```
可见，`bash` 也在 `/bin` 目录下。

了解了什么是 Shell 解释器， ls 命令的执行原理也就差不多清楚了：**当我们在终端键入 ls 命令，系统会去当前环境变量下查找 ls 这个命令对应的可执行文件 `/bin/ls`，然后由当前终端对应的解释器 bash 来解析 ls 这个命令，并由 bash 解释器调用内核功能来列出当前目录下的内容**。

后面也会带着大家一起**手动编译 `bash` 解释器**。


#### 写一个 myls 命令并自己调用
总是用别人写的东西多没挑战性，来试试自己写一个 `myls` 命令然后自己使用怎么样？输入下面的 `myls.c` 代码，功能如何实现的不是这里的重点，真正的 `ls.c` 的代码可没有这么简单，这里重点是知道这个过程。


```c
#include <stdio.h>  
#include <stdlib.h>    
#include <dirent.h>  

int main(int argc, char *argv[]) {  
	DIR *pd;      
	struct dirent *pdir;  

	if(argc != 2) {  
		printf("Usage: ls dir\n");  
		exit(1);  
	}  

	if(NULL == (pd = opendir(argv[1]))){  
		printf("Can't open %s\n", argv[1]);  
		exit(1);  
	}  

	while((pdir = readdir(pd)) != NULL)  
		printf("%s\n", pdir->d_name);  

	closedir(pd);  
	return 0;  
}  

```

编译：
```
gcc myls.c -o myls
```

测试：
```
# 列出当前目录的内容
./myls ./
```
安装到系统中，其实就是拷贝到系统环境变量的目录中去：
```
sudo cp myls.c /bin/myls
```
拷贝完成后，我们在终端直接执行 `myls ./`：
```
myls ./
# 列出了当前目录下的内容
```
如果你成功使用 `myls` 列出当前目录的内容，那么再好好回顾下整个过程，你应该已经有些领悟了吧，代码不是重点，重点是要理解这个过程。


## 学会类比
通过对 ls 命令调用机制的学习，相信你已经了解的差不过了，通过类比，你也应该清楚了 Linux 下的命令的实现机制，但是为了更进一步让你了解这些原理，**作为最佳实践，下面就带你一起来手动编译自己的 `bash` 和 `ls` 这些工具**。



## 编译 Bash 和 Coreutils
话说回来，既然这些工具都是大牛写出来的程序，那么是程序就应该能够编译安装啊，如何在系统中编译 `bash` 呢（但不安装）？下面就来告诉你方法。

>  我们不在自己的系统中再次安装 bash 等工具了，系统默认已经安装了。

**手动源码编译**的方法在我的这篇[如何在 Linux 上安装普通应用程序？](http://cheng-zhi.me/posts/linux-install-soft)文章中也有介绍，这里我们再来实践复习一遍。

#### 1. 下载 Bash 和 Coreutils
在这之前，我们再来回顾下 Linux Shell 的命令机制，它主要由下面两部分组成：
1. **`Coreutils` 提供具体的命令功能的实现**，就是一个个的 C 语言写的工具，例如：`ls.c`，你自己也可以写这些工具。
2. **`Bash` 等命令解释器来解释具体的命令**，例如：`ls` 命令，由 `Bash` 来解析 `Coreutils` 提供的 `ls.c` 的功能，**解析过程其实就是调用 Linux 系统底层类库来在屏幕上列出当前目录下的内容**。

所以，我们必须要有这两部分才可以正常访问内核提供的功能。我们到 `GNU` 的官网去下载这两个软件包：[下载 Bash](https://directory.fsf.org/wiki/Bash)，[下载 Coreutils](https://directory.fsf.org/wiki/Coreutils)，**点击右侧的 Download 链接即可下载软件包**，网速不好的同学，文末有百度云链接，我已经帮你们下载好了。

大家一定要善于利用 GNU 的官网：
```
www.gnu.org
```
如果你需要编译源码来安装程序，在这上面可以找到几乎所有的软件包，而且全部免费，这就是开源的力量！

#### 2. 配置，编译
两个软件包的配置编译过程几乎都是相同的。

**解压 Bash**
```
tar -xzvf bash-4.4.tar.gz
```

**配置 Bash**
```
cd bash-4.4/
./configure
```

**编译 Bash**
```
make
```

**测试 Bash**
```
./bash -version

# 出现版本信息，说明编译成功
GNU bash, version 4.4.0(1)-release (x86_64-unknown-linux-gnu)
Copyright (C) 2016 Free Software Foundation, Inc.
License GPLv3+: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>

This is free software; you are free to change and redistribute it.
There is NO WARRANTY, to the extent permitted by law.
```

**编译 Coreutils 的步骤只有第一步解压的方法不同，其他都是相同的**，解压 Coreutils 使用下面的命令：
```
xz -d coreutils-8.26.tar.xz
tar -xvf coreutils-8.26.tar
```

编译完 Coreutils 后，可以看看 src 目录下的内容：
```
cd coreutils-8.26/
ls src/
```

可以看到 `/src` 下都是常用命令的 C 语言实现文件和编译出来的可执行文件，例如：`ls.c`，`ls`。知道了这个原理，我们以后再使用命令的时候就知道它们的来源了，他们其实也是程序罢了，只不过经常使用就慢慢变成了标准，等你变成了大牛，你也可以制定标准。

#### 3. 如何安装

你如果只是为了了解原理，那么不建议你安装，你只需要成功编译，并了解安装的方法即可，如果你确实需要升级系统的 bash 和 Coreutils 的话，你可以安装。

安装方法有手动安装和自动安装 2 种，先介绍**手动安装**：

手动安装 bash ，只需要将 bash 可执行文件拷贝到系统目录下：
```
sudo cp bash-4.4/bash /bin/
```

手动安装 Coreutils，只需要将 `/coreutils-8.26/src` 下的可执行文件拷贝到系统目录下（这里以安装 ls 为例）：
```
sudo cp coreutils-8.26/src/ls /bin/
```

**如何自动安装：**

直接在 2 个目录中键入下面的命令，自动安装其实也只是帮你拷贝而已：
```
sudo make install
```

> 在 Linux 下安装软件的过程其实就是拷贝程序和库的过程。


这样一来，我们就成功编译了 bash 和 Coreutils 了，有没有感觉现在再使用 ls 这些命令，心里比以前有底了。


## 总结
本次主要介绍了 Linux 命令的实现机制，了解这个机制，以后再使用命令的时候就不会陌生了，希望看完一定要实践，谢谢阅读 :)


Bash-Coreutils: 链接 [https://pan.baidu.com/s/1pKY3HiV](https://pan.baidu.com/s/1pKY3HiV) 密码: h6x8




