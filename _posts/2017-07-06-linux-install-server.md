---
title: 如何在 Linux 上安装服务器程序？ 
date : 2017-07-06 12:00:00
---

# 如何在 Linux 上安装服务器程序？ 
***
> 版权声明：本文为 cheng-zhi 原创文章，可以随意转载，但必须在明确位置注明出处！ 

## 前言
之前的一篇文章 [如何在 Linux 上安装普通应用程序？](http://cheng-zhi.me/posts/linux-install-soft) 介绍在 `Linux` 下安装应用程序的方法，主要针对普通应用程序，而在 `Linux` 上安装服务器程序，不仅需要我们会安装，而且还要求我们会配置服务。因此，本次就带大家了解服务器程序的安装和配置的基本方法。

在此之前，我们先来了解一些与服务相关的知识以帮助我们理解 `Linux` 的服务的运行原理。

## Linux 运行级别
`Linux` 系统有下面 7 个运行级别：
1. 系统停机模式 [0]：系统默认运行级别不能设置为 0，否则不能正常启动
2. 单用户模式 [1]：具有 root 权限，用于系统维护，禁止远程登录，类似 Windows 下的安全模式
3. 多用户模式 [2]：没有 NFS 网络支持，启动 XWindows
4. 完整的多用户文本模式 [3]：有 NFS，登录后进入控制台命令行模式
5. 系统未使用 [4]：保留一般不用，在一些特殊情况可以用来做一些事情，例如笔记本电脑的电池用尽时，可以切换到这个模式来做一些设置
6. 图形化模式 [5]：登录后进入 GUI 模式，启动 XWindows 系统
7. 重启模式 [6]：默认级别不能设置为 6，否则机器重复启动，运行 `sudo init 6` 命令机器就会重启

在命令行键入下面的命令查看当前运行级别：
```
runlevel
# 显示当前运行级别是 5，即图形化模式
N 5
```

键入下面的命令来更改当前运行级别：
```
sudo init [0 - 6]
```
为什么要了解运行级别呢？因为每个运行级别都有自己默认启动的服务，如果我们想让自己的服务程序自动启动，只要将可执行文件或者指向它的链接放到指定的目录下即可。

## 每个运行级别默认启动的服务
`Linux` 系统的每个运行级别默认启动的服务在 `/etc/rc$runlevel.d/` 目录下。例如在图形界面下，系统的运行级别为 5，那么 `$runlevel = 5`，前面的路径即为：`/etc/rc5.d/`，我们查看这个目录的内容：
```
ll /etc/rc5.d/
# 结果
lrwxrwxrwx   1 root root    15 Oct 27  2016 S01acpid -> ../init.d/acpid*
lrwxrwxrwx   1 root root    17 Oct 27  2016 S01anacron -> ../init.d/anacron*
lrwxrwxrwx   1 root root    16 Oct 27  2016 K01apport -> ../init.d/apport*
..
```

> `rc` 是 `run command` 的缩写。

可以发现在这个目录下都是以 S 和 K 开头的指向 `../init.d/` 目录下程序的软链接，这些链接指向的服务程序就是该启动级别下默认需要启动和关闭的服务， **S(tart) 开头的代表该服务需要启动，而 K(ill) 开头的代表不启动该服务，序号表示启动顺序，序号小先启动**。毫无疑问，`/etc/init.d/` 目录下存放的就是所有的服务器程序，这些程序称之为**守护进程**。

那么当我们自己写的一个**守护进程程序**想让其开机自动启动，该如何做呢？其实很简单，只需要下面 3 步：
1. 将程序的可执行文件 `you_app` 拷贝到 `/etc/init.d/you_app`
2. 在你需要的运行级别目录下加上程序的软链接，例如在 `/etc/rc5.d/` 下增加 `you_app` 的软链接，并更换程序的名称为 `S[num]you_app`，例如 `S02you_app`。这样就表示在该运行级别下，自动启动该程序。
3. 运行 `update-rc.d` 命令来更新系统设置，该命令在 `/usr/sbin/update-rc.d`

另外，下面这 4 个目录你需要了解：
1. `/etc/`：存放系统和程序相关的配置文件
2. `/etc/init/`：存放系统启动相关的配置文件
3. `/etc/init.d/`：存放系统守护进程
4. `/etc/rc[0 - 6].d`：存放每个运行级别需要启动和关闭的服务

基本原理了解这些即可，下面开始介绍服务器程序的安装思路。我为何总是強調原理，因为我想让你看完我的文章能够学会如何主动去思考，主动提出问题，主动去解决，因为人的进步 90% 都是要靠自己，总是依靠别人会吃亏的。

## 服务器程序安装思路
前面介绍的服务器程序也称为守护进程，例如 `Linux` 系统大名鼎鼎的 `init` 守护进程，http, httpd, ftp，等等。这些非系统自带的服务器程序的安装方法大多数都可以使用 `apt-get` 来安装，只是在安装完成之后，我们还需要手动来进行配置，毕竟是服务器程序，它需要知道你想让它如何工作。

配置之前我们需要知道服务器程序的配置文件在何处，前面提到 `/etc/` 存放的都是配置文件，那么如何查看我们安装的服务器程序的配置文件的具体位置呢？我们再一次想到了使用 `man`。比如，我们想知道 `ssh` 服务的配置文件在何处，我们直接 `man ssh`，然后定位到 `FILES` 字段，该字段下面就是该服务的相关文件的路径，可以看到下面这一行：
```
FILES
	...
	/etc/ssh/ssh_config
		Systemwide configuration file.  The file format and configurationoptions are described in ssh_config(5).
	...
```
这直接列出了这个文件的路径和功能，这个文件就是 `ssh` 的系统配置文件，使用 `man` 基本上可以解决大多数问题，如果使用 `man` 不行，那么 Google 和 stackoverflow 也可以帮助你，至于 Baidu 就不要使用了。

其实服务器程序的路径都差不多，在你熟悉一个服务器程序的配置之后，就知道一般的服务器程序都是安装在 `/bin/` 或 `/sbin/` 下的，我们可以在 `/etc/init.d/` 下找到这些程序的映射。

至于配置文件，基本都可以在 `/etc/` 下或者 `/etc/service_name` 下找到类似于 `service_name.conf` 的配置文件名，例如前面的 `/etc/ssh/ssh_config` 配置文件就符合这个特征，其他的服务也基本是一样的。


## 操作服务的命令
我们在安装完服务后，需要启动或者关闭服务，这是可以使用下面的 2 种方法：

#### 方法一
```
cd /etc/init.d/
service_name start/stop/restart/status
```

#### 方法二
```
service service_name start/stop/restart/status
```

第二种方法是对第一种方法的封装，通常使用第二种方法比较简单，如果其中一种不能正确启动，换另一种即可。


下面以一个安装 `Samba` 服务器为例来练习安装服务器程序。

## 练习：安装 Samba 服务器
`Samba` 是可以让 `Windows` 通过网络来访问 `Linux` 文件的服务程序，学会使用它可以方便共享 `Windows` 和 `Linux` 的文件。我使用的是 `ubuntu` 来安装，其他的 `Linux` 或多或少有些区别，这里我只能告诉你安装的基本方法，不可能所有的问题都列出来，你要学会使用 Google 来搜索，这样才可以锻炼自己解决问题的能力。

#### 卸载 iptables
在 `Ubuntu` 下先卸载 `iptables` 防火墙：
```
sudo apt-get remove iptables
```

#### 安装 samba
键入下面的命令来安装 `samba`：
```
sudo apt-get install samba
```

#### 配置 samba
查看 `samba` 的配置文件：
```
sudo vim /etc/samba/smb.conf
```

这个文件是 `samba` 的配置文件，这里可以自定义客户端访问 `samba` 的一些属性，例如你可以加入下面的 root 用户访问的属性：
```
# 字段的含义这里就不介绍了，自行 Google
[root]
	comment = root dir
	browseable = yes
	path = /
	writeable = yes
	valid users = root
```

然后为 root 用户设置访问 `samba` 的密码：
```
smbpasswd -a root
```

输入完后，即可启动服务，如果方法二启动失败，就使用方法一来启动：
```
/etc/init.d/samba start
```
提示你输入密码，输入即可，然后查看状态：
```
/etc/init.d/samba status
```
下面的圆圈显示为绿色表示启动成功，否者启动失败：
```
● smbd.service - Samba SMB Daemon
	Loaded: loaded (/lib/systemd/system/smbd.service; enabled; vendor preset: enabled)
	Active: active (running) since Wed 2017-07-05 10:29:08 CST; 27min ago
```

重启使用下面的命令：
```
/etc/init.d/samba restart
```

#### 配置 Windows
要想 `Windows` 能够访问 `Linux` 的服务器，必须要保证下面 2 个条件：
1. 两者连接同一个网络，例如连接同一个 wifi 或者直接用网线相连接或者连接同一个交换机
2. 两者可以相互 `ping` 通，这个是关键

一般情况下，如果能够相互 `ping` 通，则访问成功的概率比较大，`ping` 的使用方法如下：

首先查看两者的 ip：
```
# 在 Windows CMD 下键入下面的命令来查看 Windows ip
ipconfig
# 例如：192.168.1.1

# 在 Linux 下键入下面的命令来查看 Linux ip
ifconfig
# 例如：192.168.1.2
```

然后分别在 2 台机器上 ping 对方的 ip，观察是否有回复：
```
# Windows
ping 192.168.1.2
# 观察是否有回复

# Linux
ping 192.168.1.1
# 观察是否有回复
```

如果都有回复，表示可以相互访问，连接成功概率较大，否则你可以尝试关闭 `Windows` 和 `Linux` 的防火墙。


#### 连接 Linux
如果能够 ping 通，你就可以在 `Windows` 下通过浏览器来访问 `Linux` 了，还可以将 `Samba` 映射为 `Windows` 下的网络驱动器，方便以后的使用。


## 总结
本文主要介绍了在 `Linux` 上安装服务器程序的思路和方法，并给出了安装 `samba` 服务器的例子作为实践。此外，我建议你自己去尝试安装其他的一些服务，例如，ftp，tftp，NFS，等等，具体安装过程遇到的问题还需要你自己到 Google 和 stackoverflow 上去寻找答案，一般你遇到的问题别人都已经遇到了，就看你有没有能力找到那个答案了，祝你学习愉快 :)

