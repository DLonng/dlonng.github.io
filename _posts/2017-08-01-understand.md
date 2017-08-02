---
title: 免费跨平台源码阅读工具 understand 
date: 2017-08-01 10:00:00
---

# 免费跨平台源码阅读工具 understand 

***
> 版权声明：本文为 {{ site.name }} 原创文章，可以随意转载，但必须在明确位置注明出处！ 

最近在 `ubuntu` 下需要阅读源代码，之前在 `Windows` 下有 [Sourch Insight](https://www.sourceinsight.com/) 可以用，但是 SI 不能跨平台。如果要在 Linux 下用还需要使用 wine，可是碰巧我的 wine 有点问题，启动不了 SI，有点闹心。

对于阅读源码我还是喜欢用图形界面，虽然 `vim + ctags + cscope` 也可以，但是我个人倾向与图形界面来看，于是在网上搜索了一会，找到一个跨平台的源码阅读工具 [understand](https://scitools.com/)，但是需要付费，不过还好国人强大......

> 如果你在工作，我还是建议购买，毕竟同为程序员，互相尊重吗。如果你是学生，破解即可，因为你是为了学习。

废话不多说，来正式安装。

## 1. 下载
我的电脑是 64 位的，百度云：[understand 64 bit](https://pan.baidu.com/s/1i52nrut)。你也可以到[官网](https://scitools.com/)下载 32 bit 和 64 bit，后面的破解都是可用的。

## 2. 安装，破解
我解压到用户主目录下，下面是解压的文件：
```
bin  conf  doc  sample  scripts  src
```

其中 `bin/linux64/understand` 是可执行文件，我们先 `./understand` 启动，以此进行下面 3 步：
1. 弹出对话框，点击中间的 `Add Permanent License` 按钮
2. 又弹出对话框，点击第一个 `Add Eval or SDL (RegCode)` 按钮添加密钥
3. 输入密钥：`09E58CD1FB79`，大功告成


![understand]({{ site.url }}/images/understand.png)


就是这么简单，开始你的源码阅读之旅吧 :)

