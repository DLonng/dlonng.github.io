---
title: Mac 10.13 安装 Python-3.6.8 和 IPython-Notebook
date: 2020-03-31 11:30:00
---
# Mac 10.13 安装 Python-3.6.8 和 IPython-Notebook
***
> 版权声明：本文为 {{ site.name }} 原创文章，可以随意转载，但必须在明确位置注明出处！

机器学习的作业要求 Python-3.6 的环境，记录下配置 mac 的配置过程。

## 一、安装 Python-3.6.8

我用的 3.6.8 版本，你也可以安装其他版本，方法相同。

### 1.1 查看 Mac CPU 位数

```shell
uname -a

Darwin MacdeAir 17.7.0 Darwin Kernel Version 17.7.0: Thu Jun 21 22:53:14 PDT 2018; root:xnu-4570.71.2~1/RELEASE_X86_64 x86_64
```

x86_64 表示 64 位，否则是 32 位，后面需要下载对应位数的安装包。

### 1.2 查看当前默认 Python 环境

```shell
python

Python 2.7.10 (default, Oct  6 2017, 22:29:07)
[GCC 4.2.1 Compatible Apple LLVM 9.0.0 (clang-900.0.31)] on darwin
Type "help", "copyright", "credits" or "license" for more information.
>>> 
```

我的电脑默认安装了 Python-2.7，因为有些软件依赖 2.7 版本，所以不要卸载 2.7，我们直接安装 3.6.8 版本。

### 1.3 官网下载 Python-3.6.x

下载地址：[https://www.python.org/downloads/](https://www.python.org/downloads/)，选择合适的版本下载，我选择的是 3.6.8：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/download_py.png)

因为我的电脑是 64 位的，所以这里下载 macOS 64-bit installer：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/py_64_install.png)

下载完成后，双击安装包，一路继续就可以了：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/py_install.png)

### 1.4 查看是否安装成功

```shell
python3

Python 3.6.8 (v3.6.8:3c6b436a57, Dec 24 2018, 02:04:31)
[GCC 4.2.1 Compatible Apple LLVM 6.0 (clang-600.0.57)] on darwin
Type "help", "copyright", "credits" or "license" for more information.
>>> 
```

打印 3.6.8 版本信息，说明 Python 3 安装成功了！

## 二、安装 IPython-Notebook

因为科学计算的许多项目代码都是用 IPython-Notebook 发布的，所以这里也需要安装，用来打开 `ipynb`文件。

### 2.1 安装 Jupyter

一行命令即可安装，注意 Python3 用的是 pip3：

```shell
pip3 install --user jupyter
```

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/jupyter_install.png)

等待安装完毕即可。

### 2.2 打开 IPython-Notebook

在终端中用命令打开：

```shell
python3 -m IPython notebook
```

打开成功后，会自动打开浏览器并跳到软件界面：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/ipynb_open.png)

之后就可以写代码调试了 ^_^：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/debug_ipynb.png)

搞定！


> {{ site.prompt }}

<div  align="center">
<img src="https://dlonng.com/images/wechart.jpg" width = "200" height = "200"/>