---
title: Ubuntu Install oh-my-zsh
date: 2019-06-13 15:00:00
---
# Ubuntu Install oh-my-zsh
***
> 版权声明：本文为 {{ site.name }} 原创文章，可以随意转载，但必须在明确位置注明出处！

记录一下 ubuntu 16.04 安装 zsh 的过程。
### 1、Install zsh
先安装 zsh，如果提示已经安装就不用继续了：
```
sudo apt install zsh
```
查看安装版本 5.1.1（current）：
```
zsh --version
```
从 bash 切换到 zsh：
```
chsh -s $(which zsh)
```
Log out 重新登录，出现 zsh 的配置界面，按 2 保存即可，然后查看当前的 Shell 程序：
```
echo $SHELL
```
输出 `/bin/zsh` 就对了。
### 2、Install oh-my-zsh
`oh-my-zsh` 是一个 zsh 的配置项目，因为起初 zsh 的配置有点麻烦，所以就有大神做了这个项目，配置 zsh 变得特别方便。

先来安装 oh-my-zsh：
```
sudo wget https://github.com/robbyrussell/oh-my-zsh/raw/master/tools/install.sh -O - | sh
```
打开 oh-my-zsh 的配置文件：
```
vim ~/.zshrc
```
将主题修改为 `ys`，个人觉得这个好看：
```
11：ZSH_THEME="ys"
```
source 更新当前 shell 环境：
```
source ~/.zshrc
```
搞定了，这是我的 Shell 界面，还不错吧：

<div  align="center">
<img src="{{ site.url }}/images/ubuntu_config/my_zsh.png"/>
</div>

对了，如果之前用 bash 并且在 .bashrc 中加了系统环境的相关配置，比如代理，某些程序的环境变量啥的，切换到 zsh 后也要在 .zshrc 中进行相同的配置，否则某些程序可能启动不了哦，切记！

> {{ site.prompt }}

<div  align="center">
<img src="{{ site.url }}/images/wechart.jpg" width = "200" height = "200"/>