---
title: Ubuntu 16.04 安装 vim 分屏工具 Terminator
date: 2019-11-19 12:00:00
---
# Ubuntu 16.04 安装 vim 分屏工具 Terminator
***
> 版权声明：本文为 {{ site.name }} 原创文章，可以随意转载，但必须在明确位置注明出处！

默认的 vim 分屏比较鸡肋，所以在晚上找了一个分屏工具 Terminator，记录下配置过程，这是安装后的界面，还不错：

<div  align="center">
<img src="https://dlonng.com/images/terminator/termin.png"/>
</div>

### 1、安装 Terminator
```
sudo apt-get install terminator
```
### 2、配置界面
默认的界面比较丑，右击终端打开 preference 配置一下终端的 color，background 等就完美了。



最终的效果还跟主题有关，所以按照自己喜欢的方式设置就行了，下面是我的设置，可以参考。

#### general
<div  align="center">
<img src="https://dlonng.com/images/terminator/general.png"/>
</div>

#### colors
<div  align="center">
<img src="https://dlonng.com/images/terminator/colors.png"/>
</div>

#### scrollbar
<div  align="center">
<img src="https://dlonng.com/images/terminator/scrollbar.png"/>
</div>

#### background
<div  align="center">
<img src="https://dlonng.com/images/terminator/bkg.png"/>
</div>

开启你的 vim IDE 之旅吧！



### 3、常用分屏快捷键

- Ctrl + Shift + E：垂直分割窗口
- Ctrl + Shift + O：水平分割窗口
- Ctrl + Tab：分割窗口之间切换
- Ctrl + Shift + C：在 Shell 中复制
- Ctrl + Shift + V：在 Shell 中粘贴
- Ctrl + Shift + W：退出当前窗口
- Ctrl + Shift + Q：退出当前所有窗口，终端将被关闭
- Alt + Up：移动到上面的终端
- Alt + Down ：移动到下面的终端
- Alt + Left ：移动到左边的终端
- Alt + Right：移动到右边的终端
- Ctrl + Shift + Right：在垂直分割的终端中将分割条向右移动
- Ctrl + Shift + Left：在垂直分割的终端中将分割条向左移动
- Ctrl + Shift + Up：在水平分割的终端中将分割条向上移动
- Ctrl + Shift + Down：在水平分割的终端中将分割条向下移动
- Ctrl + Super（Win）+ 方向上：最大化窗口
- Ctrl + Super（Win）+ 方向下：最小化窗口
- Ctrl + Super（Win）+ 方向左：把 Shell 窗口移动到屏幕左半边
- Ctrl + Super（Win）+ 方向右：把 Shell 窗口移动到屏幕右半边



> {{ site.prompt }}

<div  align="center">
<img src="https://dlonng.com/images/wechart.jpg" width = "200" height = "200"/>