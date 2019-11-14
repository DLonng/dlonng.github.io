---
title: ROS 初级 - 安装和配置 ROS 环境
date: 2019-11-12 20:00:00
---
# ROS 初级 - 安装和配置 ROS 环境
***
> 版权声明：本文为 {{ site.name }} 原创文章，可以随意转载，但必须在明确位置注明出处！

这个 ROS 系列教程主要来自 [ROS Wiki](http://wiki.ros.org/)，加上自己学习过程中踩过的一些坑和个人理解。

## 1、安装 ROS
在配置 ROS 环境前要完全安装 ROS，参看之前的安装 ROS 的文章：[Ubuntu 16.04 安装 Kinetic](https://dlonng.com/posts/install-kinetic)。

## 2、管理 ROS 环境变量
学习 ROS 环境变量的目的是为了更好的管理 ROS 项目，不同版本的 ROS 安装完都会提供一个 setup.*sh 文件，当我们使用 source 命令执行某个分支的 setup.*sh，即可切换到对应版本的 ROS 环境下。

要查看当前的 ROS 相关的环境变量，执行下面的命令：
```
printenv | grep ROS
```

<div  align="center">
<img src="https://dlonng.com/images/ros/config_envir/ros_env.png"/>
</div>


> 补充：这个命令的意思是把 printenv 的输出通过 Linux 下的进程间通信方式管道 | 来作为 grep 命令的输入，而 grep ROS 的作用是从输入中查找含有 ROS 关键字的行。

注意查看 ROS_ROOT 和 ROS_PACKAGE_PATH 的路径是否是你安装的 ROS 版本。

在我们安装完 K 版本的 ROS 后，需要在终端 source 一下对应的 setup.*sh 脚本：
```
source /opt/ros/kinetic/setup.bash
```
我使用的是 zsh：
```
source /opt/ros/kinetic/setup.zsh
```
建议你体验下 zsh，极度舒适：[Ubuntu install oh-my-zsh](https://dlonng.com/posts/install-zsh)

执行完命令后就可以使用 ROS 提供的功能了，但是当我们新开一个终端后，又要重新执行上面的命令，非常麻烦，怎么办呢？

其实非常简单，我们可以将上面的命令加到终端的启动脚本末尾，例如 bash 对应 ~/.bashrc，zsh 对应 ~/.zshrc。

> 补充：使用 echo $SHELL 查看你当前使用的是什么 shell。

然后用 vim 或者你喜欢的编辑器打开你终端的启动脚本，把上面的命令加到文件末尾就行了：
```
“ bash
source /opt/ros/kinetic/setup.bash
```

```
” zsh
source /opt/ros/kinetic/setup.zsh
```

## 3、创建 ROS 工作空间
使用下面的命令来创建一个 ROS 空间：

```
# 在用户主目录创建目录 catkin_ws/src
mkdir -p ~/catkin_ws/src
# 进入 ros 工作目录
cd ~/catkin_ws
# make 一下
catkin_make
```

执行完后，你会在当前目录下发现 build，devel 文件夹以及 src 目录下多了一个 CMakeLists.txt 文件。

在 devel 目录下，也会生成 setup.zsh(bash) 文件，我们 source devel 下的 setup.zsh(bash) 文件会覆盖之前我们在终端脚本中配置的 source 环境。

但这一步是必要的，因为以后我们编译自己的 ros 包时，需要使用当前工作目录的环境，因此需要执行这一步：

```
source devel/setup.zsh(bash)
```

我们可以再查看下环境变量：
```
echo $ROS_PACKAGE_PATH
/home/dlonng/catkin_ws/src:/opt/ros/kinetic/share
```
可以看到当前工作目录一已经被加入到 ROS 环境中了。至此，ROS 环境配置完成，下一篇开始学习 ROS 的基本命令。



> {{ site.prompt }}

<div  align="center">
<img src="https://dlonng.com/images/wechart.jpg" width = "200" height = "200"/>