---
title: ROS 机器人技术 - rosbag 详细使用教程
date: 2020-06-28 23:00:00
---
# ROS 机器人技术 - rosbag 详细使用教程
***
> 版权声明：本文为 {{ site.name }} 原创文章，可以随意转载，但必须在明确位置注明出处！

在 ROS 系统中，可以使用 bag 文件来保存和恢复系统的运行状态，比如录制雷达和相机话题的 bag 包，然后回放用来进行联合外参标定。

这里记录下我学习官方的 rosbag 教程的笔记：[ROS rosbag](http://wiki.ros.org/rosbag/Commandline)

## 我常用的几个操作

虽然命令很多，但是我目前在工作中常用的命令就如下几个：

### 1. 录包

录制所有话题：

```shell
rosbag record -a
```

录制指定话题，设置 bag 包名：

```shell
rosbag record -O bag_name.bag /topic1_name /topic2_name /xxx
```

有时候我录制包不设置名称，默认按照录制结束时间命名：

```shell
rosbag record /topic1_name /topic2_name /xxx
```

### 2. 回放

我常用的是以暂停的方式启动，防止跑掉数据：

```shell
rosbag play --pause record.bag
```

你也可以直接回放：

```shell
rosbag play record.bag
```

我建图过程中有时会设置以 0.5 倍速回放，也就是以录制频率的一半回放：

```shell
rosbag play -r 0.5 record.bag
```

回放一个包中的指定主题：

```shell
rosbag play record.bag --topics /topic1 /topic2
```

回放完后，我一般会用 `rostopic list` 查看下发布的主题，确保是我需要的：

```shell
rostopic list
```

### 3. 修复

之前在我们小车的 Intel NUC 上录包拷到台式机上回放会报错，提示需要 reindex，执行一下即可，不过数据好像会少一些：

```shell
rosbag reindex xxx.bag
```

下面是完整的 rosbag 用法，需要的可以查找下。

## 一、rosbag 基本作用

rosbag 工具可以录制一个包、从一个或多个包中重新发布消息、查看一个包的基本信息、检查一个包的消息定义，基于 Python 表达式过滤一个包的消息，压缩和解压缩一个包以及重建一个包的索引。

rosbag 目前常用的命令如下（fix 和 filter 暂时没有用到）：

- `record`：用指定的话题录制一个 bag 包
- `info`：显示一个 bag 包的基本信息，比如包含哪些话题
- `play`：回放一个或者多个 bag 包
- `check`：检查一个 bag 包在当前的系统中是否可以回放和迁移
- `compress`：压缩一个或多个 bag 包
- `decompress`：解压缩一个或多个 bag 包
- `reindex`：重新索引一个或多个损坏 bag 包

我在目前项目的使用中，用的最多的就是 record、info 以及 play 功能，先录制想要的话题包，录制完毕检查下包的信息，最后再回放作为算法的输入，下面就详细学习下上面命令的用法。

## 二、rosbag record

使用 record 订阅指定主题并生成一个 bag 文件，其中包含有关这些主题的所有消息，常见用法如下。

用指定的话题 topic_names 来录制 bag 包：

```shell
rosbag record <topic_names>
```

比如录制 `rosout`、`tf`、`cmd_vel` 3 个话题，录制的 bag 包以时间命名：

```shell
rosbag recoed rosout tf cmd_vel
```

录制完保存 bag 包名称为 `session1 + 时间戳.bag` 格式：

```shell
rosbag record -o session1 /chatter
```

录制完保存为指定文件名 `session2_090210.bag`：

```shell
rosbag record -O session2_090210.bag /chatter
```

录制系统中所有的话题：

```shell
rosbag record -a
```

使用 `-h` 查看 record 使用方法，很多命令都可以用这个：

```shell
rosbag record -h
```

## 三、rosbag info

rosbag info 显示包文件内容的可读摘要，包括开始和结束时间，主题及其类型，消息计数、频率以及压缩统计信息，常见用法如下：

显示一个 bag 包的信息：

```shell
rosbag info name.bag
```

```shell
$ rosbag info foo.bag
path:        foo.bag
version:     2.0
duration:    1.2s
start:       Jun 17 2010 14:24:58.83 (1276809898.83)
end:         Jun 17 2010 14:25:00.01 (1276809900.01)
size:        14.2 KB
messages:    119
compression: none [1/1 chunks]
types:       geometry_msgs/Point [4a842b65f413084dc2b10fb484ea7f17]
topics:      /points   119 msgs @ 100.0 Hz : geometry_msgs/Point
```

查看常用命令：

```shell
rosbag info -h
```

输出 [YAML](https://yaml.org/) 格式的信息：

```shell
rosbag info -y name.bag
```

输出 bag 中指定域的信息，比如只显示持续时间：

```shell
rosbag info -y -k duration name.bag
```

## 四、rosbag play

rosbag play 读取一个或多个 bag 文件的内容，并以时间同步的方式回放，时间同步基于接收消息的全局时间。回放开始后，会根据相对偏移时间发布消息。

如果同时回放两个单独的 bag 文件，则根据时间戳的间隔来播放。比如我先录制一个 bag1 包，等待一个小时，然后录制另一个 bag2 包，那我在一起回放 bag1 和 bag2 时，在回放的中间会有 1 个小时的停滞期，也就是先回放 bag1，然后需要等待 1 个小时才能回放 bag2。

在回放过程中按**空格暂停**，常见用法如下，回放单个 bag：

```shell
rosbag play record.bag
```

回放多个 bag，基于全局时间间隔播放：

```shell
rosbag play record1.bag record2.bag
```

开始播放立刻暂停，按空格继续：

```shell
rosbag play --pause record.bag
```

以录制的一半频率回放：

```shell
rosbag play -r 0.5 --pause record.bag
```

指定回放频率，默认 100HZ：

```shell
rosbag play --clock --hz=200 record.bag
```

循环播放：

```shell
rosbag play -l record.bag
```

## 五、rosbag check

检查一个 bag 在当前系统中是否可以回放：

```shell
rosbag check xxx.bag
```

## 六、rosbag compress

如果录制的 bag 很大，我们可以压缩它，默认的压缩格式是 bz2：

```shell
rosbag compress xxx.bag
```

你也可以添加 `-j` 手动指定压缩格式为 bz2：

```shell
rosbag compress -j xxx.bag
```

也可以使用 LZ4 来压缩数据：

```shell
rosbag compress --lz4 xxx.bag
```

## 七、rosbag decompress

压缩完后，使用需要解压缩：

```shell
rosbag decompress xxx.bag
```

## 八、rosbag reindex

如果回放遇到问题，提示 reindex 的话，直接执行即可，这个会自动生成一个原 bag 的备份：

```shell
rosbag reindex xxx.bag
```

> {{ site.prompt }}

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/dlonng_qrcode.jpg#pic_center)