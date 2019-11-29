---
title: ROS 入门 - 服务和参数
date: 2019-11-25 20:00:00
---
# ROS 入门 - 服务和参数
***
> 版权声明：本文为 {{ site.name }} 原创文章，可以随意转载，但必须在明确位置注明出处！



## 1、ROS Services

ROS 服务是 ROS 提供的一种节点之间相互通信的方式，一个服务允许节点发送一个请求 request 或者接收一个响应 response。



##  2、rosservice

rosservice 命令可以对服务进行操作，比如调用服务，显示服务类型等，如下：

```sh
rosservice list print information about active services
rosservice call call the service with the provided args
rosservice type print service type
rosservice find find services by service type
rosservice uri print service ROSRPC uri
```

### 2.1 rosservice list

list 命令能够列出当前运行的节点提供的所有服务，让我们先来运行上一篇博客的小乌龟节点，先运行 roscore：

```sh
roscore
```

再开启一个新终端启动小乌龟节点，也可以在一个终端中分屏，参考[这篇](https://dlonng.com/posts/terminator)文章：

```sh
rosrun turtlesim turtlesim_node
```

然后使用 list 命令查看当前运行的小乌龟节点提供的服务：

```sh
rosservice list

/clear
/kill
/reset
/rosout/get_loggers
/rosout/set_logger_level
/spawn
/teleop_turtle/get_loggers
/teleop_turtle/set_logger_level
/turtle1/set_pen
/turtle1/teleport_absolute
/turtle1/teleport_relative
/turtlesim/get_loggers
/turtlesim/set_logger_level
```

你的输出应该跟上面类似，再来详细看看一个服务的类型。

### 2.2 rosservice type

type 命令显示服务的类型，即这个服务发送和接收请求的数据类型：

```sh
rosservice type [service]
```

来看看 clear 服务的 type：

```sh
rosservice type /clear

std_srvs/Empty
```

输出显示这个 clear 服务的类型为空，为什么呢？这是因为 clear 服务是清除功能，调用这个服务不需要传递参数，自然也就不需要指定服务的 type 了。

那如何来调用服务呢？

### 2.3 rosservice call

Call 命令使用方法如下：

```sh
rosservice call [service] [args]
```

来调用 clear 服务试试，这个服务不需要传递参数：

```sh
rosservice call /clear
```

可以发现小乌龟窗口背景中的白色路劲线被清除了，恢复成刚运行的样子：

<div  align="center">
<img src="http://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams?action=AttachFile&do=get&target=turtlesim.png"/>
</div>

那🈶参数的服务如何调用呢？再来看看 spawn 服务的参数，这个服务可以在指定的位置和方向产生一个新的小乌龟：

```sh
rosservice type /spawn | rossrv show

float32 x
float32 y
float32 theta
string name
---
string name
```

来调用这个服务试试吧：

```sh
rosservice call /spawn 2 2 0.2 ""
```

上面的调用命令中的 2 2 0.2 "" 分别对应 spawn 服务的 4 个参数，其中 name 没有指定，但系统会自动赋值一个名称，运行的结果如下，产生一个新的小乌龟：

<div  align="center">
<img src="http://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams?action=AttachFile&do=get&target=turtle%28service%29.png"/>
</div>

并且运行完 spawn 服务后，命令行会返回新产生小乌龟的名称，我们调用的时候没有指定名称 name，所以系统就自己取了名字：

```sh
name：tutle2
```



## 3、rosparam

rosparam 命令允许我们在 ROS 参数服务器上存储和操作数据，数据类型包括：整数，浮点，字符串，布尔，字典，列表，rosparam 使用 YAML 标记语言来作为数据的语法，如下：

- 1：整数
- 1.0：浮点
- one：字符串
- true：布尔
- {a:b, c:d}：字典
- [1, 2, 3]：列表

rosparam 常用的命令如下：

```sh
rosparam set            set parameter
rosparam get            get parameter
rosparam load           load parameters from file
rosparam dump           dump parameters to file
rosparam delete         delete parameter
rosparam list           list parameter names
```

下面就用这些命令来看看小乌龟有哪些参数。

### 3.1 rosparam list

保持之前的小乌龟节点运行，然后在命令行键入 list 命令：

```sh
rosparam list
```

输出以下参数，其中前 3 个是小乌龟窗口的背景颜色参数：

```sh
/background_b
/background_g
/background_r
/rosdistro
/roslaunch/uris/host_57aea0986fef__34309
/rosversion
/run_id
```

我们可以使用 set 和 get 命令来修改和获取对应参数的值。

### 3.2 rosparam set & get

我们先来改变以下背景颜色中 red 通道的颜色值：

```sh
# using: rosparm set [param_name] [param_value]
rosparam set /background_r 150
```

修改了参数后，我们需要重新调用 clear 服务：

```sh
rosservice call /clear
```

可以看到小乌龟窗口的背景颜色改变了：

<div  align="center">
<img src="http://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams?action=AttachFile&do=get&target=turtle%28param%29.png"/>
</div>







<div  align="center">
<img src="https://dlonng.com/images/xxx/xxx.png"/>
</div>

> {{ site.prompt }}

<div  align="center">
<img src="https://dlonng.com/images/wechart.jpg" width = "200" height = "200"/>