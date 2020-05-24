---
title: ROS 机器人技术 - 解析 roslaunch 文件
date: 2020-02-15 18:00:00
---
# ROS 机器人技术 - 解析 roslaunch 文件
***
> 版权声明：本文为 {{ site.name }} 原创文章，可以随意转载，但必须在明确位置注明出处！

## 1. roslaunch 命令

roslaunch 命令允许我们一次启动 launch 文件中定义的多个 ROS 节点，启动参数等在启动文件（launch 文件）中配置，并且如果系统之前没有启动 roscore，则 roslaunch 会自动启动它。

roslaunch 的命令行用法如下：

```sh
roslaunch [package] [filename.launch]
```

例如：

```sh
roslaunch beginner_tutorials turtlemimic.launch
```

其中 beginner_tutorials 是 turtlemimic.launch 文件中定义的一个节点，那 launch 文件是什么呢？我们来看下。

## 2. 解析 launch 文件

简单来说 launch 文件就是一堆节点和参数的集合，使用 launch 文件的目的是为了一键启动。因为目前运行在机器人上的 ROS 系统一般都要同时运行多个节点，如果我们开机启动时，一个一个的输入命令来启动每一个节点，想想就可怕，所以 ROS 系统就给我们提供了这么一个 launch 文件，可以方便我们将要启动的节点和对应的参数全部写到这个文件中，然后使用 roslaunch 命令一键启动！

一个 lanunch 文件例子：

```xml
<launch>
  <!-- local machine already has a definition by default.
       This tag overrides the default definition with
       specific ROS_ROOT and ROS_PACKAGE_PATH values -->
  <machine name="local_alt" address="localhost" default="true" ros-root="/u/user/ros/ros/" ros-package-path="/u/user/ros/ros-pkg" />
  <!-- a basic listener node -->
  <node name="listener-1" pkg="rospy_tutorials" type="listener" />
  <!-- pass args to the listener node -->
  <node name="listener-2" pkg="rospy_tutorials" type="listener" args="-foo arg2" />
  <!-- a respawn-able listener node -->
  <node name="listener-3" pkg="rospy_tutorials" type="listener" respawn="true" />
  <!-- start listener node in the 'wg1' namespace -->
  <node ns="wg1" name="listener-wg1" pkg="rospy_tutorials" type="listener" respawn="true" />
  <!-- start a group of nodes in the 'wg2' namespace -->
  <group ns="wg2">
    <!-- remap applies to all future statements in this scope. -->
    <remap from="chatter" to="hello"/>
    <node pkg="rospy_tutorials" type="listener" name="listener" args="--test" respawn="true" />
    <node pkg="rospy_tutorials" type="talker" name="talker">
      <!-- set a private parameter for the node -->
      <param name="talker_1_param" value="a value" />
      <!-- nodes can have their own remap args -->
      <remap from="chatter" to="hello-1"/>
      <!-- you can set environment variables for a node -->
      <env name="ENV_EXAMPLE" value="some value" />
    </node>
  </group>
</launch>
```

下面是一个简单的 launch 文件格式，我带你一步一步解析它的标签用法：

```xml
<launch>
    <node .../>
    <rosparam .../>
    <param .../>
    <include .../>
    <env .../>
    <remap .../>
    <arg .../>
    <group>  </group>
</launch>
```

### 2.1 launch

launch 标签是 launch 文件的根节点，它只作为其他子标签的容器，没有其他功能。

```xml
<launch>
	...
</launch>
```

### 2.2 node

node 标签指定一个将要运行的 ROS 节点，node 标签可以说是 launch 文件中最重要的标签，因为我们配置 launch 文件的目的就是一次启动多个 ROS node。

常见的用法如下：

```xml
<node name="bar1" pkg="foo_pkg" type="bar" args="--test" respawn="true" output="sceen">
```

- name：节点名称 bar1
-  pkg：节点所在的包 foo_pkg
-  type：节点类型是可执行文件（节点）的名称，项目中必须要有一个相同名称的可执行节点
-  args：命令行启动参数 --test
-  respawn：是否自动重启，true 表示如何节点未启动，则自动重启，false 则不重启，默认 false
- output：是否将节点信息输出到屏幕，如果不设置该属性，则节点信息会被写入到日志文件

在 node 标签下页可以嵌套使用以下标签：

- env：为节点设置环境变量
- remap：为节点设置重映射参数
- rosparam：为节点加载 rosparam 文件
- param：为节点设置参数

### 2.3 param

在项目中某些参数需要经常改变，如果在程序中写死了，以后我们每次修改参数都需要重新 build 一遍程序，非常麻烦，param 便签给我们提供了一个传递参数的方法。

param 标签定义一个将要被设置到参数服务器的参数，它的参数值可以通过文本文件、二进制文件或命令等属性来设置，另外 param 标签可以嵌入到 node 标签中，以此来作为该 node 的私有参数。

常见用法如下：

```xml
<param name="publish_frequency" type="double" value="10.0">
```

- name：参数名称 publish_frequency
- type：参数类型 double，str，int，bool，yaml
- value：参数值 10.0

param 标签也可以为一组 group 节点同时设置参数。

### 2.4 remap

remap 标签提供了一种节点名称的重映射方法，每个 remap 标签包含一个元素名称和一个新名称，在系统运行后原始名称会被替换为新名称。

常见用法如下： 

```xml
<remap from="chatter" to="hello">
```

- from：原始名称
- to：新名称

可以这样理解这个替换标签：你有一个节点订阅了「chatter」主题，但是你只有一个节点发布「hello」主题，而「hello」和「chatter」的类型相同，所以我们可以将「chatter」简单地替换为「hello」，从而实现订阅「hello」主题。

### 2.5 machine

machine 标签定义了节点所运行的机器信息，如果只是在本地运行节点则不需要配置这个标签，它主要使用在 SSH 和远程机器，不过也可以用来配置本地机器的相关信息。

常见用法如下：

```xml
<launch>
	<machine name="foo" address="foo-address" env-loader="/opt/ros/kinetic/env.sh" user="someone">
	<node machine="foo" name="footalker" pkg="test_ros" type="talker.py">
</launch>
```

- name：机器名称
- address：机器的网络地址
- env-loader：设置机器的环境变量，必须是一个设置了所有要求变量的 shell 脚本
- user：用户名称

### 2.6 rosparam

rosparam 标签允许节点从参数服务器上 load、dump 和 delete YAML 文件，也可以使用在远程机器上，需要注意的是 delete 必须在 load 或者 dump 之后进行。

常见用法如下：

```xml
# 参数较多使用 yaml 文件
<rosparam command="load" file="$(find rosparam)/example.yaml">
<rosparam command="delete" param="my/param">

# 传递数组
<rosparam param="a_list">[1, 2, 3, 4]</rosparam>

<rosparam>
	a: 1
	b: 2
</rosparam>
  
<arg name="whitelist" default="[3, 2]"/>
<rosparam param="whitelist" subt_value="True">$(arg whitelist)</rosparam>
```

- command：load，dump，delete
- file：参数文件的路径
- param：参数名称
- subt_value：是否允许在 YAML 中替换参数

补充下 yaml 文件基本用法，yaml 文件就是单纯的来存储启动参数，格式如下：

```sh
a: 1
str: hello
c: 2.0
```

其中不需要指定变量类型，yaml 文件会自动确定类型。使用 yaml 文件的目的就是方便配置参数，如果有很多参数需要配置，不需要写很多 rosparam 命令。

yaml 文件还有挺多复杂的写法，后续项目中用到再总结出来。

### 2.7 include

include 类似编程语言中的 include 预处理，它可以导入其他 roslaunch 的启动文件到当前 include 标签所在的位置。

常见用法如下：

```xml
<include file="$(find package_name)/launch_file_name">
```

项目中使用绝对路径不太方便，可以使用 find 来查找。

### 2.8 env

env 标签可以在启动的节点上设置环境变量，这个标签基本只会使用在 launch、include、node、machine 这 4 个标签内部，当使用在 launch 内部时，env 设置的环境标量会应用到内部定义的节点。

常见用法如下：

```xml
<env name="ENV_EXAMPLE" value="some value" />
```

- name：环境变量名称
- value：环境变量值

### 2.9 test

test 标签在语法上类似 node 标签，但在功能上只表示当前的节点作为测试节点去运行。

常见用法如下：

```xml
<test test-name="test_1" pkg="my_pkg" type="test_1.py" time-limit="10.0" args="--test1">
```

- test-name：测试节点名称
- pkg：测试节点所在的包
- type：测试节点类型
- time-limit：超时时间
- arg：测试节点启动的命令参数

### 2.10 arg

arg 标签表示启动参数，该标签允许创建更多可重用和可配置的启动文件，其可以通过命令行、include 标签、定义在高级别的文件这 3 种方式配置值。

arg 标签声明的参数不是全局的，只能在声明的单个启动文件中使用，可以当成函数的局部参数来理解。

常见用法如下：

```sh
# 1. 命令行传递启动参数
roslaunch my_file.launch my_arg:=my_value
```

```xml
# 2. 定义时赋值
<arg name="arg_name" default="arg_name">
<arg name="arg_name" value="arg_name">
```

这两者有点区别，命令行传递的 arg 参数可以覆盖 default，但不能覆盖 value。

```xml
# 3. 通过 launch 文件传递，两个 launch 文件中的 arg 参数值必须相同！

# my_file.launch
<include file="include.launch">
	<arg name="hoge" value="fuga">
</include>

# include.launch
<launch>
	<arg name="hoge" value="fuga">
</launch>
```

注意 arg 和 param 标签的区别：

- arg：启动时参数，只在启动文件 launch 中有意义
- param：运行时参数，存储在参数服务器中

### 2.11 group

group 标签可以方便的将一组配置应用到组内的所有节点，它也具有命名空间 ns 特点，可以将不同的节点放入不同的 namespace。

常见用法如下：

```xml
<group ns="namespace">
	<node pkg="pkg_name1" .../>
	<node pkg="pkg_name2" .../>
	...
</group>
```

配合 if 和 unless 使用：

- if=value：value 为 true 则包含内部信息
- unless=value：value 为 false 则包含内部信息

```xml
<group if="$(arg foo1)">
	<node pkg="pkg_name1" .../>
</group>

<group unless="$(arg foo2)">
	<node pkg="pkg_name2" .../>
</group>
```

- 当 foo1 == true 时包含其标签内部
- 当 foo2 == false 时包含其标签内部




> {{ site.prompt }}

<div  align="center">
<img src="https://dlonng.com/images/wechart.jpg" width = "200" height = "200"/>