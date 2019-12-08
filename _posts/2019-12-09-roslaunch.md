---
title: ROS 初级 - 解析 roslaunch 文件
date: 2019-12-08 22:00:00
---
# ROS 初级 - 解析 roslaunch 文件
***
> 版权声明：本文为 {{ site.name }} 原创文章，可以随意转载，但必须在明确位置注明出处！

### 1、roslaunch 命令

roslaunch 命令允许我们启动 launch 文件中定义的节点：

```sh
roslaunch [package] [filename.launch]
```

例如：

```sh
roslaunch beginner_tutorials turtlemimic.launch
```

其中 beginner_tutorials 是 turtlemimic.launch 文件中定义的一个节点，那 launch 文件是什么呢？我们来看下。

### 2、解析 launch 文件

简单来说 launch 文件就是一堆节点和参数的集合，使用 launch 文件的目的是为了一键启动。因为目前运行在机器人上的 ROS 系统一般都要同时运行多个节点，如果我们开机启动时，一个一个的输入命令来启动每一个节点，想想就可怕，所以 ROS 系统就给我们提供了这么一个 launch 文件，可以方便我们将要启动的节点和对应的参数全部写到这个文件中，然后使用 roslaunch 命令一键启动！

下面是一个简单的 launch 文件，我带你一步一步解析它：

```sh
<launch>

  <group ns="turtlesim1">
    <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
  </group>

  <group ns="turtlesim2">
    <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
  </group>

  <node pkg="turtlesim" name="mimic" type="mimic">
    <remap from="input" to="turtlesim1/turtle1"/>
    <remap from="output" to="turtlesim2/turtle1"/>
  </node>

</launch>
```











<div  align="center">
<img src="https://dlonng.com/images/xxx/xxx.png"/>
</div>

> {{ site.prompt }}

<div  align="center">
<img src="https://dlonng.com/images/wechart.jpg" width = "200" height = "200"/>