---
title: ROS 初级 - 编写简单的服务端和客户端(C++)
date: 2019-06-15 20:00:00
---
# ROS 初级 - 编写简单的服务端和客户端(C++)
***
> 版权声明：本文为 {{ site.name }} 原创文章，可以随意转载，但必须在明确位置注明出处！

## 1、编写服务端
这次利用之前已经编写好的服务文件 AddTwoInts.srv 来编写一个服务端，核心代码如下：

1）包含服务文件：
```cpp
#include "beginner_tutorials/AddTwoInts.h"
```

2）编写服务函数：
```cpp
bool add(beginner_tutorials::AddTwoInts::Request  &req, beginner_tutorials::AddTwoInts::Response &res)
```

3）创建服务器对象：
```cpp
ros::ServiceServer service = n.advertiseService("add_two_ints", add);
```

4）自旋等待客户端访问：
```cpp
ros::spin();
```
下面看看客户端程序的核心代码。
## 2、编写客户端
1）创建客户端对象：
```cpp
ros::ServiceClient client = n.serviceClient<beginner_tutorials::AddTwoInts>("add_two_ints");
```

2）创建自己的服务类
```cpp
beginner_tutorials::AddTwoInts srv;
srv.request.a = atoll(argv[1]);
srv.request.b = atoll(argv[2]);
```
3）传递服务类对象来调用服务器
```cpp
client.call(srv)
```

## 3、编译
因为是通过 CMake 编译，所以要在 CMakeList.txt 文件中添加编译选项：
```
add_executable(add_two_ints_server src/add_two_ints_server.cpp)
target_link_libraries(add_two_ints_server ${catkin_LIBRARIES})
add_dependencies(add_two_ints_server beginner_tutorials_gencpp)

add_executable(add_two_ints_client src/add_two_ints_client.cpp)
target_link_libraries(add_two_ints_client ${catkin_LIBRARIES})
add_dependencies(add_two_ints_client beginner_tutorials_gencpp)
```

然后进入主工作空间，catkin_make：
```shell
cd ~/.catkin_ws
catkin_make
```

## 4、测试
编译成功后，先启动 ros：
```shell
roscore
```

然后新开两个终端，分别 source 一下当前的 ros 环境：
```shell
// 我的机器路径
source ~/catkin_ws/devel/setup.zsh
```

在一个终端中开启服务端：
```shell
rosrun beginner_tutorials add_two_ints_server
```
在另一个终端中开启客户端，传递参数 1 和 3：
```shell
rosrun beginner_tutorials add_two_ints_client 1 3
```
然后即可看到运行结果：

服务端：
<div  align="center">
<img src="{{ site.url }}/images/ros/server_client/server.png"/>
</div>

客户端：

<div  align="center">
<img src="{{ site.url }}/images/ros/server_client/client.png"/>
</div>

> {{ site.prompt }}

<div  align="center">
<img src="{{ site.url }}/images/wechart.jpg" width = "200" height = "200"/>