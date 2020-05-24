---
title: ROS 机器人技术 - 如何编写一个完整的 ROS 功能包？
date: 2020-05-24 19:20:00

---

# ROS 机器人技术 - 如何编写一个完整的 ROS 功能包？

***

> 版权声明：本文为 {{ site.name }} 原创文章，可以随意转载，但必须在明确位置注明出处！



项目要求写一个图像点云的融合节点，花了段时间基本搞定了，今天来总结下编写一个 ROS 功能包最基本的步骤和常用的技术。

## 一、ROS 功能包的组成

以下是一个基本的 ROS Package 的组成，以我写的语义融合 Package lidar_camera_fusion 为例：

```
lidar_camera_fusion      -- 包名称
      include            -- 存放 cpp 头文件
      src                -- 存放 cpp 源文件
      launch             -- 存放启动文件
      CMakeLists.txt     -- CMake 的编译规则文件
      package.xml        -- 当前包的清单文件
      xxx                -- 其他文件或目录
```

这是我的包目录：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/fusion_pkg.png)

在创建 pkg 的时候建议使用命令自动添加依赖：

```shell
catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
```

如果后期需要添加别的依赖直接在 `CMakeLists.txt` 中添加即可，关于 CMake 构建规则、`package.xml` 和 `launch` 这 3 个配置文件的写法，我之前都总结过，建议系统的学习下：

- [ROS 机器人技术 - 解析 CMakeList.txt 文件](https://dlonng.com/posts/ros-cmakelist)
- [ROS 机器人技术 - 解析 package.xml 文件](https://dlonng.com/posts/ros-package)
- [ROS 机器人技术 - 解析 roslaunch 文件](https://dlonng.com/posts/roslaunch)

下面我再总结下我在编写节点的过程中用到的比较多的技术。

## 二、编写节点常用的技术

### 2.1 NodeHandle 获取启动参数

NodeHandle 称为节点句柄，在 ROS 节点中有 2 个功能：

- 提供节点的自动启动和结束，如果节点没启动则自动启动，当节点句柄变量离开作用域，节点自动关闭
- 提供一层额外的命名空间，使得子组件的编写更加容易

我在节点中把句柄设置为类成员变量，这样当这个类结束时节点才自动关闭：

```cpp
private:
    ros::NodeHandle nh;
```

初始化可以给这个节点提供一层命名空间：

```cpp
nh("my_namespace")
```

这样就可以提供一层「my_namespace」命名空间，比如把前者替换为后者：

-  `<node_spacename>/my_topic` 
-  `<node_namespace>/my_namespace/my_topic`

在节点初始化函数 InitRos 中，利用节点句柄获取 launch 文件中配置的启动参数。比如获取参数名为 `my_param` 的启动参数，并不指定缺省值：

```cpp
std::string s;

if (node_handle.getParam("my_param", s))
    ROS_INFO("Got param: %s", s.c_str());
else
    ROS_ERROR("Failed to get param 'my_param'");

```

如果我们想在获取 `my_param`参数失败后（比如忘记在 launch 文件中配置），可以用下面这种方法指定缺省值：

```cpp
std::string s;
node_handle.param<std::string>("my_param", s, "default_value");
```

更多关于 ROS 参数服务器的用法，见官方教程：

- [http://wiki.ros.org/cn/roscpp_tutorials/Tutorials/Parameters](http://wiki.ros.org/cn/roscpp_tutorials/Tutorials/Parameters)

### 2.2 订阅和发布话题

我在获取完参数后，接着就是订阅一些话题 topic 作为我们当前节点的数据输入，因为我是融合图像和点云，所以自然要订阅图像话题和点云话题，具体的话题名要从 launch 文件中获取：

```cpp
// 获取的参数名：image_input，获取的参数值存储在：image_input，缺省值：/cv_camera/image_raw
param_handle.param<std::string>("image_input", image_input, "/cv_camera/image_raw");
param_handle.param<std::string>("cloud_input", cloud_input, "/velodyne_points");
param_handle.param<std::string>("fusion_topic", fusion_topic, "/fusion_cloud");

// 订阅 image_input 话题
// 第二个参数是队列大小，以防我们处理消息的速度不够快，当缓存达到 1 条消息后，再有新的消息到来就将开始丢弃先前接收的消息。
// 当处理消息速度不够时，可以调大第二个参数！
// 收到订阅消息后调用 ImageCallback 处理图像 msg
sub_image = topic_handle.subscribe(image_input, 1, &LidarCameraFusion::ImageCallback, this);

// 订阅点云话题，参数同上
sub_cloud = topic_handle.subscribe(cloud_input, 1, &LidarCameraFusion::CloudCallback, this);
```

在我订阅完图像和点云，然后融合成点颜色的 RGB 点云之后，我就要把融合的点云给发布出去，别的节点如果需要就可以自行订阅，比如建图节点，发布一个主题如下：

```cpp
// 在 fusion_topic 上发布 sensor_msgs::PointCloud2 类型的消息
// 第二个参数为缓冲区大小，缓冲区被占满后会丢弃先前发布的消息，目前为 1，如果消息处理速度慢，可以调大该参数！
// 发布消息：pub_fusion_cloud.publish(msg)
pub_fusion_cloud = topic_handle.advertise<sensor_msgs::PointCloud2>(fusion_topic, 1);
```

### 2.3 TF 坐标系统

在整个项目中因为用到多个传感器，所以每个传感器的坐标系定义和维护就显得非常重要，ROS 中提供了 TF 坐标管理系统，可以很方便的管理整个系统的坐标转换关系，详细的 TF 基础知识见我之前的一篇文章，后续会继续更新的：

- [ROS 机器人技术 - TF 坐标系统基本概念](https://dlonng.com/posts/ros-tf2)

OK！今天就分享这些，大家下期再见喽！


> {{ site.prompt }}



![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/dlonng_qrcode.jpg#pic_center)

