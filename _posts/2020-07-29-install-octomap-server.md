---
title: ROS 八叉树地图构建 - 安装 octomap 和 octomap_server 建图包！
date: 2020-07-29 16:00:00
---
# ROS 八叉树地图构建 - 安装 octomap 和 octomap_server 建图包！
***
> 版权声明：本文为 {{ site.name }} 原创文章，可以随意转载，但必须在明确位置注明出处！


项目要用到八叉树库 Octomap 来构建地图，这里记录下安装、可视化，并启用带颜色的 Octomap 的过程。

## 一、Apt 安装 Octomap 库

如果你不需要修改源码，可以直接安装编译好的 octomap 库，记得把 ROS 版本「kinetic」替换成你用的：

```shell
sudo apt-get install ros-kinetic-octomap*
```

上面这一行命令等价于安装以下的 octomap 组件：

```shell
sudo apt-get install ros-kinetic-octomap ros-kinetic-octomap-mapping ros-kinetic-octomap-msgs ros-kinetic-octomap-ros ros-kinetic-octomap-rviz-plugins
```

注意：上面没有安装 `ros-kinetic-octomap-server`，原因是我要使用这个包来建图，并且需要修改它，所以在下一步我直接通过编译源码来安装它！

## 二、编译安装 OctomapServer 建图包

因为我要启用八叉树体素栅格的 RGB 颜色支持，需要修改源码，所以必须使用源码编译安装，过程如下：

### 2.1 创建编译用的工作空间

```shell
cd 你的一个目录/

# 创建工作空间
mkdir octomap_ws
cd octomap_ws/

# ROS 的工作空间必须包含 src 目录
mkdir src/

# 创建
catkin_make

# 记得 source 环境变量
source devel/setup.zsh
```

### 2.2 下载编译源码

下载 [octomap_server](https://github.com/OctoMap/octomap_mapping) 源码到 src 文件夹中：

```shell
cd src/
git clone https://github.com/OctoMap/octomap_mapping.git
```

返回你的工作空间主目录，安装下依赖，然后开始编译：

```shell
cd ../

rosdep install octomap_mapping

catkin_make
```

编译过程基本没有报错，如果你遇到问题，直接复制错误信息浏览器搜索解决，然后启动测试的 launch：

```shell
roslaunch octomap_server octomap_mapping.launch
```

没问题的话应该可以用 `rostopic list` 看到一个 `octomap_full` 的话题：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/build_octomap.png)

有这个话题说明这个建图包可以正常工作啦：）

## 二、Rviz 可视化 Octomap

ROS 中提供了一个 Rviz 可视化 Octomap 的插件，如果没有安装使用下面的命令即可：

```shell
sudo apt-get install ros-kinetic-octomap-rviz-plugins
```

安装后启动 Rviz，直接添加一个八叉树占用网格的类型，第一个是带颜色的类型，第二个不带颜色：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/octomap_rviz_plugin2.png)

建图节点启动后，选择话题名称为 `octomap_full`，即可显示出八叉树体素栅格，这是我的实验结果，我用的是第一个带颜色的类型：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/octomap_rviz_result.png)

我把点云和体素栅格一起显示了，所以会重叠。这里要注意的是，如果你的点云显示不出来，要检查下「Global Options」的「Fixed Frame」有没有设置正确，我是设置的是 Robosense 雷达的 frame_id：「rslidar」。

## 三、启用 ColorOctomap

默认编译的 octomap 不能显示颜色，要开启颜色的支持，需要 2 个步骤，第一步编辑 `OctomapServer.h` 文件：

```shell
vim octomap_mapping/octomap_server/include/octomap_server/OctomapServer.h
```

打开下面 `COLOR_OCTOMAP_SERVER` 宏的注释即可：

```cpp
// switch color here - easier maintenance, only maintain OctomapServer. 
// Two targets are defined in the cmake, octomap_server_color and octomap_server. One has this defined, and the other doesn't
// 打开这个注释
#define COLOR_OCTOMAP_SERVER
```

然后重新编译一遍源码：

```shell
cd octomap_ws/

catkin_make
```

第二步是在使用时，在 `launch` 文件中禁用 `height_map`，启用 `colored_map`，这个配置是我阅读源码查找的，因为官网文档很久没有更新了，一些参数配置方法需要通过阅读源码才能知道：

```xml
<param name = "height_map" value = "false" />
<param name = "colored_map" value = "true" />
```

比如以下是我实验用的 launch 文件：

```xml
<launch>
  <node pkg="octomap_server" type="octomap_server_node" name="octomap_server">

    <!-- resolution in meters per pixel -->
    <param name="resolution" value="0.10" />

    <!-- name of the fixed frame, needs to be "/map" for SLAM -->
    <!-- 
         增量式构建地图时，需要提供输入的点云帧和静态全局帧之间的 TF 变换
    -->
    <param name="frame_id" type="string" value="world" />

    <param name = "height_map" value = "false" />
    <param name = "colored_map" value = "true" /> 

    <!-- topic from where pointcloud2 messages are subscribed -->
    <!-- 要订阅的点云主题名称 /fusion_cloud -->
    <remap from="/cloud_in" to="/fusion_cloud" />
  </node>
</launch>
```

我设置了八叉树帧的 frame 为 `rslidar`，并将融合的点云话题 `/fusion_cloud` 作为节点的输入，我没有提供 TF，因为目前只是做了一个单帧的体素栅格构建，效果如下：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/color_octomap.png)

我在 B 站录了个简短的视频，可以去看下初步的效果：[ROS 单帧带颜色八叉树 Octomap 地图构建实验](https://www.bilibili.com/video/BV1hz4y197Wb)




> {{ site.prompt }}



![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/dlonng_qrcode.jpg#pic_center)