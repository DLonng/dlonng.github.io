---
title: Ubuntu 16.04 安装 PCL 库并测试
date: 2019-11-16 16:00:00
---
# Ubuntu 16.04 安装 PCL 库并测试
***
> 版权声明：本文为 {{ site.name }} 原创文章，可以随意转载，但必须在明确位置注明出处！

最近在做点云和图像融合方面的算法，需要使用一个 PCL（Point Cloud Lib）点云库，记录下安装过程。

PCL 官网提供了编译好的包，但是网络不好，没有安装成功，所以就直接从源码编译了，一路顺利，没有错误。

### 1、配置依赖包
```
sudo apt-get update
sudo apt-get install git build-essential linux-libc-dev cmake cmake-gui libusb-1.0-0-dev libusb-dev libudev-dev mpi-default-dev openmpi-bin openmpi-common libflann1.8 libflann-dev libeigen3-dev libboost-all-dev libvtk5.10-qt4 libvtk5.10 libvtk5-dev libqhull* libgtest-dev freeglut3-dev pkg-config libxmu-dev libxi-dev mono-complete qt-sdk openjdk-8-jdk openjdk-8-jre
```

### 2、编译源码
clone 源码到用户主目录：
```
cd ~/
git clone https://github.com/PointCloudLibrary/pcl.git
```
使用 CMake 编译，创建 build 文件夹，并进入：
```
cd ~/pcl
mkdir build
cd build
```
cmake 配置：
```
cmake -DCMAKE_BUILD_TYPE=None -DCMAKE_INSTALL_PREFIX=/usr -DBUILD_GPU=ON -DBUILD_apps=ON -DBUILD_examples=ON -DCMAKE_INSTALL_PREFIX=/usr ..
```
编译，要挺久的：
```
make
```
然后 sudo 安装：
```
sudo make install
```

### 3、测试
在用户主目录下建立 test_pcl 文件夹：
```
mkdir ~/test_pcl
cd test_pcl
```
建立 test_pcl.cpp 文件：
```cpp
#include <iostream>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
 
int main(int argc, char **argv) {
  std::cout << "Test PCL !" << std::endl;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  uint8_t r(255), g(15), b(15);
  for (float z(-1.0); z <= 1.0; z += 0.05) {
  	for (float angle(0.0); angle <= 360.0; angle += 5.0) {
      pcl::PointXYZRGB point;
      point.x = 0.5 * cosf (pcl::deg2rad(angle));
      point.y = sinf (pcl::deg2rad(angle));
      point.z = z;
      uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
      point.rgb = *reinterpret_cast<float*>(&rgb);
      point_cloud_ptr->points.push_back (point);
    }
    if (z < 0.0) {
      r -= 12;
      g += 12;
    }
    else {
      g -= 12;
      b += 12;
    }
  }
  
  point_cloud_ptr->width = (int) point_cloud_ptr->points.size ();
  point_cloud_ptr->height = 1;

  pcl::visualization::CloudViewer viewer ("test");
  viewer.showCloud(point_cloud_ptr);
  while (!viewer.wasStopped()){ };
  return 0;
}
```
再建立 CMakeLists.txt 文件：
```
cmake_minimum_required(VERSION 2.6)
project(test_pcl)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(test_pcl test_pcl.cpp)

target_link_libraries (test_pcl ${PCL_LIBRARIES})

install(TARGETS test_pcl RUNTIME DESTINATION bin)
```
建立 build 文件夹来编译源码：
```
mkdir build
cd build
cmake ..
make
```
运行：
```
./pcl_test
```
出现一个 demo 窗口，使用鼠标滑轮和左右键可以改变大小和方向：

<div  align="center">
<img src="https://dlonng.com/images/test_pcl.png"/>
</div>

测试源码：[https://github.com/DLonng/Autopilot-XMU/tree/master/PCL](https://github.com/DLonng/Autopilot-XMU/tree/master/PCL)

> {{ site.prompt }}

<div  align="center">
<img src="https://dlonng.com/images/wechart.jpg" width = "200" height = "200"/>