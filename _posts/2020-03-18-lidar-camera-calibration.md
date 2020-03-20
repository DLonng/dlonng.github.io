---
title: Camera-LIDAR 联合标定方法总结
date: 2020-03-18 20:00:00
---
# Camera-LIDAR 联合标定方法总结
***
> 版权声明：本文为 {{ site.name }} 原创文章，可以随意转载，但必须在明确位置注明出处！

项目需要融合雷达和相机，所以要做联合标定，记录下收集的标定方法。

## 一、总体标定步骤

标定就是找到雷达到相机的空间转换关系，在不同的坐标系之间转换需要旋转矩阵 R 和平移矩阵 T，为后续的雷达和相机数据融合做准备：

<div  align="center">
<img src="https://img-blog.csdn.net/20180903162619645?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2xlYXJuaW5nX3RvcnRvc2ll/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70"/>
</div>



Camera-LIDAR 联合标定分为 2 步：

1. 相机内参标定
2. 雷达相机外参标定

### 相机内参标定方法

- [ROS Camera Calibration Tools](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration)
- [SLAM之相机标定](https://blog.csdn.net/learning_tortosie/article/details/79901255)

### 外参标定：lidar_camera_calibration

- [lidar_camera_calibration](https://github.com/ankitdhall/lidar_camera_calibration)
- [lidar_camera_calibration项目——激光雷达和相机联合标定](https://tumihua.cn/2019/02/20/698adb0e68a9eaa19a52d52c69f34bea.html)
- [相机雷达联合标定]([http://www.elkulas.com/2019/05/22/%E7%9B%B8%E6%9C%BA%E9%9B%B7%E8%BE%BE%E8%81%94%E5%90%88%E6%A0%87%E5%AE%9A/](http://www.elkulas.com/2019/05/22/相机雷达联合标定/))
- [无人驾驶-激光雷达与相机联合校准（Lidar Camera Calibration）](https://blog.csdn.net/a2281965135/article/details/79785784)

### 外参标定：but_calibration_camera_velodyne

- [but_calibration_camera_velodyne](https://github.com/robofit/but_velodyne/tree/master/but_calibration_camera_velodyne)
- [博客：but_calibration_camera_velodyne](https://blog.csdn.net/learning_tortosie/article/details/82385394)

### 外参标定：Autoware

- [激光雷达和相机的联合标定（Camera-LiDAR Calibration）之Autoware](https://blog.csdn.net/learning_tortosie/article/details/82347694)
- [无人驾驶汽车系统入门（二十二）——使用Autoware实践激光雷达与摄像机组合标定](https://blog.csdn.net/AdamShan/article/details/81670732)
- [使用Autoware进行(双目)相机与激光雷达的联合标定](https://blog.csdn.net/X_kh_2001/article/details/89163659?depth_1-utm_source=distribute.pc_relevant.none-task&utm_source=distribute.pc_relevant.none-task)
- [Camera-Lidar Calibration with Autoware](https://zhuanlan.zhihu.com/p/65226371)
- [视觉激光雷达信息融合与联合标定](https://zhuanlan.zhihu.com/p/55825255)
- [激光雷达（lidar）和相机(camera)联合标定调研（基于Autoware的详细步骤）](https://blog.csdn.net/mxdsdo09/article/details/88370662)
- [多目相机、Velodyne标定那些破事](http://s1nh.org/post/calib-velodyne-camera/)

### 外参标定：Apoll

- [激光雷达和相机的联合标定（Camera-LiDAR Calibration）之apollo](https://blog.csdn.net/learning_tortosie/article/details/82351553)



## 二、其他传感器联合标定方法

### 2.1 LIDAR-IMU 联合标定

- [lidar-imu 联合标定](https://blog.csdn.net/Crystal_YS/article/details/90763723#lidarimulidar_align_12)
- [lidar-imu 联合标定开源项目](https://github.com/ethz-asl/lidar_align)



### 2.2 Camera-IMU 联合标定

- [利用kalibr工具进行camera-IMU标定](https://blog.csdn.net/zhubaohua_bupt/article/details/80222321)
- [kalib](https://github.com/ethz-asl/kalibr)



持续更新...


> {{ site.prompt }}

<div  align="center">
<img src="https://dlonng.com/images/wechart.jpg" width = "200" height = "200"/>