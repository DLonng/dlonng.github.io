---
title: 自动驾驶规划算法 - Apollo LatticePlanner 基本原理
date: 2021-04-14 23:00:00
---
# 自动驾驶规划算法 - Apollo  LatticePlanner 基本原理
***
> 版权声明：本文为 {{ site.name }} 非原创文章，可以随意转载，但必须在明确位置注明出处！

**本文非原创，参考博客见文末！**

## Apollo 总体和规划框架

![img](https://img-blog.csdn.net/20180903041914315?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)



![img](https://img-blog.csdn.net/20180903041923497?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

规划模块的输入输出

- 输入：预测模块、routing模块、高精地图和定位的结果
- 输出：一条平稳、舒适、安全的轨迹（每一个轨迹点包含位置，速度，加速的等信息），交给控制模块去执行

## 规划算法的要求

![img](https://img-blog.csdn.net/2018090304211457?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

一个合格的规划算法，必须满足几个条件：

- 必须能够使自动驾驶汽车到达目的地
- 必须符合交规
- 能够避免碰撞
- 也需要能保证一定的舒适性（优化 Jerk）

在 Apollo 中，规划算法的输出是一系列轨迹点连成的轨迹，每一个轨迹点包含位置，速度，加速的等信息。

## Lattice Planner 基本流程

简介：**Lattice Planner 是一种基于采样 + 选择的路径、速度同时规划的规划器**

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/20210415202341.png)

图中前面蓝色带尖头的曲线是蓝车的预测轨迹。

![img](https://img-blog.csdn.net/20180903042137177?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

### 1 采样候选轨迹

面对这样的场景，有些司机会按照右图中**浅红色**的轨迹，选择绕开蓝色的障碍车。另外有一些司机开车相对**保守**，会沿着右图中**深红色**较短的轨迹做一个减速，给蓝色障碍车让路。

![img](https://img-blog.csdn.net/20180903042058964?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

既然对于同一个场景，人类司机都会有多种处理方法，那么 Lattice 规划算法也可以这样做，方法就是第一步**采样足够多的候选轨迹**，为后续提供尽可能多的选择。

![img](https://img-blog.csdn.net/20180903042223554?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

### 2 计算轨迹成本

采样完足够多的候选轨迹后，算法如何知道选择哪一条呢？Lattice 的方法是对每条轨迹计算一个 cost，然后选择 cost 最小的轨迹，这就相当于模拟我们人类选择最优的轨迹去执行。计算 cost 的方法需要保证轨迹的可行性、舒适性等因素。

![img](https://img-blog.csdn.net/20180903042235286?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

### 3 选择最低成本轨迹、限制检测

计算出轨迹的 cost 以后，第三步就是一个不断挑选最优轨迹的过程。在这个过程中，我们每次会先挑选出 **cost 最低**的轨迹，对其进行**物理限制检测和碰撞检测**。如果挑出来的轨迹不能同时通过这两个检测，就将其筛除，考察下一条 cost 最低的轨迹。

**动力学运动学限制**：以下图为例，假设我们首先挑选出 cost最低的是**深红色较短的轨迹**。但我们发现即便猛踩刹车也无法执行这条减速的轨迹。也就是说，这条轨迹的减速度超出了汽车的减速度上限，那么它就无法通过物理限制检测，我们会将其筛除。

![img](https://img-blog.csdn.net/20180903042329859?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

**碰撞检测**：假设我们下一条选出来 cost 最低的轨迹是右图中**深红色较长**的轨迹。我们会发现若沿着这条轨迹前进，红车会和蓝色障碍车**发生碰撞**。也就是说，这条轨迹轨迹**无法通过碰撞检测**，于是只能放弃这条轨迹，考虑下一条 cost 最低的。

![img](https://img-blog.csdn.net/20180903042338278?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

这个过程循环继续下去，假设我们现在挑选出右图中靠左边的深红色轨迹，它既符合汽车的动力学和运动学约束，也不会有碰撞风险，在所有候选轨迹中是最优的，我们就选它。

![img](https://img-blog.csdn.net/20180903042349194?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

### 4 输出轨迹 

得到 cost 最小的轨迹后，就将轨迹发送给底层控制模块执行。

## Lattice Planner 详细流程

### 1 采样候选轨迹

在自动驾驶规划问题中，我们的工作是基于道路的。这种情况下，X-Y 坐标系并不是最方便的，这里需要使用基于车道线横向和纵向的 Frenet 坐标系来表示一辆汽车的状态。

![img](https://img-blog.csdn.net/20180903042645630?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

- 首先有一条光滑的参考线（右图中红线），可以按右图所示将汽车的坐标点投影到参考线上，得到一个参考线上的投影点（图中蓝色点）
- 从参考线起点到投影点的路径长度就是汽车在 Frenet 坐标系下的纵向偏移量，用 s 表示
- 投影点到汽车位置的距离则是汽车在 Frenet 坐标系下的横向偏移量，用 l 表示
- 因为参考线是足够光滑的，也可通过汽车的朝向、速度、加速度来计算出 Frenet 坐标系下，横向和纵向偏移量的一阶导和二阶导

**注意**：我们将横向偏移量 l 设计成纵向偏移量 s 的函数，这是因为对于大多数的汽车而言，横向运动是由纵向运动诱发的。

#### 1.1 Step 1：采样起始、终止状态

首先我们可以通过计算得到自动驾驶汽车在 Frenet 坐标系下的零时刻 `t0` 的起始状态（当前状态），为了生成一条轨迹，第一步就是在 Frenet 坐标系下采样一个在 `t1` 时刻的末状态。

![img](https://img-blog.csdn.net/20180903043159369?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

#### 1.2 Step 2：多项式拟合横纵向轨迹

第二步就是将末状态和起始状态做 **5 次多项式拟合**，分别形成横向和纵向的多项式轨迹。

![img](https://img-blog.csdn.net/20180903043029272?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

##### 1.2.1 采样横向轨迹（没看懂）

横向轨迹的采样需要涵盖**多种横向运动状态**：

- Apollo 的代码中设计了**三个末状态横向偏移量(表示什么含义？)，-0.5，0.0 和 0.5**
- 以及**四个到达这些横向偏移量的纵向位移**，分别为 **10，20，40，80**
- 我们用两层循环遍历各种组合，再通过多项式拟合，即可获得一系列的横向轨迹

![img](https://img-blog.csdn.net/20180903043337639?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

##### 1.2.2 采样纵向轨迹

对于纵向轨迹的采样，我们需要考虑三种状态

- 巡航
- 停车
- 跟车或超车

![img](https://img-blog.csdn.net/20180903043355603?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

###### 巡航状态

通过两层循环来完成采样。外层循环将速度从零到上限值按等间隔均匀遍历，内层循环遍历到达末状态速度的时间，从 1 秒到 8 秒按 1 秒的间隔均匀遍历，由于巡航状态**不需要指明到达末状态的 S 值（没有 $s(t_1)$ 的值）**，所以这里**只需要用四次多项式拟合**即可。

![img](https://img-blog.csdn.net/2018090304342678?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

###### 停车状态

给定停车点，末状态的速度和加速度都是零，所以末状态是确定的，那么我们只需用一层循环来采样到达停车点的时间即可。

![img](https://img-blog.csdn.net/20180905040542956?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

###### 超车或超车

在介绍跟车，超车的采样逻辑之前，需要介绍一下 S-T 图的概念，以下图中的场景为例，对应的 S-T 图就如右图所示：

- 蓝色障碍车从车道右侧切入，在 T_in 时刻开始进入当前车道
- 从 T_in 时刻开始出现一块斜向上的阴影区域
- 这块阴影区域的高度就是蓝色障碍车的车身长，上边界表示车头，下边界表示车尾，斜率表示车速

![img](https://img-blog.csdn.net/20180903043442840?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

如果上述场景变成这样，障碍车从 T_in 时刻进入车道，然后在 T_out 时刻离开车道，那么这个场景对应的 S-T 图就会缩短。

![img](https://img-blog.csdn.net/20180903043455355?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

有了 S-T 图的概念，观察下图中的两条规划轨迹：

- 红色的是一条跟车轨迹，在蓝色阴影区域下方（为何？）
- 绿色的是一条超车轨迹，在蓝色阴影区域上方（为何？）

![img](https://img-blog.csdn.net/20180903043509338?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

采样跟车或超车的末状态时，就可以分别在 S-T 图中障碍物对应的阴影区域的上方和下方分别采样，上方的末状态对应超车，下方的末状态对应跟车，如下图：

![img](https://img-blog.csdn.net/20180903043532753?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

如果有多个障碍物，我们就对这些障碍物分别采样超车和跟车所对应的末状态，那么总结下来就是遍历所有和车道有关联的障碍物，对他们分别采样超车和跟车的末状态，然后用多项式拟合即可获得一系列纵向轨迹：

![img](https://img-blog.csdn.net/2018090304360592?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

##### 1.2.3 如何根据起始、终止状态拟合多项式

在给定被控对象的初始以及终止状态后，反向计算中间过程，我们需要对其进行**两点边界值问题（BVP）**的求解，才能求得两个状态之间轨迹。

**基本思路**：**经典的两点边界值问题，其实就是多项式方程求系数的过程**

- 首先，我们将车辆的状态方程表示为**五阶多项式形式**
- 然后，对其状态方程求各阶导数，即可求得**位置、速度、加速度**关于时间的表达式
- 最后，我们只需要将两点边界状态信息（每个边界有位置，速度，加速度 3 个状态信息）代入对应的多项式方程（一共 6 个方程），联立即可求得多项式的 6 个系数（解方程组），以此得到 2 点之间的轨迹

![img](https://pic4.zhimg.com/80/v2-427a05d21ca9a1aaaa0fdd77b79f8c8b_720w.jpg)

#### 1.3 Step 3：二维合成

最后将三组纵向轨迹组合起来，就可以获得所有纵向轨迹，再将所有纵向轨迹和所有横向轨迹两两配对二维合成，就可以完成轨迹采样的工作。

![img](https://img-blog.csdn.net/20180903043617526?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

二维合成的步骤：

- 给定一个时刻 `t*`，可以计算出在 `t*` 时刻的纵向偏移量 `s(t*)`和横向偏移量 `l(t*)`
- 再通过参考线，即可还原（如何还原？）成一个二维平面中的轨迹点 $p_{t*}$
- 通过一系列的时间点 `t0，t1，...，tn`，可以获得一系列的轨迹点 `P0，P1，…，Pn`，最终形成一条完整的轨迹。

![img](https://img-blog.csdn.net/20180903043617526?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

### 2 计算轨迹成本

前面提到，轨迹规划所需要满足的四点要求，分别是：

- 到达目的地
- 符合交规
- 避免碰撞
- 平稳舒适

针对这四点要求，Lattice Planner 设计了 6 个 cost，cost 越高就表示越不满足要求

![img](https://img-blog.csdn.net/2018090304364982?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

#### 2.1 Objective achievement cost

![img](https://img-blog.csdn.net/20180903043636630?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

目的地成本分成两种情况：

- 如果存在停车指令（红灯），相对大的车速对应的轨迹 cost 就越大（要优先减速，安全第一）
- 如果没有停车指令（绿灯），那么低速轨迹的 cost 就会越大（优先选择速度快的轨迹到达目的地）

![img](https://img-blog.csdn.net/20180903043709817?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

实现方法，如何用一个 cost function 处理 2 种情况？设置参考速度 Ref Speed，用轨迹的速度与参考速度相减求差的绝对值，绝对值越小 cost 越小

- 上面左图蓝线表示没有停车指令时的参考速度（较大）
    - 绿色的加速轨迹会获得一个较小的 cost
    - 红色的减速轨迹会获得一个较大的 cost
- 上面右图蓝线表示有停车指令时的参考速度（参考速度会呈下降趋势）
    - 绿色的减速轨迹会获得一个较小的 cost
    - 红色的加速轨迹会获得一个较大的 cost

#### 2.2 Lateral offset cost

横向偏移成本是为了让自动驾驶汽车能尽量**沿着道路中心行驶**，像左图靠道路一边行驶，和中间画龙的行驶轨迹，他们的 cost 都相对较高。

- 实现方法：计算与道路中心线的距离，距离越短 cost 越小？

![img](https://img-blog.csdn.net/20180903043739572?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

#### 2.3 Collision Cost

碰撞成本，左图中的两条轨迹反映在右边 S-T 图中，我们可以发现：

- 红色的轨迹和蓝色障碍车在 S-T 图中的阴影区域有重叠，说明有碰撞风险，那么它的碰撞 cost 就会相对较高
- 绿色的轨迹在 S-T 图中没有与蓝色障碍车产生重叠，那么它的碰撞 cost 就相对较低

实现方法：？？？

![img](https://img-blog.csdn.net/20180903043751316?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

#### 2.4 Longitudinal jerk cost

纵向加加速度的 cost，加加速度（**jerk**）是加速度对时间的导数，表示加速度的变化率，反映乘坐的舒适度。

- 实现方法：用加加速度的最大值来表示这个 cost
    - jerk 较大，舒适度较差
    - jerk 较小，舒适度较高

![img](https://img-blog.csdn.net/20180903043803339?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

#### 2.5 Lateral acceleration cost

横向加速度的 cost，功能是为了**平稳地换道**，像左图猛打方向盘的轨迹，它的横向加速度 cost 就会相对较大

- 实现方法：用横向加速度的最大值来表示这个 cost ？
    - 横向加速度越大，cost 越大
    - 横向加速度越小，cost 越小

![img](https://img-blog.csdn.net/20180903043833333?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

#### 2.6 Centripetal acceleration cost

**向心加速度成本**，目的是为了在**转弯或调头的时候能够减速慢行**。在弯道处，车速慢的轨迹，其向心加速度 cost 就会相对较低，那么就会更容易被率先挑选出来。

- 实现方法
  - 计算向心加速度 cost？

![img](https://img-blog.csdn.net/20180903043845107?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

### 3 选择最低成本轨迹、限制检测

#### 3.1 目标函数

将以上的 6 个成本函数**加权累加**作为优化的目标函数，因此需要 6 个参数，开发者可以根据产品的需要，调试这 6 个权重参数。

![img](https://img-blog.csdn.net/20180903043856997?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

#### 3.2 限制检测

限制检测考虑的内容有：

- 轨迹的加速度、加加速度、和曲率
- 碰撞检测：把轨迹和其他**障碍物的预测轨迹**进行比对，观察是否有轨迹重叠

![img](https://img-blog.csdn.net/20180903043913234?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

### 4 输出轨迹

应该是发送第一个轨迹点的速度，加速度给底层的控制器。

## Lattice Planner 优缺点

- 优点
  - 同时结合了横向与纵向规划，考虑了车辆动力学
  - 适合简单低速的场景
  - 参数较少，流程简单
- 缺点
  - 不太适用于复杂的城市场景

## Lattice Planner 相关问题

### 1 Lattice 如何换车道？

对于换道场景，Lattice 算法仅仅需要**对目标车道对应的参考线做一次采样 + 选择**的流程。本车道和目标车道均能产生一条最优轨迹，给**换道轨迹的 cost 上增加额外的车道优先级的 cost**，再将两条轨迹比较，选择 cost 较小的那条即可。

为何要给换道的轨迹增加额外的车道优先级？

- 目的是不让车辆经常换道，尽量保持当前车道行驶
- 只有在增加车道优先级成本后的换道轨迹成本比当前车道最优轨迹成本要小才选择换道

![img](https://img-blog.csdn.net/20180903043254517?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

### 2 Lattice Planner 将规划统一成代价函数，寻找代价最小的，在规划的上层是否还需要决策层？

在规划**上层的决策仅仅包含了来自交规的停车指令**（比如红绿灯），其余的策略均有下层采样 + cost 来完成。

### 3 Lattice Planner 适用于哪些场景？

Lattice Planner 现在已经在**低速园区和高速公路的场景中**由产品落地，对于普通城市道路，对于相对复杂的交规处理还有待完善。

### 4 适合多弯道复杂的场景下吗？复杂的停车场等？

该算法**可以处理多弯道**的场景，对于**停车场暂不使用**，因为这个算法首先需要参考线，而**复杂的停车场很难做出一条参考线**。

 ### 5 Cost 里面已经考虑了碰撞成本，为什么后续还要做碰撞检测？

- 碰撞成本是一个比较软的限制
  - 仅仅是把有碰撞风险的轨迹的 cost 值设置的比较高，为了把这样的轨迹优先级排到比较后，从而使得我们能够优先考察其他更安全的轨迹，但它并**没有起到删选轨迹的作用**
- 碰撞检测是一个硬的限制
  - 出于安全的考虑，把会产生碰撞的轨迹直接删除

### 6 Lattice Planner 和 EM Planner 的区别是？或者说分别应用在什么场景下？

- 区别：Lattice Planner 主要基于**采样+选择**，而 EM Planner 的思路是**逐层优化迭代**
- 场景：
  - 从规划层来看：两者均可用于各种场景
  - 从决策层来看：
    - Lattice 的决策相对简单，适用于相对简单的场景，如低速园区，高速公路
    - EM 算法对交规的决策做的相对更完善，可以处理相对复杂的普通城市道路

### 横向轨迹和纵向轨迹俩俩组合咋样理解? 是横向的一条轨迹和纵向的所有轨迹组合吗？

两两组合指的是每一条横向轨迹和每一条纵向轨迹的组合

### 8 计算量是不是有点大？普通CPU可以吗？

以目前的经验来看，普通 CPU 是可以处理的。当然，这个算法可以**随着计算机性能的提升，采样更多的轨迹，使得我们对解空间的涵盖更加完备**。

### 9 关于轨迹的生成，Lattice 使用的是多项式拟合，一般使用三项还是五项？

- 对于横向轨迹
  - 初始状态的零阶导，一阶导，二阶导
  - 末状态的零阶导，一阶导，二阶导
  - 一共 6 个变量，所以拟合 5 次多项式。
- 对于纵向轨迹
  - 停车和跟车状态，也是 5 次多项式
  - 巡航状态，由于我们不需要确定末状态的 S 值，所以只有 5 个变量，那么用 4 次多项式就可以

### 10 在 st 图上下阴影处取样，那是取多少个点？那个末状态是指超车结束的状态点吗？

- 在 ST 图上下取样，点的数目可以由开发者自行决定，这个没有限制
- 末状态指的就是超车结束的状态点

### 11 只有起始状态和末状态怎么进行横向和纵向的拟合？

通过求解多项式的系数使得起始点和终止点的各阶导数满足多项式。

### 12 Lattice Planner 是路径规划的算法，但也涵盖了部分行为规划的处理内容？

是的。横向轨迹主要针对路径，纵向轨迹主要针对速度、加速度等行为。

### 13 多项式拟合，是什么多项式？拟合后如何保证满足无人车的运动学和动力学要求？

多项式指的就是普通的多项式，**拟合时已经通过高阶导数考虑了动力学要求**。

### 14 多项式拟合具体方法是什么？如何避免拟合的曲线超出路面范围？

- 拟合的具体方法是**求解多项式系数的线性方程组**
- 开发者可以通过添加在路检测的检查 validity check 来避免曲线超出路面范围 

### 15 Apollo 的代码中出现了 S-L 坐标系和 Frenet 坐标系的名称，这两个是否是指同一个坐标系？

这两个指的是同一个坐标系。

### 16 换道场景，是要提前获取目标车道和当前行驶车道的参考线吗？

是的，这个信息在 Apollo 里都是有的。

### 17 Frenet 坐标系下面是一个车道一个中心线？还是一条线一个中心线？这两的 s 值的 0 点是从什么地方开始算的？

- 一个车道一个中心线
- S 值的 0 点是从参考线的起点开始
  - 参考线的起点会随着主车的位置做**实时动态调整**
  - **通常是在主车车身后 30 米左右的位置**

### 18 在障碍车辆较多的环境下可能需要频繁的规划路径，由于 cost 值有多个评价组成，有可能多次出现最佳轨迹的横向方向完全相反的情况，可能造成车辆左右微微摆动，如何解决这种情况？

在这种情况下，建议额外补充一个和上一周期相似性的 cost

- 相似性如何计算？
  - 保存上一周期最优轨迹的横向方向（左 -1 或者右 1）
  - 计算当前周期轨迹的横向方向与上一周期的差的绝对值
    - 相同方向，cost 小：abs (-1 - (-1)) = 0，abs(1 - 1) = 0
    - 不同方向，cost 大：abs(-1-1) = 2，abs(1 - (-1)) = 2

### 19 对于单向双车道场景，规划的时候是只规划行驶车道还是规划双车道?

如果有换道需求的话，会同时规划两条车道。

### 初始状态本车的 theta 是车头朝向还是指车速朝向， 初始状态曲率怎么计算，用方向盘角度推？

- 初始状态的 theta 是车头方向
- 曲率是基于IMU的信息计算出来的

### 21 在运行 Apollo demo 时 ，在只有软件模拟没有硬件的情况，能不能测试和调试规划算法，能的话怎么做？

Apollo 有一个开放的仿真平台，azure.apollo.auto，开发者可以在仿真平台中调试规划算法。

### 22 有碰撞风险的轨迹为什么不在 cost 环节或者之前直接删掉？

cost 环节仅仅是一个软的排序，并不做删除的工作。

### 23 如果如果在过弯道的时候，reference line 上的 end point 有噪音，在不停都抖动，如何在这种情况下规划一条稳定的轨迹？

这就需要我们不断优化 reference line 的**平滑**算法。

### 24 现在 Apollo 的代码中设计了三个末状态横向偏移量，-0.5，0.0 和 0.5，以及四个到达这些横向偏移量的纵向位移，分别为 10，20，40，80，能解释下为什么这样定意思这些常量？

这些常量是根据平时路测的经验得到的，开发者可以根据自己的产品和场景来调整这些常量。

### 25 横向运动是由纵向运动诱发的，该如何理解？

汽车是非全向运动的，就是不能横向平移，要转弯必须先向前走。

## 不懂的问题

- 如何做 4 次或 5 次多项式拟合？
  - 求经典的两点边界值问题（BVP）
- 如何采样横向和纵向轨迹？
  - 在确定了终点和起点状态以后，再通过五阶或者四阶的多项式连接起始状态和终止状态，从而得到规划的横向和纵向轨迹
  - 这个步骤是 Lattice 算法的精髓所在，它直接决定了算法的效率以及解空间
  - 举个例子来说，在撒点的时候可以的加入车辆物理动力学性能的约束，同时根据障碍物进行可行域的预筛选等，有效的裁剪无效空间，从而达到更优的性能
- 轨迹会不会出现都被筛掉的情况，如果出现怎么办？
- `DWA/MPC` 是执行第一个轨迹点的速度，Lattice 也是吗？
- 为什么巡航状态只需要拟合 4 次多项式拟合？
  - 由于巡航状态**不需要指明到达末状态的 S 值（没有 $s(t_1)$ 的值）**
- 三个末状态横向偏移量表示什么含义？
  - -0.5，0.0 和 0.5，左右偏移 0.5 米？
- 二维轨迹合成中如何通过参考线还原一个二维平面中的轨迹点？
- 如何计算轨迹的横向偏移成本？
  - 计算与参考线的距离？
- 碰撞成本如何根据 ST 图计算？
- 向心加速度成本如何计算？
  - 直接以向心加速度最大值作为 cost ？
- 目标函数使用加权多目标和的原理？



## 参考博客

- [https://www.jianshu.com/p/0ce5a4064b30](https://www.jianshu.com/p/0ce5a4064b30)
- [https://zhuanlan.zhihu.com/p/145466792](https://zhuanlan.zhihu.com/p/145466792)
- [https://zhuanlan.zhihu.com/p/339947476](https://zhuanlan.zhihu.com/p/339947476)
- [https://blog.csdn.net/yuxuan20062007/article/details/82330165](https://blog.csdn.net/yuxuan20062007/article/details/82330165)


> {{ site.prompt }}



![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/dlonng_qrcode.jpg#pic_center)