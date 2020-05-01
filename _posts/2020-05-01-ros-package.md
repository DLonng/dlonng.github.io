---
title: ROS 机器人技术 - 解析 package.xml 文件
date: 2020-05-01 18:17:00
---
# ROS 机器人技术 - 解析 package.xml 文件
***
> 版权声明：本文为 {{ site.name }} 原创文章，可以随意转载，但必须在明确位置注明出处！

最近项目要写一个 ROS 语义点云融合节点，借此机会总结下 package.xml 文件的用法。

## 一、package.xml 文件作用

在每个 ROS 包中都包含一个 `package.xml` 文件，这个文件的作用是来描述这个功能包的基本信息：

- 包的名称
- 版本号
- 维护人员
- 作者
- 依赖项

比如下面这个例子：

```xml
<package format = "2">
  <name> foo_core </name>
  
  <version> 1.2.4 </version>
  
  <description>
      This package provides foo capability.
  </description>
  
  <maintainer email = "ivana@osrf.org"> Ivana Bildbotz </maintainer>
  
  <license> BSD </license>
  
  <depend> roscpp </depend>
  <depend> std_msgs </depend>
</package>
```

目前 ROS 有 2 个版本的 package.xml 编写语法，分别为 Format 1 和 Format 2，官方建议现在创建的 ROS 包使用 Format 2 的编写语法。

以下我对这两种语法都做下总结，其实语法差别不大，一起来学习下吧 (*^▽^*)！

## 二、package format 2 编写方法

### 2.1 根节点标签

每个 package.xml 文件的根节点都用 `<package>` 定义，默认是 Format 1 语法，可以指定使用 Format 2 语法：

```xml
<package format = "2">

</package>
```

### 2.2 必备的标签

以下的标签建议每个包的 package.xml 都添加：

- `<name>`：包的名称
- `<version>`：版本号，建议使用 3 个点分割，比如 1.2.3
- `<description>`：对该包功能的描述
- `<url>`：该功能包的对应网站
- `author`：该功能包的作者
- `maintainer`：该功能包的维护者
- `license`：开源许可证，比如 BSD，GPL 等

一个例子如下：

```xml
<package format = "2">
  <name> sensor_funsion </name>
  
  <version> 1.2.3 </version>
  
  <description>
      This package fusion point cloud and image.
  </description>
  
  <url> www.xxx.com </url>
  
  <author> DLonng </author>
  
  <maintainer email="xxx@xxx.com"> xxx </maintainer>
  
  <license> BSD </license>
</package>
```

### 2.3 依赖项标签

依赖项标签主要用来定义该功能包编译和执行时依赖的库，编译工具等，主要分为以下 6 类依赖：

1. 构建依赖项 `<build_depend>`：指定编译功能包需要的依赖包
2. 构建导出依赖项 `<build_export_depend>`：指定对功能包构建库需要依赖哪些功能包
3. 执行依赖项 `exec_depend`：指定运行该功能包所依赖的其他功能包
4. 测试依赖项 `test_depend`：指定进行单元测试的依赖包
5. 构建工具依赖项 `buildtool_depend`：指定构建该功能包使用的工具，默认使用 catkin
6. 文档工具依赖项 `doc_depend`：指定使用什么文档生成工具来为该功能包生成文档

**重要**：在 Format 2 语法中，使用 `<depend>` 标签来作为 `<build_depend>`、`build_export_depend` 和 `<exec_depend>` 这 3 个标签功能的组合，这是官方建议使用的标签：

```xml
<depend> roscpp </depend>

等价于以下 3 行依赖关系：

<build_depend> roscpp </build_depend>
<build_export_depend> roscpp </build_export_depend>
<exec_depend> roscpp </exec_depend>
```

每个 ROS 功能包都至少有一个依赖项，一个指定了构建、执行、测试、文档依赖项的 xml 文件如下：

```xml
<package format = "2">
  <name> sensor_funsion </name>
  
  <version> 1.2.3 </version>
  
  <description>
      This package fusion point cloud and image.
  </description>
  
  <url> www.xxx.com </url>
  
  <author> DLonng </author>
  
  <maintainer email="xxx@xxx.com"> xxx </maintainer>
  
  <license> BSD </license>

  <buildtool_depend> catkin </buildtool_depend>

  <depend> roscpp </depend>
  <depend> std_msgs </depend>

  <build_depend> message_generation </build_depend>

  <exec_depend> message_runtime </exec_depend>
  <exec_depend> rospy </exec_depend>

  <test_depend> python-mock </test_depend>

  <doc_depend> doxygen </doc_depend>
</package>
```

以上就是 package.xml 文件 Format 2 的语法编写方法，下面再来学习下 Format 1 的语法。

## 三、package format 1 编写方法

### 3.1 根节点标签

与 Format 2 一样，只是不用指定版本：

```xml
<package>

</package>
```

### 3.2 必备的标签

与 Format 2 的语法完全一样，不多解释了：

```xml
<package format = "2">
  <name> sensor_funsion </name>
  
  <version> 1.2.3 </version>
  
  <description>
      This package fusion point cloud and image.
  </description>
  
  <url> www.xxx.com </url>
  
  <author> DLonng </author>
  
  <maintainer email="xxx@xxx.com"> xxx </maintainer>
  
  <license> BSD </license>
</package>
```

### 3.3 构建、运行、测试依赖项标签

Format 1 的依赖项只有 4 个，用法跟 Format 2 一样，这里也不多解释了：

1. 构建工具依赖项 `<buildtool_depend>`
2. 构建依赖项 `<build_depend>`
3. 运行依赖项 `<run_depend>`：等价于 Format 2 中的 `<exec_depend>`
4. 测试依赖项 `<test_depend>`

一个 Format 1 的 package.xml 文件如下：

```xml
<package format = "2">
  <name> sensor_funsion </name>
  
  <version> 1.2.3 </version>
  
  <description>
      This package fusion point cloud and image.
  </description>
  
  <url> www.xxx.com </url>
  
  <author> DLonng </author>
  
  <maintainer email="xxx@xxx.com"> xxx </maintainer>
  
  <license> BSD </license>

  <buildtool_depend> catkin </buildtool_depend>

  <build_depend> message_generation </build_depend>
  <build_depend> roscpp </build_depend>
  <build_depend> std_msgs </build_depend>

  <run_depend> message_runtime </run_depend>
  <run_depend> roscpp </run_depend>
  <run_depend> rospy </run_depend>
  <run_depend> std_msgs </run_depend>

  <test_depend> python-mock </test_depend>
</package>
```

## 四、Format 1 转为 Format 2 格式

如果你目前项目使用的是 Format 1 语法，官方也建议转为 Format 2 语法，Format 1 转为 Format 2 格式只需要用新的标签替换含有相同值的旧标签。

### 4.1 package

Format 1：

```xml
<package>
  
</package>
```

Format 2：

```xml
<package format = "2">

</package>
```

### 4.2 depend

Format 1，注意两个都是 foo：

```xml
<build_depend> foo </build_depend>
<run_depend> foo </run_depend>
```

替换为 Format 2 的 depend：

```xml
<depend> foo </depend>
```

这个定义在 Format 2 中等价于对 foo 定义构建、导出和执行依赖：

```xml
<build_depend> foo </build_depend>
<build_export_depend> foo </build_export_depend>
<exec_depend> foo </exec_depend>
```

### 4.3 run_depend

在 Format 2 中删除了 `<run_depend>`，需要用构建和执行依赖重新替换。



Format 1：

```xml
<run_depend> foo </run_depend>
```

Format 2 中用以下 2 个标签替换：

```xml
<build_export_depend> foo </build_export_depend>
<exec_depend> foo </exec_depend>
```

如果只在运行时依赖 foo：

```xml
<exec_depend> foo </exec_depend>
```

如果只是导出依赖 foo：

```xml
<build_export_depend> foo </build_export_depend>
```

如果运行和导出都需要 foo，则使用 `depend` 标签代替以上 2 行：

```xml
<depend> foo </depend>
```

### 4.4 test_depend

Format 1 中的测试依赖项用 `<build_depend>`：

```xml
<build_depend> rostest </build_depend>
```

Format 2 中用 `<test_depend>` 代替：

```xml
<test_depend> rostest </test_depend>
```

### 4.5 doc_depend

Format 1 中定义的文档生成工具依赖，在 Format 2 中用：

```xml
<doc_depend> doxygen </doc_depend>
<doc_depend> epydoc </doc_depend>
<doc_depend> python-sphinx </doc_depend>
<doc_depend> rosdoc_lite </doc_depend>
```

## 五、总结

目前 ROS 官方建议使用 Format 2 的 package.xml 语法，Format 2  相比于 Format 1 的区别主要是增加了 `<depend>` 标签，方便依赖项的编写。

感谢阅读，多多实践 (*^▽^*)！


> {{ site.prompt }}

<div  align="center">
<img src="https://dlonng.com/images/wechart.jpg" width = "200" height = "200"/>