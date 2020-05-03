---
title: ROS 机器人技术 - 解析 CMakeList.txt 文件
date: 2020-05-03 19:46:00
---
# ROS 机器人技术 - 解析 CMakeList.txt 文件
***
> 版权声明：本文为 {{ site.name }} 原创文章，可以随意转载，但必须在明确位置注明出处！

编写 ROS 节点还需要配置构建规则，CMakeList.txt 文件就是用来指定如何编译当前节点，里面包含一些编译指令，今天就来学习下常用的配置指令，不需要把 CMakeList 全部的内容都学会，先把基础的搞定，后面用到再学即可。

## 一、CMakeList 的作用

ROS 项目使用 CMake 来构建的，为了方便一次性构建，所以用一个 CMakeList.txt 文件作为 CMake 系统的输入，通过读取这个文件中的构建命令，来完成对项目的构建。

比如一个简单的 CMakeList.txt 如下：

```cmake
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(OSeg)

set( CMAKE_EXPORT_COMPILE_COMMANDS ON )
set( CMAKE_CXX_FLAGS "-std=c++11")

find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(o_seg o_seg.cpp)

target_link_libraries (o_seg ${PCL_LIBRARIES})
```

## 二、常用的命令

自己编写 CMakeList 文件必须使用下面的格式，顺便不是最重要的，不过按照以下的顺序会更好阅读：

1. CMake 版本：`cmake_minimum_required()`
2. 要构建项目的名称：`project()`
3. 查找其他的需要构建的 CMake 或者 Catkin 包：`find_package()`
4. 开启 Python 模块支持：`catkin_python_setup()`
5. 添加自定义 Message、Service、Action 文件：`add_message_files`、`add_service_files()`、`add_action_files()`
6. 生成自定义 Message、Service、Action：`generate_message()`
7. 指定包的构建信息输出：`catkin_package()`
8. 指定生成的可执行文件：`add_executable()`
9. 指定生成可执行文件需要链接的库：`target_link_libraries()`
10. 指定编译生成库文件：`add_library()`
11. 指定测试规则：`catkin_add_gtest()`
12. 指定安装规则：`install()`

下面详细介绍里面常用的命令。

## 三、常见命令用法

### 3.1 CMake 版本

CMakeList 文件的第一行指定使用的 CMake 最低版本：

```cmake
cmake_minimum_required(VERSION 2.8.3)
```

### 3.2 项目名称

```cmake
project(pkg_name)
```

### 3.3 查找依赖包

查找项目构建需要的依赖包：

```cmake
find_package(catkin REQUIRED)
```

如果还需要依赖其他的包（比如 nodelet），可以直接将包作为组件（COMPONENTS）加到上面的命令后面、为了节省时间：

```cmake
find_package(catkin REQUIRED COMPONENTS nodelet)
```

当然也可以分开写，与上面等价：

```cmake
find_package(catkin REQUIRED)
find_package(nodelet REQUIRED)
```

### 3.4 catkin_package

这个命令是 Catkin 提供的一个宏，用于把 Catkin 的构建信息输出到构建系统中，用于生成 package 配置文件和 CMake 文件。

这个命令必须在 `add_library()` 和 `add_executable()` 前调用，它有 5 个可选的参数：

- `INCLUDE_DIRS`：导出包的头文件路径
- `LIBRARIES`：导出项目的库
- `CATKIN_DEPENDS`：依赖的 Catkin 包
- `DEPENDS`：依赖的非 Catkin 包
- `CFG_EXTRAS`：其他额外的配置信息

一个例子如下：

```cmake
catkin_package(
   # include 文件夹是导出头文件的地方
   INCLUDE_DIRS include
   # 导出的库使用包的名称
   LIBRARIES ${PROJECT_NAME}
   # 构建项目依赖的 catkin 包，roscpp 和 nodelet
   CATKIN_DEPENDS roscpp nodelet
   # 构建项目依赖的非 catkin 包
   DEPENDS eigen opencv)
```

### 3.5 指定构建目标规则

目标的构建主要有 2 中：

- 构建可执行目标
- 构建库目标

#### 3.5.1 目标名称

注意，Catkin 中的构建目标的名称必须唯一，无论它在哪个目录中构建/安装，使用以下命令更改构建目标名称 rviz_image_view 为 image_view：

```cmake
set_target_properties(rviz_image_view
                      PROPERTIES OUTPUT_NAME image_view
                      PREFIX "")
```

#### 3.5.2 头文件路径

比如添加本项目的 include 文件夹，Boost 头文件 和 catkin 头文件：

```cmake
include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
```

#### 3.5.3 库文件路径（不推荐使用）

在 ROS 中不推荐使用 link_directories 命令添加库文件路径，因为之前的 find_package 命令会自动添加依赖的库文件路径，也可以使用 target_link_libraries 命令添加，如果非要使用这个也行：

```cmake
link_directories(~/my_libs)
```

#### 3.5.4 可执行目标

指定构建的可执行目标名称和源文件：

```cmake
add_executable(myProgram src/main.cpp src/some_file.cpp src/another_file.cpp)
```

#### 3.5.5 构建共享库

默认情况 Catkin 输出共享库：

```cmake
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS})
```

#### 3.5.6 链接目标库文件

通常在 `add_executable` 后面使用链接库文件命令：

```cmake
add_executable(foo src/foo.cpp)
# 链接共享库 libmoo.so 到可执行文件 foo
target_link_libraries(foo moo)
```

### 3.6 配置消息、服务和响应

ROS 中的消息 msg、服务 srv、响应 action 在构建之前需要使用一个特殊的预处理构建宏，以此来生成编程语言需要的文件：

- 处理消息：`add_message_files(FILES xxx.msg)`
- 处理服务：`add_service_files(FILES xxx.srv)`
- 处理响应：`add_action_files(FILES xxx.action)`

添加了处理宏后还需要写上一行生成命令，以此在 devel 目录下的 include 文件夹中生成头文件：

```cmake
generate_messages(DEPENDENCIES xxx)
```

#### 3.6.1 配置步骤

将处理宏添加到 catkin_package 之前：

```cmake
find_package(catkin REQUIRED COMPONENTS ...)

add_message_files(...)
add_service_files(...)
add_action_files(...)

generate_messages(...)

catkin_package(...)
```

然后在 catkin_package 命令中添加 message_runtime 依赖：

```cmake
catkin_package(
   ...
   CATKIN_DEPENDS message_runtime
   ...
 )
```

在 find_package 中添加 message_generation 组件：

```cmake
find_package(catkin REQUIRED COMPONENTS 
             ...
             message_generation)
```

#### 3.6.2 注意 2 点

如果当前构建的目标依赖其他需要构建消息、服务、响应的对象，则需要向目标 `catkin_EXPORTED_TARGETS` 中添加明确的依赖关系，确保构建顺序：

```cmake
# some_target 是 add_executable 设置的构建目标名称
add_dependencies(some_target ${catkin_EXPORTED_TARGETS})
```

如果您有一个用于构建消息或服务的程序包以及使用它们的可执行文件，则需要在自动生成的消息目标上创建显式依赖项，以便以正确的顺序构建它们：

```cmake
add_dependencies(some_target ${${PROJECT_NAME}_EXPORTED_TARGETS})
```

如果你的包满足以上 2 点，则需要添加 2 个依赖：

```cmake
add_dependencies(some_target ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
```

#### 3.6.3 一个例子

我们有一个功能包定义了 2 个自定义消息 和 1 个自定义服务：

- MyMessage1.msg：依赖 std_msgs 和 sensor_msgs
- MyMessage2.msg：依赖 std_msgs 和 sensor_msgs
- MyService.srv

CMakeList 需要构建 2 个可执行目标：

- message_program：依赖以上 msg 和 srv

- does_not_use_local_messages_program：不依赖以上 msg 和 srv

则我们的 CMakeList.txt 编写如下：

```cmake
# 声明构建依赖项
find_package(catkin REQUIRED
             COMPONENTS 
             message_generation 
             std_msgs 
             sensor_msgs)

# 声明要构建的消息文件
add_message_files(FILES
                  MyMessage1.msg
                  MyMessage2.msg)

# 声明要构建的服务文件
add_service_files(FILES
                  MyService.srv)

# 添加依赖项，生成上面定义的消息和服务
generate_messages(DEPENDENCIES
                  std_msgs 
                  sensor_msgs)

# 声明运行时依赖项
catkin_package(CATKIN_DEPENDS 
               message_runtime 
               std_msgs 
               sensor_msgs)

# 声明构建使用消息和服务的可执行文件 message_program
add_executable(message_program src/main.cpp)
add_dependencies(message_program ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 声明构建不使用消息和服务的可执行文件 does_not_use_local_messages_program
add_executable(does_not_use_local_messages_program src/main.cpp)
add_dependencies(does_not_use_local_messages_program ${catkin_EXPORTED_TARGETS})
```

#### 3.6.4 添加响应 action

action 的添加特殊一些，需要添加一个 `actionlib_msg`，步骤如下：

- 创建 MyAction.action 文件

- 添加 actionlib_msg 组件到 find_package

  ```cmake
  find_package(catkin REQUIRED
               COMPONENTS 
               ...
               actionlib_msg)
  ```

- 在 CMakeList.txt 中 add_action_files

  ```cmake
  add_action_files(FILES
                   MyAction.action)
  ```

- 添加 actionlib_msg 到 generation_message

  ```cmake
  generate_messages(DEPENDENCIES
                    ...
                    actionlib_msg)
  ```

- 添加 actionlib_msg 到 catkin_package

  ```cmake
  catkin_package(CATKIN_DEPENDS 
                 ...
                 actionlib_msg)
  ```

  

## 四、其他命令

这部分暂时没有用到，先记录下来吧。

### 4.1 启动 Python 模块支持

如果 ROS 项目中使用了 Python 代码，要创建一个 `setup.py` 然后在 `genreate_message` 和 `catkin_package` 调用前启动 Python 支持：

```cmake
catkin_python_setup()
```

### 4.2 单元测试

使用以下配置来启用 gtest 的支持：

```cmake
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(myUnitTest test/utest.cpp)
endif()
```

## 五、安装命令

这部分暂时也没使用到，先记录下来，以后用到再更新。

`install` 命令用来指定编译的文件被安装到哪些位置，这个命令有 3 个参数来指定不同类型文件的安装位置：

- `TARGETS`：要安装的目标
- `ARCHIVE DESTINATION`：静态链接库安装位置
- `LIBRARY DESTINATION`：非 DLL 共享库和模块安装位置
- `RUNTIME DESTINATION`：可执行文件和共享库安装位置

### 5.1 常见用法

用法如下：

```cmake
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
```

指定可执行文件安装位置：

```cmake
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

### 5.2 特殊 Python 模块

如果有特殊 Python 模块的库则需要指定特殊的安装位置：

```cmake
install(TARGETS python_module_library
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)
```

### 5.3 指定 Python 脚本安装位置

```cmake
catkin_install_python(PROGRAMS scripts/myscript
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
```

### 5.4 安装头文件

```cmake
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
```

### 5.5 安装 roslaunch 和其他文件

launch 和其他文件安装到 `CATKIN_PACKAGE_SHARE_DESTINATION`：

```cmake
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
```



以上这些命令规则不用死记硬背，多在项目中使用就会了。


> {{ site.prompt }}

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/dlonng_qrcode.jpg#pic_center)