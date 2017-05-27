---
title: Message Bus
date : 2017-05-23 18:00:00
---

# Message Bus
***
> 版权声明：本文为 cheng-zhi 原创文章，可以随意转载，但必须在明确位置注明出处！ 

## 什么是 C++11 消息总线 ?
最近在 `C++` 项目中需要处理对象之间大量的消息，如果使用传统的 `SendMessage` 和 `PostMessage` 会使得对象之间的耦合程度过高，因此我们最后采用了一个基于 `C++11` 的**消息总线库** `Message Bus`，它有下面的一些优点：
1. 使用简单，发送和接收只需要一条语句，并使用 `lambda` 来简化函数的回调
2. 可以使得模块之间高度解偶，这依赖于一个全局的 `MessageBus` 对象
3. 支持发送自定义消息或者数据包
4. 支持发送有参和无参消息

## 基本原理
你可以将 `Message Bus` 理解为一个**全局的消息通道**，你可以将自己的加上**标识字的消息**放到这个通道上，别的模块就可以通过这个标识字来拿到你的消息，这些都依赖与一个全局对象 `MessageBus g_bus`，它就充当了全局消息通道。

## 基本使用方法
#### 发送和接受无参消息
发送：
```cpp
// g_bus 是一个全局变量
extern MessageBus g_bus;

// 发送一个带有 `InitAll` 标识字的无参数消息
g_bus.SendReq<void>("InitAll");
```

接收：
```cpp
// g_bus 是一个全局变量
extern MessageBus g_bus;

// 绑定带有 `InitAll` 标识字的消息，一般在构造函数中进行.
// 一收到消息，InitAll 函数就会被调用，使用了简单粗暴 lambda 表达式
g_bus.Attach([this](){ InitAll(); }, "InitAll");

// 解除绑定带有 `InitAll` 的消息，一般在析构函数中进行
g_bus.Remove<void>("InitAll");
```

#### 发送和接受有参消息
发送：
```cpp
// g_bus 是一个全局变量
extern MessageBus g_bus;

YourType m_yourType;
// 发送一个带有 `YourType` 类型参数的有参消息
g_bus.SendReq<void, const YourType&>(m_yourType, "YourType");
```

接收：
```cpp
// g_bus 是一个全局变量
extern MessageBus g_bus;

// 绑定带有 `YourType` 标识字的有参消息，一般在构造函数中进行.
// 一收到消息，you_function 就会被调用，并且发送时候传递的 `m_yourtype` 参数传递到这里的 `your_type`，最终传递到 `you_function`，是不是非常简单。
g_bus.Attach([this](const YourType& your_type){ you_function(your_type); }, "YourType");

// 解除绑定 `YourType` 标识的消息，一般在析构函数中进行
g_bus.Remove<void, const YourType&>("YourType");
```

