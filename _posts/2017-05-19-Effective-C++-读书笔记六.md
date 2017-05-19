---
yout:     	post
title:      "Effective C++ 读书笔记(六)"
subtitle:   "Effective C++"
date:       2017-05-19 16:00:00
author:     "陈登龙"
header-img: "img/post-bg-unix-linux.jpg"
catalog: true
tags:
    - C++
---

# Effective C++ 读书笔记(六)
***
> 版权声明：本文为 cheng-zhi 原创文章，可以随意转载，但必须在明确位置注明出处！ 

## 构造 / 析构 / 赋值运算 - 条款 - 05

**了解 C++ 默认编写并调用哪些函数**

C++ 在新建一个 `empty class` 并且你没有进行任何修改后会默认声明出下面的 4 个 `public` `inline` 函数：
```cpp
class Empty
{
public:
  // default 构造函数
  Empty() {}
  // default 析构函数
  ~Empty() {}
  // default copy 构造函数
  Empty(const Empty& rhs) {}
  // default 复制赋值操作符
  Empty& operator=(const Empty& rhs) {}
};
```
上面的等价于下面的默认实现：
```cpp
class Empty
{
};
```

**注意：** 上面的 4 个函数只是声明出来，但是只有当这些函数被**实际调用**时，它们才会被**创建**。
例如：
```cpp
{
  // e1 的 default 构造被创建并调用
  Empty e1;

  //离开作用域后，e1 的 default 析构函数被创建并调用
}

{
  // e2 的 default 构造被创建并调用
  Empty e2;

  // e1 的 copy 被创建并调用
  Empty e1(e2);
  // e2 的复制赋值操作符被创建并调用
  e2 = e1;
}
```

**注意：** 当你手动创建上面的 4 个函数之一时，编译器就不在提供缺省的那个函数了。例如你创建了自己的构造函数，default 构造就不会被创建了。

**记住：**
编译器默认为空类声明下面 4 个函数，并在实际调用他们时创建他们：
1. 缺省构造函数
2. 缺省析构函数
3. 缺省 `copy` 构造函数
4. 缺省复制赋值操作符


## 构造 / 析构 / 赋值运算 - 条款 - 07

**为多态基类声明 `virtual` 析构函数**


在开发中你可能会遇到下面情况的内存泄漏问题：

你的派生类对象由基类的指针被删除，但是基类的析构函数是 `non-virtual` 析构函数，结果导致你的**对象的派生属性成分没有被删除**，从而导致内存泄漏！
```cpp
class BaseClass
{
public:
  // non-virtual 析构函数
  ~BaseClass() {}
};
```


解决方法：**把基类的析构函数改为 `virtual` 析构函数。**
```cpp
class BaseClass
{
public:
  // virtual 析构函数
  virtual ~BaseClass() {}
};
```
之后你就可以正常的删除派生类对象，而不用担心内存泄漏的问题。

**何时应该使用 `virtual` 析构函数?**
当你的 `class` 带有 `virtual` 函数时，你都应该有一个 `virtual` 析构函数。因为你声明 `virtual` 函数的目的就是允许生成派生类，如果不使用 `virtual`函数就会引起上面的内存泄漏问题。

**virtual 析构函数在抽象类中的使用**

抽象类总是被当做基类使用，因此在抽象类中定义 `virtual` 析构函数也有必要。

当你想定义一个抽象类是，为你的抽象类声明一个**纯虚析构函数**，并**提供一份定义**：
```cpp
class AbstractClass
{
public:
  // 声明 pure-virtual 析构函数
  virtual ~AbstractClass() = 0;
}

// 因为派生类在析构函数中会调用基类的析构函数，所以必须提供定义
AbstractClass::~AbstractClass()
{
}
```

记住：
1. 如果你的 `class` 需要有派生类继承，那就要有一个 `virtual` 析构函数
2. 在抽象类中定义纯虚析构函数，需要提供一份**定义**



## About Me
- cheng-zhi：C / C++
- GitHub   ：[cheng-zhi](https://github.com/cheng-zhi)
- 个人主页 ：[cheng-zhi](https://cheng-zhi.github.io/)
- 微博     ：@cheng-zhi
- 微信号   ：chengzhi-01
- 微信公众号：记录我的成长与分享，欢迎扫码关注，id：growingshare

![ID:growingshare](https://cheng-zhi.github.io/img/wechart.jpg)


