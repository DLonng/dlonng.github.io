---
layout:     post
title:      "Effective C++ 读书笔记(三)"
subtitle:   "Effective C++"
date:       2017-05-06 19:00:00
author:     "陈登龙"
header-img: "img/post-bg-unix-linux.jpg"
catalog: true
tags:
    - C++
---

# Effective C++ 读书笔记(三)
***
> 版权声明：本文为 cheng-zhi 原创文章，可以随意转载，但必须在明确位置注明出处！ 

## 构造/析构/赋值 - 条款 10

**令 operator= 返回一个 reference to *this **

这个条款可以实现变量的**连锁赋值**问题，它被 C++ 所有内置类型和标准库的类型所遵守，我们也应当遵守。

你的自定义类型的 operator= 应该定义成这样：

``` cpp
MyType& operator=(const MyType& rhs)
{
	//...
	return *this;
}

```


**原则**

* 让 operator= 返回 reference to *this。


## 继承与面向对象设计 - 条款 34

**区分接口继承和实现继承**

在编写自己的 class 时，你应该明白提供下面 3 种类型函数的理由
1. pure virtual function
2. virtual function
3. no-virtual function


何时提供 pure virtual function？

**要求派生类只继承接口时，提供纯虚函数。**

```cpp
class MyType
{
public:
	/* 派生类只继承接口 */
	virtual void fun() const = 0; 
};
```

何时提供 virtual function？

**要求派生类继承接口和缺省实现时，提供虚函数。**

```cpp
class MyType
{
public:
	/* 派生类继承接口和缺省实现 */
	virtual void fun() const
	{
		//default code
	}
};
```

何时提供 no-virtual function？

**要求派生类继承接口的强制实现时，提供非虚函数。**

```cpp
class MyType
{
public:
	/* 派生类继承接口的强制实现 */
	void fun() const
	{
		//default code
	}
};
```


**原则**
1. 纯虚函数指定接口继承。
2. 虚函数指定接口和缺省实现继承。
3. 非虚函数指定接口的强制实现继承。
4. 接口继承和实现继承不同。


## About Me
* cheng-zhi：C / C++
* Github：[cheng-zhi](https://github.com/cheng-zhi)
* 个人主页：[cheng-zhi](https://cheng-zhi.github.io/)



