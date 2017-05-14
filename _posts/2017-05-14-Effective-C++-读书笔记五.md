---
yout:     	post
title:      "Effective C++ 读书笔记(五)"
subtitle:   "Effective C++"
date:       2017-05-14 15:00:00
author:     "陈登龙"
header-img: "img/post-bg-unix-linux.jpg"
catalog: true
tags:
    - Effective C++
---

# Effective C++ 读书笔记(五)
***
> 版权声明：本文为 cheng-zhi 原创文章，可以随意转载，但必须在明确位置注明出处！ 

#### 让自己习惯 C++ - 条款 03

**尽可能的使用const**

`const` 允许你指定一个语义约束：**指定一个对象不可被改变**，下面是 const 的几种常见的使用方法：

##### const 修饰指针
```cpp
	char greeting[] = "Hello";
	// non-const pointer, non-const data
	char *p = greeting;
	// non-const pointer, const data
	const char *p = greeting;
	// const pointer, non-const data
	char *p const = greeting;
	// const pointer, const data
	const char * const p = greeting;
	// const iter, non-const data
	const std::vector<int>::iterator iter;
	// non-const iter, const data
	std::vector<int>::const_iterator iter;
```

##### const 修饰函数 

- const 修饰函数返回值

```cpp
	// 将函数的返回值声明为 const， 可以防止返回值被恶意赋值
	const MyType function();
```
- const 修饰函数参数

```cpp
	// 表示参数 x 在函数内部不能被改变
	MyType function(const int x);
```

- const 修饰成员函数

```cpp
	// 该成员函数可以操作 const 对象
	MyType function(const int x) const;

	// 两个成员函数如果常量性不同，可以被重载
	char& operator[](std::size_t position) const;// 针对 const 对象
	char& operator[](std::size_t position); // 针对 non-const 对象
```

#### 如何取消 const 属性？

使用 **mutable** 去掉 const 属性。

```cpp
	// 这个成员变量可以在 const 成员函数内被更改
	mutable bool flag;
```

#### 如何在 `const` 函数和 `non-const` 函数之间避免代码重复？

如果 `const` 函数和 `non-const` 函数的函数定义基本相同，我们可以使用 `non-const` 函数来间接调用 `const` 函数来减少重复的代码，这其中涉及到两次 `const` 转型。

```cpp
	// const 函数
	const char& operator[](std::size_t position) const
	{
		...
	}
	
	// non-const 函数
	char& operator[](std::size_t position)
	{
		// 1. 将 *this 转换成 const ClassName& 类型，使得后面的 [position] 得以调用到 const 版本的函数
		// 2. 从 const operator[] 中移除 const 属性。
		return const_cast<char&>(static_cast<const ClassName&>(*this)[position]);
	}
	
```

原则：
1. 将事实上确实不可变的东西声明为 `const`，可以帮助编译器侦测出错误用法。
2. 当 `const` 和 `non-const` 成员函数的定义等价时，让 `non-const` 调用 `const` 可以避免代码重复。

> 尽可能的使用 const

#### About Me
* cheng-zhi：C / C++
* GitHub：[cheng-zhi](https://github.com/cheng-zhi)
* 个人主页：[cheng-zhi](https://cheng-zhi.github.io/)

