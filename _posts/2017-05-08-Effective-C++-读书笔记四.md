---
title: Effective C++ 读书笔记（四）
date : 2017-05-08 21:00:00
---

# Effective C++ 读书笔记（四）


***
> 版权声明：本文为 cheng-zhi 原创文章，可以随意转载，但必须在明确位置注明出处！ 

## 让自己习惯 C++ - 条款 02

**尽量以 const，enum，inline 替换 #define**

不建议的做法
```cpp
#define NUM 1
```

建议的做法
```cpp
const int num = 1;
```

当你需要在一个 class 中使用常量作为数组的大小时，万一你的编译器不允许你使用 **static 整数型常量完成 in class 的初始值设定**，此时你可以使用 enum，例如：
```cpp
class MyType
{
private:
	enum { num = 10};
	int arrays[num];
};
```

在 C++ 中不要使用 #define 来定义宏函数，不要像下面这样
```cpp
#define MAX(a, b) f((a) > (b) ? (a) : (b))
```

你应该使用 inline 函数替换 #define 
```cpp
template<typename T>
inline void my_max(const T& a, const T& b)
{
	f(a > b ? a : b);
}
```

**原则**
1. 使用 const 来定义单个常量。
2. 使用 inline 来定义形式函数的宏。


## 模板与泛型编程 - 条款 42

**了解 typename 的双重意义**

第一层：作为类模板的参数时，与 class 功能相同。
```cpp
template<class T> class MyType;
template<typename T> class MyTYpe;
```
这两个定义完全相同。

第二层： typename 可以让模板里面定义嵌套从属名称的类型变成有效的类型，因为 C++ 的解析器在模板中遇到嵌套从属类型时，默认认为它是无效的类型。

例如：无效的嵌套从属类型
```cpp
template<typename T>
void fun(const T& t)
{
	T::const_iterator iter(t.begin());
}
```
我们需要认为指定它为有效的嵌套从属类型
```cpp
template<typename T>
void fun(const T& t)
{
	typename T::const_iterator iter(t.begin());
}
```
**一般情况**
当你想在 template 中指定一个有效的嵌套从属类型名称，只需要在嵌套从属类型前面加上 typename 关键字即可。

**例外**
* 不得在 base class list 中使用 typename
```cpp
/* 错误用法 */
class Deriver : public typename Base<T>::MyType;
```
* 不得在 member initialization list 中使用 typename
```cpp
class Deriver : public Base<T>::MyType
{
public:
	/* 错误用法 */
	explicit Deriver(int x) : typename Base<T>::MyType(x)
	{
	}
};
```

**原则**
1. class 和 typename 在声明模板参数时作用相同。
2. 使用 typename 标识嵌套从属类型，但是不得在 base class list 和 member initialization list 中使用。

