---
title: Effective C++ 读书笔记（二）
date : 2017-05-05 16:00:00
---

# Effective C++ 读书笔记（二）
***
> 版权声明：本文为 cheng-zhi 原创文章，可以随意转载，但必须在明确位置注明出处！ 

#### 设计与声明 - 条款 20

**宁以 pass-by-reference-to-const 替换 pass-by-value**

函数的参数如果是自定义的对象类型，则最好定义为 pass-by-reference-to-const ，内置类型(int，double ...)除外。

不建议下面的做法：

``` cpp
/* Student为自定义类型，但却直接按照默认的 pass-by-value 的方式传递 */
void function(Student s);
```
**缺点：**
1. 会导致自定义类型的构造函数和析构函数多次被调用，当自定义类型的构造函数和析构函数比较费时的时候，效率比较底下。
2. 在传递派生类时容易产生对象被切割的问题。

建议的做法：

``` cpp
/* 传递对象的引用 */
void function(const Student& s);
```

**优点：**
1. 可以回避自定义类型的构造和析构函数的调用，不会影响效率。
2. 不会产生对象切割问题，因为引用的底层其实就是指针，在内存中只有一份实例。

**原则：**
1. 如果可以的话，尽量传递 const 的引用作为函数的参数。
2. 不要将这个条款应用在内置类型上，对于内置类型，pass-by-value更适合。


#### 实现 - 条款 27

**尽量少做转型动作**

尽可能少的在代码中进行转型，如果避免不了，在 C++ 中应该抛弃旧式 C 类型的转型，而使用新式转型。

旧式转型

```
表达式风格：(T)expression
函数风格  ：T(expression)
```

新式转型

```cpp
/* static_cast比较常用 */
static_cast<T>(expression)
const_cast<T>(expression)
dynamic_cast<T>(expression)
reinterpret_cast<T>(expression)
```

新式转型示例

```cpp
int x = 1;

/* 旧式转型 */
double d1 = (double)x;

/* 新式转型 */
double d2 = static_cast<double> x;
```

**原则**
1. 尽量避免转型。
2. 将转型隐藏在接口内部，不要暴露给客户。
3. 抛弃旧式转型，使用新式转型。



