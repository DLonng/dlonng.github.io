---
title: 一些经典的 C++ 笔试和面试题
date: 2019-04-14 16:00:00
---
# 一些经典的 C++ 笔试和面试题
***
> 版权声明：本文为 {{ site.name }} 原创文章，可以随意转载，但必须在明确位置注明出处！

### 1、C++ 中，有哪 4 个与类型转换相关的关键字？各有何特点，应在什么场合下使用？
- static_cast：静态类型转换，主要用于基类和子类之间转换，基本数据类型转换，空指针类型转换，任何类型转换成 void 型。
- const_cast：去掉类型的 const 或 volatile 属性。
- dynamic_cast：动态类型转换，用于安全的基类和子类之间转换，必须要有虚函数，转换失败结果为 NULL。
- reinterpret_cast：仅仅重新解释类型，但没有进行二进制的转换，最普通的用途是转换函数指针。

### 2、sizeof(空类型) = ？
等于 1，空类型不包含任何信息，本应是 0，但声明该实例后在内存中要占有一定的空间，占用的空间由编译器决定，VS 中空类型占 1 个字节。

如果在该类型中加上构造和析构函数呢？

还是 1，调用函数只需知道函数地址，但函数地址只与类型相关，而与类型的实例无关，编译器不会在实例内添加额外信息。

如果把析构函数标记为虚函数呢？

C++ 会为含有虚函数的类生成虚函数表，并在该类型的每一个实例中添加一个指向虚函数表的指针，在 32 位机器上，指针占 4B，所以 sizeof 得到 4，64 位机器的指针占 8B，所以 sizeof 得到 8

### 3、拷贝构造函数
```cpp
class A {
private:
  int value;

public:
  A(int n) {value = n;}
  // C++ 不允许复制构造函数进行值传递，否则会产生递归调用，导致栈溢出
  //A(A other) {value = other.value;}
  A(const A &other) {value = other.value;}
  void Print() {std::cout << value << std::endl;}
};
```

### 4、常引用问题
```cpp
string foo();
void bar(string &s);

bar(foo());
bar("hello world");
```

`foo()` 和 `hello world` 都会产生一个临时对象，而在 C++ 中临时对象都是 const 类型，所以上面的 2 个函数调用都试图将 const 类型转换为非 const 类型，非法！



### 5、编写拷贝构造函数
```cpp
class CMyString {
public:
  CMyString(char *pData = NULL);
  CMyString(const CMyString& str);
  ~CMyString();
private:
  char *m_pData;
};
```
经典代码：
```cpp
CMyString& CMyString::operator=(const CMyString &str) {
  if (this == &str)
    return *this;

  delete []m_pData;
  m_pData = NULL;
  m_pData = new char[strlen(str.m_pData) + 1];
  strcpy(m_pData, str.m_pData);

  return *this;
}
```



### 6、实现 Singleton 单例模式
设计一个类，我们只能生成该类的一个实例。

```cpp
class S {
public:
  static S& getInstance() {
    static S instance;
    return instance;
  }

private:
  S() {};
  S(S const&);
  void operator=(S const&);
};
```
实现机制：静态局部变量在第一次使用时初始化，并不会销毁直到程序退出。



> {{ site.prompt }}

<div  align="center">
<img src="{{ site.url }}/images/wechart.jpg" width = "200" height = "200"/>
