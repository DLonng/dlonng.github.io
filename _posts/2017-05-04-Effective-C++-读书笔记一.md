---
title: Effective C++ 读书笔记（一）
date : 2017-05-04 21:00:00
---

# Effective C++ 读书笔记（一）
***
> 版权声明：本文为 cheng-zhi 原创文章，可以随意转载，但必须在明确位置注明出处！ 

#### 资源管理 - 条款 16
* 成对使用 new 和 delete 时要采取相同形式

错误的做法： 

``` cpp
int *p;
delete []p;

std::string *stringArray = new std::string[100];
delete stringArray;
```

正确的做法：
``` cpp
int *p;
delete p;

std::string *stringArray = new std::string[100];
delete []stringArray;
```
原则：
* 如果在 new 中使用了 []，必须在 delete 时加上 []，否则绝对不要加 [] 。
* 当你的指针不是原始的简单数据类型时，delete 时加上 [] 。

#### 设计与声明 - 条款 22
* 将成员变量声明为 private

错误的做法：
``` cpp
class Test {
public:
	int var;
};
```
正确的做法：
``` cpp
class Test {
private:
	int var;
};
```
优势：
* 将成员变量声明为 private 可以让客户保证访问该数据只能通过指定的接口，而 class 的作者则可以灵活的改变接口实现，但是不会影响用户。


#### 设计与声明 - 条款 23
* 宁以 non - member，non-friend 替换 member 函数

不好的做法：
``` cpp
class Test {
public:
	void clear_a();
	void clear_b();
	void clear_c();
public:
	/* member fun */
	void clear_all() 
	{
		clear_a();
		clear_b();
		clear_c();
	}
};
```
优秀的做法：
``` cpp
namespace TestSpace {
	class Test {
	public:
		void clear_a();
		void clear_b();
		void clear_c();
	};
	
	/* non-member，non-friend fun */
	void clear_all(Test& t)
	{
		t.clear_a();
		t.clear_b();
		t.clear_c();
	}
}
```
原则：
当你在一个 class 中定义的一个函数**可以重新定义**为 non-member，non-friend 函数时，请将它定义成 non-member，non-friend，因为这样可以提高这个 class 的封装程度，这虽然不符合面向对象的守则，但是这个一个例外，因为这种方法比使用原始的守则封装度更好，你应该对你的代码更好，不是吗？

