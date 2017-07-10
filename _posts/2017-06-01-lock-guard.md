---
title: std::lock_guard 引起的思考
date: 2017-06-01 12:00:00
---

# std::lock_guard 引起的思考
***
> 版权声明：本文为 {{ site.name }} 原创文章，可以随意转载，但必须在明确位置注明出处！ 

## 从哪里来的思考？
最近在项目总结过程中，发现项目大量使用了 `std::lock_guard` 这个模板类，仔细分析后发现这个类牵扯到了很多重要的计算机基础，例如：多线程，互斥，锁等等，这里便记录下来，也算是一次简单的总结。


## std::lock_guard 简介
这个类是一个互斥量的包装类，用来**提供自动为互斥量上锁和解锁**的功能，简化了多线程编程，用法如下：
```cpp
#include <mutex>

std::mutex kMutex;

void function() {
  // 构造时自动加锁
  std::lock_guard<std::mutex> (kMutex);
  
  // 离开局部作用域，析构函数自动完成解锁功能
}
```
用法非常简单，只需在保证线程安全的函数开始处加上一行代码即可，其他的都在这个类的构造函数和析构函数中自动完成。


如何自动完成？其实 Just so so ...


## 实现 my_lock_guard
这是自己实现的一个 `lock_guard`，就是在**构造和析构中完成加锁和解锁**的操作，之所以会自动完成，是因为**离开函数作用域会导致局部变量析构函数被调用**，而我们又是手动构造了 `lock_guard`，因此这两个函数都是自动被调用的。

```cpp
namespace myspace {
	template<typename T> class my_lock_guard {
	public:
		// 在 std::mutex 的定义中，下面两个函数被删除了
		// mutex(const mutex&) = delete;
		// mutex& operator=(const mutex&) = delete;
		// 因此这里必须传递引用
		my_lock_guard(T& mutex) :mutex_(mutex){
			// 构造加锁
			mutex_.lock();
		}

		~my_lock_guard() {
			// 析构解锁
			mutex_.unlock();
		}
	private:
		// 不可赋值，不可拷贝
		my_lock_guard(my_lock_guard const&);
		my_lock_guard& operator=(my_lock_guard const&);
	private:
		T& mutex_;
	};

};

```

要注意的是这个类官方定义是**不可以赋值和拷贝**，因此需要私有化 `operator =` 和 `copy` 这两个函数。

## 什么是 std::mutex ？
如果你细心可以发现，不管是 `std::lock_guard`，还是`my_lock_guard`，都使用了一个 `std::mutex` 作为构造函数的参数，这是因为我们的 `lock_guard` 只是一个包装类，而实际的加锁和解锁的操作都还是 `std::mutex` 完成的，那什么是 `std::mutex` 呢？

`std::mutex` 其实是一个**用于保护共享数据不会同时被多个线程访问的类**，它叫做**互斥量**，你可以把它看作一把锁，它的基本使用方法如下：
```cpp
#include <mutex>

std::mutex kMutex;

void function() {
  //加锁
  kMutex.lock();
  //kMutex.try_lock();

  //do something that is thread safe...
  
  // 离开作用域解锁
  kMutex.unlock();
}
```

前面都提到了**锁**这个概念，那么什么是锁，有啥用处？


## 什么是锁？
**锁是用来保护共享资源（变量或者代码）不被并发访问的一种方法**，它只是方法，实际的实现就是 `std::mutex` 等等的类了。

可以简单的理解为：

1. 当前线程访问一个变量之前，将这个变量放到盒子里锁住，并且当前线程拿着钥匙。这样一来，如果有其他的线程也要访问这个变量，则必须等待当前线程将盒子解锁之后才能访问，之后其他线程在访问这个变量之前也将会再次锁住这个变量。

2. 当前线程执行完后，就将该盒子解锁，这样其他的线程就可以拿到盒子的钥匙，并再次加锁访问这个变量了。

这样就**保证了同一时刻只有一个线程可以访问共享资源**，解决了简单的线程安全问题。


什么，你还没有遇到过线程安全问题？下面开始我的表演...

## 一个简单的线程安全的例子
这个例子中，**主线程开启了 2 个子线程**，每个子线程都修改**共享的全局变量** `kData`，如果没有增加必要的锁机制，那么每个子线程打印出的 `kData` 就可能会出错。

这里使用了 3 种不同的加锁方法来解决：
1. 使用 `std::lock_guard`
2. 使用 `std::mutex` 实现原生的加锁
3. 使用自己的 `myspace::my_lock_guard`

```cpp
#include <iostream>
#include <mutex>
#include <thread>

// 两个子线程共享的全局变量
int kData = 0;

// std::mutex 提供了一种防止共享数据被多个线程并发访问的简单同步方法
// 调用线程可以通过 lock 和 try_lock 来获取互斥量，使用 unlock() 释放互斥量
std::mutex kMutex;


void increment() {
	// 1.创建一个互斥量的包装类，用来自动管理互斥量的获取和释放
	// std::lock_guard<std::mutex> lock(kMutex);
	
	// 2.原生加锁
	// kMutex.lock();

	// 3.自己实现的 std::mutex 的包装类
	myspace::my_lock_guard<std::mutex> lock(kMutex);
	
	for (int i = 0; i < 10; i++) {
		// 打印当前线程的 id : kData
		std::cout << std::this_thread::get_id() 
		          << ":" << kData++ << std::endl;
	}
	
	// 2. 原生解锁	
	//kMutex.unlock();
	
	// 离开局部作用域，局部锁解锁，释放互斥量
	
}


int main()
{
	// 打印当前函数名
	std::cout << __FUNCTION__ << ":" << kData << std::endl;

	// 开启两个线程
	std::thread t1(increment);
	std::thread t2(increment);

	// 主线程等待这两个线程完成操作之后再退出
	t1.join();
	t2.join();

	// 防止立刻退出
	getchar();
	return 0;

}
```


注意：在 `vs` 中编译这段代码。

#### 结果分析
为什么不加锁的结果会出错？

首先线程是一种轻量级的进程，也存在调度，假设当前 `CPU` 使用的是**基于时间片的轮转调度算法**，为每个进程分配一段可执行的时间片，因此每个线程都得到一段可以执行的时间（这里只是简单概括，仔细研究其实是有点复杂的，涉及到内核线程和用户线程，这里就不多说了，不是这里讨论的重点），这就导致子线程 1 在修改并打印 `kData` 的时候，子线程 1 的时间片用完了，`CPU` 切换到子线程 2 去修改并打印 `kData`，这就导致了最终的打印结果不是预先的顺序，就是这个原理，简单的理解是不难的。 


能力有限，就总结这些吧。
