---
title: leveldb AtomicPointer
date : 2017-05-25 11:00:00
---

# leveldb AtomicPointer
***
> 版权声明：本文为 {{ site.name }} 原创文章，可以随意转载，但必须在明确位置注明出处！ 

## AtomicPointer 简介

`AtomicPointer` 是 `leveldb` 提供的一个原子指针操作类，使用了基于**内存屏障**的同步访问机制，这比用锁和信号量的效率要高。

源文件位置：`leveldb/port/atomic_pointer.h`

```cpp
class AtomicPointer {
  private:
    void* rep_;
  public:
    AtomicPointer() { }
    explicit AtomicPointer(void* p) : rep_(p) {}

    // 不使用内存屏障的读操作，即不同步的读操作
    inline void* NoBarrier_Load() const { return rep_; }

    // 同上，是不同步的写操作
    inline void NoBarrier_Store(void* v) { rep_ = v; }

    // 使用内存屏障的读操作，即同步读
    inline void* Acquire_Load() const {
      void* result = rep_;
      // 添加一个内存屏障，后面会有原理介绍
      MemoryBarrier();
      return result;
    }

    // 使用内存屏障的写操作，即同步写
    inline void Release_Store(void* v) {
      MemoryBarrier();
      rep_ = v;
    }
};

```

这是添加一个内存屏障的函数，**当这个函数之前的代码修改了某个变量的内存值后，其他 `CPU` 和缓存 (`Cache`) 中的该变量的值将会失效，必须重新从内存中获取该变量的值**。

```cpp
inline void MemoryBarrier() {
  __asm__ __volatile__("" : : : "memory");
}
```

总的来说，`AtomicPointer` 这个类是为了让**我们以更高的效率实现原子性的访问**。

## 什么是内存屏障 ?

内存屏障是同步的一种方法，类似于锁和信号量，但是它有更高的效率。内存屏障深入研究的水很深，这里只介绍它的基本用途和使用，而不会深入。

### 基本用途
#### 避免编译器优化指令
有些编译器默认会在编译期间对代码进行优化，从而改变汇编代码的指令执行顺序，如果你是在单线程上运行可能会正常，但是在多线程环境很可能会发生问题(如果你的程序对指令的执行顺序有严格的要求)。


而内存屏障就可以阻止编译器在编译期间优化我们的指令顺序，为你的程序在多线程环境下的正确运行提供了保障，但是不能阻止 CPU 在运行时重新排序指令。

#### 使得 CPU 和 Cache 可以「看见」内存
如果你想实现下面这样的功能，那你可以考虑内存屏障：


**修改一个内存中的变量之后，其余的 CPU 和 Cache 里面该变量的原始数据失效，必须从内存中重新获取这个变量的值**


这保证了这个变量对 CPU 和 Cache 是「可见的」，`leveldb` 就使用了这个特性

### 基本用法
#### Gcc
```
__asm__ __volatile__ ("" ::: "memory");
```

#### C++11
```
atomic_signal_fence(memory_order_acq_rel);
```

#### VC++
```
_ReadWriteBarrier();
```

#### Intel ECC compiler
```
__memory_barrier();
```

就介绍这个多，能力有限，如果对内存屏障有兴趣，可以参看下面的 `wiki`：

[Memory ordering](http://en.wikipedia.org/wiki/Memory_ordering)

