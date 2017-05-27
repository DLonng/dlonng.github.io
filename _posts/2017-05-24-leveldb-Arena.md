---
title: leveldb Arena 分析
date : 2017-05-24 18:00:00
---

# leveldb Arena 分析
***
> 版权声明：本文为 cheng-zhi 原创文章，可以随意转载，但必须在明确位置注明出处！ 

## Arena
`Arena` 是 `leveldb` 项目里面使用的轻量级的**内存池对象**，`leveldb` 用这个对象来管理内存的分配，简化了 `new` 和 `delete` 的调用，我们也可以从这个轻量级的内存池对象学习 `google` 大神工程师是如何管理内存的。


## Arena 内存管理模型
这是[罗道文](http://luodw.cc)网站上关于 `leveldb` 的一张 `Arena` 的内存模型图:

![Arena 内存池模型](https://cheng-zhi.github.io/img/C++/leveldb/Arena.jpg)

`Arena` 使用下面几个成员变量来描述上面的模型图
```cpp
// 当前内存块未分配内存的起始地址
char* alloc_ptr_;

// 当前内存块剩余的内存
size_t alloc_bytes_remaining_;

// Arena 使用 vector 来存储每个内存块的地址
std::vector<char*> blocks_;

// 当前 Arena 已经分配的总内存量 
port::AtomicPointer memory_usage_;
```

在开始分析之前你需要了解**申请内存**和**分配内存**的区别：
1. 申请内存：使用 `new` 来向操作系统申请一块连续的内存区域。
2. 分配内存：将已经申请的内存分配给项目组件使用，这体现在增加 `alloc_ptr_` 和减少 `alloc_bytes_remaining_` 这两个指针上。

为什么要强调这两个概念呢？


因为 `Arena` 是一个内存池，他的功能就是内存的管理，包括**申请内存，分配内存，释放内存**。


## Arena 的构造和析构

`Arena` 的源码位置：`/leveldb/util/arena.h`, `/leveldb/util/arena.cc`

```cpp
// 良好的初始化风格
Arena::Arena() : memory_usage_(0) {
  alloc_ptr_ = NULL;  
  alloc_bytes_remaining_ = 0;
}

Arena::~Arena() {
  // 分别释放 vector 中每个指针指向的内存块
  for (size_t i = 0; i < blocks_.size(); i++) {
    delete[] blocks_[i];
  }
}
```


## Arena 提供的接口函数
`Arena` 给我们提供了 3 个 `public` 函数来简化我们的内存访问
```cpp
public:
  // 基本的内存分配函数
  char* Allocate(size_t bytes);

  // 按照字节对齐来分配内存
  char* AllocateAligned(size_t bytes);

  // 返回目前分配的总的内存
  size_t MemoryUsage() const;
```

下面一一分析

#### Allocate
这是一个最重要的内存分配函数，这个函数会根据你要申请的内存大小来调用另外两个私有的内存分配函数
```cpp
inline char* Arena::Allocate(size_t bytes) {
  // 不需要分配 0 字节的内存
  assert(bytes > 0);

  // 申请的内存小于剩余的内存，就直接在当前内存块上分配内存
  if (bytes <= alloc_bytes_remaining_) {
    char* result = alloc_ptr_;
    alloc_ptr_ += bytes;
    alloc_bytes_remaining_ -= bytes;
    return result;
  }

  // 申请的内存的大于当前内存块剩余的内存，就用这个函数来重新申请内存
  return AllocateFallback(bytes);
}

```

#### AllocateFallback

在申请的内存大于当前块剩余内存时，`AllocateFallback`会被调用，它提供 2 种申请内存的策略：
1. 当前块剩余内存 < 申请的内存 < 默认内存块大小的 1 / 4 (1096 KB / 4 = 1024 KB)，重新申请一个默认大小的内存块 (4096 KB)。
2. 申请的内存大于当前块剩余内存，并且大于默认内存块大小的 1 / 4，直接申请一个需要的的 bytes 大小的内存块。

第二种分配方法的理由是可以**减少内存分配的次数**，分析如下：

假如我当前块剩余 900 KB 内存，而我需要申请 1200 KB 内存，如果我直接申请一块大小为 1200 KB 的内存块，就只需要申请一次；但是如果我在当前块先分配 900 KB 内存，然后再申请一个新的 4096 KB 的内存块，再在里面分配 1200 - 900 = 300 KB 的内存，这样就需要申请 1 次内存，分配两次内存，效率低下了不少(在分配比较频繁的时候)，因此这样直接申请并分配一个 bytes 大小的内存块非常高效方便。

```cpp
// bytes 代表实际要申请的内存大小
char* Arena::AllocateFallback(size_t bytes) {
  // kBlockSize = 4096 KB
  // bytes > 1 / 4 (1024 KB)，调用 AllocateNewBlock 申请一块新的大小为 bytes 的内存
  if (bytes > kBlockSize / 4) {
    char* result = AllocateNewBlock(bytes);
    return result;
  }

  // bytes < 1 / 4 (1024 KB)，申请一个默认大小为 4096 KB 的内存块
  alloc_ptr_ = AllocateNewBlock(kBlockSize);
  alloc_bytes_remaining_ = kBlockSize;

  // 在申请的默认大小的内存块里分配 bytes 字节内存
  char* result = alloc_ptr_;
  alloc_ptr_ += bytes;
  alloc_bytes_remaining_ -= bytes;
  return result;
}

```

#### AllocateNewBlock

这是 `AllocateNewBlock` 的实现，它就是简单的申请了一个 `block_bytes` 大小的内存块

```cpp
char* Arena::AllocateNewBlock(size_t block_bytes) {
  // 申请一个 block_bytes 大小的内存块
  char* result = new char[block_bytes];
  
  // 向 vector 中添加这个内存块的地址
  blocks_.push_back(result);

  // 增加当前内存分配的总量
  memory_usage_.NoBarrier_Store(
      reinterpret_cast<void*>(MemoryUsage() + block_bytes + sizeof(char*)));
  return result;
}
```


#### AllocateAligned

这个函数可以**分配字节对齐的内存**，实现稍微有些复杂，不过仔细分析还是有很多营养的，注释很详细
```cpp
char* Arena::AllocateAligned(size_t bytes) {
  // 设置要对齐的字节数，最多 8 字节对齐，否则就按照当前机器的 void* 的大小来对齐
  const int align = (sizeof(void*) > 8) ? sizeof(void*) : 8;

  // 字节对齐必须是 2 的次幂, 例如 align = 8
  // 8 & (8 - 1) = 1000 & 0111 = 0, 表示 8 是字节对齐的
  assert((align & (align-1)) == 0);   
  
  // 了解一个公式：A & (B - 1) = A % B
  // 所以，这句话的意思是将 alloc_ptr_ % align 的值强制转换成 uintptr_t 类型
  // 这个 uintptr_t 类型代表了当前机器的指针大小
  size_t current_mod = reinterpret_cast<uintptr_t>(alloc_ptr_) & (align-1);
  
  // 如果上面的代码返回 0 代表当前 alloc_ptr_ 已经是字节对齐了
  // 否则就计算出对齐的偏差
  // 例如 current_mod = 2, 则还需要 8 - 2 = 6 个字节才能使得 alloc_ptr 按照 8 字节对齐
  size_t slop = (current_mod == 0 ? 0 : align - current_mod);

  // 分配的字节数加上对齐偏差就是最后需要分配的内存字节总量
  size_t needed = bytes + slop;
  char* result;

  // 当总量小于当前内存块的剩余大小，就直接在当前内存块分配 needed 大小的内存
  if (needed <= alloc_bytes_remaining_) {
    result = alloc_ptr_ + slop;
    alloc_ptr_ += needed;
    alloc_bytes_remaining_ -= needed;
  } else {
    // 否则就按照这个函数的内存分配策略来申请内存，见上面的分析
	result = AllocateFallback(bytes);
  }
  assert((reinterpret_cast<uintptr_t>(result) & (align-1)) == 0);
  return result;
}
```
 

## 总结

总的来说，`Arena` 有 3 种内存分配策略，下面申请的内存用 bytes 表示：
1. bytes < 当前块剩余内存 => 直接在当前块分配。
2. 当前块剩余内存 < bytes < 1024 KB (默认内存块大小的 1 / 4) => 直接申请一个默认大小为 4096 KB 的内存块，然后分配内存。
3. bytes > 当前块剩余内存 && bytes > 1024 KB => 直接申请一个新的大小为 bytes 的内存块，并分配内存。


`Arena` 是一个内存池对象，用来管理 `leveldb` 的内存分配，可以说是整个项目非常重要的模块了，不可不知 ~


> 试着了解开源项目的内存分配策略

