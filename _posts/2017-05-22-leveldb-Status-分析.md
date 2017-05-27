---
title: leveldb Status 分析
date : 2017-05-22 21:00:00
---

# leveldb Status 分析 
***

> 版权声明：本文为 cheng-zhi 原创文章，可以随意转载，但必须在明确位置注明出处！ 

## Status 简介



在 `leveldb` 中，你可以使用 `Status` 这个类来得到你的函数的返回的状态，它的基本用法如下：

```cpp
leveldb::Status s = function();
if (!s.ok()) 
cerr << s.ToString() << endl;
```

你可在 `leveldb/include/status.h` 中找到 `Status` 的定义


## Status 的状态

`leveldb` 用下面的一个 `enum` 类型来表示所有应该出现的状态：
```cpp
enum Code {
  kOk = 0,
  kNotFound = 1,
  kCorruption = 2,
  kNotSupported = 3,
  kInvalidArgument = 4,
  kIOError = 5
}
```

`leveldb` 使用下面的一组函数来分别返回这些状态：
```cpp

// 返回索引为 4 的字节，代表消息的类型，后面会有解释。
Code code() const {
  return (state_ == NULL) ? kOk : static_cast<Code>(state_[4]);
}

// Returns true iff the status indicates success.
bool ok() const { return (state_ == NULL); }

// Returns true iff the status indicates a NotFound error.
bool IsNotFound() const { return code() == kNotFound; }

// Returns true iff the status indicates a Corruption error.
bool IsCorruption() const { return code() == kCorruption; }

// Returns true iff the status indicates an IOError.
bool IsIOError() const { return code() == kIOError; }

// Returns true iff the status indicates a NotSupportedError.
bool IsNotSupportedError() const { return code() == kNotSupported; }

// Returns true iff the status indicates an InvalidArgument.
bool IsInvalidArgument() const { return code() == kInvalidArgument; }
```


## Status 的本质

在 `Status.h` 源码中有一段这样的注释：
```cpp
private:
// OK status has a NULL state_.  Otherwise, state_ is a new[] array
// of the following form:
// state_[0..3] == length of message
// state_[4]    == code
// state_[5..]  == message
const char* state_;

```

可以看出 `Stauts` 本身就是一个字符串，只不过 `Google` 工程师给这个字符串的某些字节区域限定了含义：
1. state_[0, 3] 字节代表消息的长度，这个长度是从 state_[5, ...] 开始的，前面的 5 个字节不算。
2. state_[4] 字节代表消息的类型，就是上面介绍的 `enum Code` 的 6 种类型。
3. state_[5, ...] 代表实际的消息体。

## 一个关键的函数

下面的这一组函数用来组合指定的状态信息：
```cpp
// Return a success status.
static Status OK() { return Status(); }

// Return error status of an appropriate type.
static Status NotFound(const Slice& msg, const Slice& msg2 = Slice()) {
  return Status(kNotFound, msg, msg2);
}

static Status Corruption(const Slice& msg, const Slice& msg2 = Slice()) {
  return Status(kCorruption, msg, msg2);
}

static Status NotSupported(const Slice& msg, const Slice& msg2 = Slice()) {
  return Status(kNotSupported, msg, msg2);
}

static Status InvalidArgument(const Slice& msg, const Slice& msg2 = Slice()) {
  return Status(kInvalidArgument, msg, msg2);
}

static Status IOError(const Slice& msg, const Slice& msg2 = Slice()) {
  return Status(kIOError, msg, msg2);
}
```

这里都调用了下面这个重要的函数，用它来完成 `msg` 和 `msg2` 的拼接：
```cpp
#include "leveldb/db/leveldbutil.cc"

Status::Status(Code code, const Slice& msg, const Slice& msg2) {
  assert(code != kOk);
  // 分别得到长度
  const uint32_t len1 = msg.size();
  const uint32_t len2 = msg2.size();
  // 如果 len2 不为 0，就在消息中加上 ": " 这两个字节
  const uint32_t size = len1 + (len2 ? (2 + len2) : 0);
  // 实际的消息需要加上前面的 5 个字节，所以要多分配 5 个
  char* result = new char[size + 5];
  // 先拷贝前面的 4 个字节作为长度， uint32_t 为 4B 大小
  memcpy(result, &size, sizeof(size));
  // 索引为 4 的字节存储消息的类型，就是那 6 种之一
  result[4] = static_cast<char>(code);
  // 拷贝实际的消息体，从索引为 5 的字节开始
  memcpy(result + 5, msg.data(), len1);
  if (len2) {
    result[5 + len1] = ':';
    result[6 + len1] = ' ';
    // len2 不为 0，就拼接上 ": "
    memcpy(result + 7 + len1, msg2.data(), len2);
  }
  
  state_ = result;
}
```


## 附加的工具函数


`Status` 还提供了下面两个函数作为工具，作为学习还是不错的：

```cpp
#include "leveldb/db/leveldbutil.cc"

// 拷贝状态，因为又分配了新的内存，相当于深拷贝
const char* Status::CopyState(const char* state) {
  uint32_t size;
  // 得到 state 的消息体的长度，消息体长度存储在前面的 4 个字节中
  memcpy(&size, state, sizeof(size));
  char* result = new char[size + 5];
  // 拷贝消息体
  memcpy(result, state, size + 5);
  return result;
}

```

这个函数将 `Code` 的最新的状态转换为字符串，我们常用 `Status s = ...` 来接受函数的返回值，然后输出 `s.ToString()` ：
```cpp
#include "leveldb/db/leveldbutil.cc"

std::string Status::ToString() const {
  if (state_ == NULL) {
    return "OK";
  } else {
    char tmp[30];
    const char* type;
	// 判断当前的状态
    switch (code()) {
      case kOk:
        type = "OK";
        break;
      case kNotFound:
        type = "NotFound: ";
        break;
      case kCorruption:
        type = "Corruption: ";
        break;
      case kNotSupported:
        type = "Not implemented: ";
        break;
      case kInvalidArgument:
        type = "Invalid argument: ";
        break;
      case kIOError:
        type = "IO error: ";
        break;
      default:
        snprintf(tmp, sizeof(tmp), "Unknown code(%d): ",
                 static_cast<int>(code()));
        type = tmp;
        break;
    }
    // 拷贝到 result 中
    std::string result(type);
    uint32_t length;
    memcpy(&length, state_, sizeof(length));
    result.append(state_ + 5, length);
    return result;
  }
}
```


## 总结

`Status` 这个类既然是用来表示状态的，所以我们在使用的使用，大部分情况都是得到一个函数的返回的状态值，然后进行相应的判断即可。

