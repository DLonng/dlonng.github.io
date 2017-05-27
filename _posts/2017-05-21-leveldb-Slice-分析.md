---
title: leveldb Slice
date : 2017-05-21 18:00:00
---


# leveldb Slice 分析

***
> 版权声明：本文为 cheng-zhi 原创文章，可以随意转载，但必须在明确位置注明出处！ 

## 什么是 leveldb Slice？


有许多优秀的 `C++` 框架都没有直接使用 `C++` 提供的 `string`，而是自己封装了一份字符串的操作，`Slice` 就是 `leveldb` 自己使用的字符串操作类。

## 源码位置


`slice` 位于源码的这个位置：
```
leveldb/include/leveldb/slice.h
```


## 模仿实现


学习大神的代码，最好的方式我认为是理解之后自己动手写一遍，即使是抄，对我们也是有帮助的，因为实践过的东西记忆更加深刻。
```cpp
#ifndef SLICE_H_
#define SLICE_H_

#include <assert.h>
#include <stddef.h>
#include <string.h>

#include <string>


namespace myspace {

class Slice {
public:
	// Create an empty slice.
	Slice() : data_(""), size_(0) { }

	// Create a slice that refers to cstr[0, n - 1].
	Slice(const char* cstr, size_t n) : data_(cstr), size_(n) { }

	// Create a slice that refers to the contents of "str"
	Slice(const std::string& str) : data_(str.data()), size_(s.size()) { }

	// Create a slice that refers to cstr[0, strlen(cstr) - 1].
	Slice(const char* cstr) : data_(cstr), size_(strlen(cstr)) { }

	// Return a pointer to the beginning of the referenced data.
	const char* data() const { return data_; }

	// Return the length (in bytes) of the referenced data.
	size_t size() const { return size_; }

	// Return true if the length of the referenced data id zero.
	bool empty() const { return size_; }

	// Return the it byte in the referenced data.
	// REQUESTS: n < size()
	char operator[](size_t n) const {
		assert(n < size());
		return data_[n];
	}

	bool operator==(const slice& lhs, const Slice& rhs) {
		return ((lhs.size_() == rhs.size()) && (memcmp(lhs.data(), rhs.data(), lhs.size()) == 0));
	}

	bool operator!=(const Slice& lhs, const Slice& rhs) {
		return !(lhs == rhs);
	}



	// Change this slice to refer to an empty array.
	void clear() { data_ = ""; size_ = 0; }

	// Drop the first "n" bytes from this slice.
	void remove_prefix(size_t n) {
		assert(n <= size());
		data_ += n;
		size_ -= n;
	}

	// Return a string that contains the copy of the referenced data.
	std::string to_string() const { return std::string(data_, size_); }

	// Three-way comparison.
	// Return value:
	// <  0 if "*this" <  "b"
	// == 0 if "*this" == "b"
	// >  0 if "*this" >  "b"
	int compare(const Slice& b) const {
		const size_t min_len = (size_ < b.size_) ? size_ : b.size_;
		int r = memcmp(data_, b.data_, min_len);

		// if equal
		if (0 == r) {
			if (size_ < b.size_)
				r = -1;
			else if (size_ > b.size_)
				r = +1;
		}

		return r;
	}

private:
	const char* data_;
	size_t size_;	
};

}
#endif //SLICE_H_
```

函数的定义都放在了类的声明里面，这些函数默认都是 `inline` 函数。

