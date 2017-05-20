---
layout:    	post
title:      "Google Test 系列之一 - HelloTest"
subtitle:   "C++ Test"
date:       2017-05-20 18:00:00
author:     "陈登龙"
header-img: "img/post-bg-unix-linux.jpg"
catalog: true
tags:
- UnitTest
---


# Google Test 系列之一 - HelloTest
***
> 版权声明：本文为 cheng-zhi 原创文章，可以随意转载，但必须在明确位置注明出处！ 

## 什么是 Google Test？
`Google Test` 的全称是 `Google C++ Testing Framework`，它是 Google 开发的用于 C++ 的单元测试框架，优秀并且跨平台，github 地址：[Google Test](https://github.com/google/googletest)。

Google 出品，必属精品，我们有必要学会使用！


## 编译 Google Test

#### 下载 Google Test
```
git clone https://github.com/google/googletest.git
```

#### 编译 Google Test
我在 `Windows` 平台使用 VS 2013 编译。

1. 使用 VS 2013 打开 `googletest\msvc\gtest.sln` 工程
2. 分别生成 `Debug` 和 `Release` 的解决方案
3. 在 `googletest\msvc\gtest\Debug(Release)` 下会生成 gtest(d).lib，即编译成功。

#### 配置 Google Test

1. 使用 VS 2013 新建一个 Win 32 控制台工程 HelloTest
2. 配置项目的 **属性 -> C/C++ -> 附加包含目录**，添加`...\googletest\include`，`.../`为你的前面的路径
3. 配置项目的 **属性 -> 链接器 -> 输入**，添加`...\google_test_lib\gtestd.lib`，`.../`为你的前面的路径
4. 配置 Debug 项目的 **属性 -> 代码生成 -> 运行库** 为**多线程调试(/MTd)**，如果是 Release 项目，则配置为**多线程(/MT)**

![Debug](http://upload-images.jianshu.io/upload_images/4613385-24006cfc0c6e0e65.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)


   ![Release](http://upload-images.jianshu.io/upload_images/4613385-ba322dd2ac91d0b1.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)

#### 使用 Google Test
在你的项目的 `_tmain` 这个函数所在的文件里面添加下面的代码
```cpp
#include "stdafx.h"
//添加 Google Test 的头文件
#include <gtest/gtest.h>

// 待测试的函数
int Foo(int a, int b)
{
	if (a == 0 || b == 0)
	{
		throw "don't do that";
	}
	int c = a % b;
	if (c == 0)
		return b;

	return Foo(b, c);
}


// 测试用例
TEST(FooTest, HandleNoneZeroInput){
	EXPECT_EQ(2, Foo(4, 10));
	EXPECT_EQ(6, Foo(30, 18));
}


// 程序入口
int _tmain(int argc, _TCHAR* argv[])
{
    // 初始化
    testing::InitGoogleTest(&argc, argv);

    // 运行所有的测试
    RUN_ALL_TESTS();

    // 等待，避免运行之后立即结束
    getchar();
    return 0;
}
```

运行结果

![结果](http://upload-images.jianshu.io/upload_images/4613385-ba96f181d9ac7dde.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)

这里只是简单的介绍了 Google Test 的基本配置和使用，详细的使用在后续会介绍。



## About Me
- cheng-zhi：C / C++
- GitHub   ：[cheng-zhi](https://github.com/cheng-zhi)
- 个人主页 ：[cheng-zhi](https://cheng-zhi.github.io/)
- 微博     ：@cheng-zhi
- 微信号   ：chengzhi-01
- 微信公众号：记录我的成长与分享，欢迎扫码关注，id：growingshare

![ID:growingshare](https://cheng-zhi.github.io/img/wechart.jpg)
