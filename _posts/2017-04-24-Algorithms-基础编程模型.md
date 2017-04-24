---
layout:     post
title:      "Algorithms - 基础编程模型"
subtitle:   "Java Algorithms"
date:       2017-04-24 11:00:00
author:     "陈登龙"
header-img: "img/post-bg-algorithms.jpg"
catalog: true
tags:
    - Algorithms
---

# Algorithms - 基础编程模型
***
> 版权声明：本文为 cheng-zhi 原创文章，可以随意转载，但必须在明确位置注明出处！ 

#### 专题的由来
算法和数据结构是计算机科学的核心，因此这里开设一个专题来记录自己的算法和数据结构的学习经历。

该专题基于 Robert Sedgewick 的 <<算法>> 一书，使用 Java 语言来学习，这本书非常不错，可以说是必看书籍，另外还有配套[网站](http://algs4.cs.princeton.edu/home/)。

#### 进入正题 - 计算机科学的核心
算法是计算机科学的基础，是这个领域的核心。

这里一再强调算法的重要性，是因为在实际的开发过程中，除了业务逻辑之外的一些复杂操作一般都需要使用算法来实现，这些算法一般都已经实现好了，我们在实际开发过程中，需要学会使用即可，但是作为学习来说，我们应该理解算法的原理。

#### 第一个算法
这里使用 Java 来编写第一个算法 - 欧几里得算法。

算法目的：找到两个数的最大公约数。

    public static int gcd(int p, int q) {
		if (0 == q)
			return p;
		int r = p % q;
		return gcd(q, r);
	}

算法非常简单，利用到了递归的思想，递归在处理简单的算法逻辑上面是非常清晰的。

#### Java 基础复习
以下是需要注意的 Java 基础：
* 使用 javac 来编译 Java 程序，使用 java 来运行 Java 程序。
* 为了减少表达式对优先级规则的依赖，强烈建议使用括号。
* 浮点转整形会截断小数部分而非四舍五入。
* Java 是一种强类型语言。
* 建议使用声明并初始化的方法。
* 学会在命令行中进行输入输出的重定向。

#### About Me
* cheng-zhi：自动化专业，喜欢 Android，不只是 App。
* Github：[cheng-zhi](https://github.com/cheng-zhi)
* 个人主页：[cheng-zhi](https://cheng-zhi.github.io/)
