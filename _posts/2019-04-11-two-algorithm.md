---
title: 分享 2 个技巧性面试算法
date: 2019-04-11 16:00:00
---
# 分享 2 个技巧性面试算法
***
> 版权声明：本文为 {{ site.name }} 原创文章，可以随意转载，但必须在明确位置注明出处！

## 1、求和
求和：1 - 2 + 3 - 4 + 5 - 6 ... + n（n 足够大）

一般方法，循环求和，当 n 很大时比较耗时。
```c
long Algorithm(long n) {
  if (n <= 0) {
    printf("error: n must > 0");
    exit(1);
  }

  long result = 0;
  int flag = 1;
  for (int i = 1; i <= n; i++) {
    result += flag * i;
    flag *= -1;
  }
  return result;
}
```

推荐方法：寻找规律，使用公式计算结果
```c
/**
 * i:   1   2   3   4   5   6
 *      1 - 2 + 3 - 4 + 5 - 6 ... + n
 * Res: 1  -1   2  -2   3  -3
 */
long Algorithm(long n) {
  if (n <= 0) {
    printf("error: n must > 0");
    exit(1);
  }

  if (0 == n % 2)
    return (n / 2) * (-1);
  else
    return (n / 2) * (-1) + n;
}
```


## 2、函数计算
使用一种技巧性的编程方法用一个函数来实现两个函数的功能：
```
Fn1 = n / 2! + n / 3! + n / 4! + n / 5! + n / 6!
Fn2 = n / 5! + n / 6! + n / 7! + n / 8! + n / 9!
```
注意：题目要求使用技巧性编程，所以肯定不能使用暴力解决，而且表达式的项数有限，所以可以采用经典的空间换时间的方法。

```c
double Algorithm(int n, int flag) {
  // flag = 0: 2! 3! 4! 5! 6!
  // flag = 1: 5! 6! 7! 8! 9!
  double x[2][5] = {
    {2, 6, 24, 120, 720},
    {120, 720, 5040, 40320, 362880}};
  double result = 0;
  for (int i = 0; i < 5; i++) {
    result += n / x[flag][i];
  }
  return result;
}
```



> {{ site.prompt }}

<div  align="center">
<img src="{{ site.url }}/images/wechart.jpg" width = "200" height = "200"/>
