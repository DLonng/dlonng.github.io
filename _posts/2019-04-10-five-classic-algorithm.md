---
title: 记录 5 个经典面试算法
date: 2019-04-10 19:00:00
---
# 记录 5 个经典面试算法
***
> 版权声明：本文为 {{ site.name }} 原创文章，可以随意转载，但必须在明确位置注明出处！

记录 5 个经典的面试算法题。

## 1、斐波那契数列
斐波那契数列：1 1 2 3 5 8 13 ...
```c
/**
 * 递归实现斐波那契数列
 * 代码简单，但效率较低
 */
long Fibonacci(int n) {
  if (1 == n)
    return 1;
  if (2 == n)
    return 1;
  if (n > 2)
    return Fibonacci(n - 1) + Fibonacci(n - 2);
}
```

```c
/**
 * 循环实现斐波那契数列
 * 效率比递归高
 */
long Fibonacci(int n) {
  long num1 = 1;
  long num2 = 1;
  long num3 = 0;
  // 从第三个数开始计算
  for (int i = 2; i < n; i++) {
    num3 = num1 + num2;
    num1 = num2;
    num2 = num3;
  }
  return num3;
}
```

## 2、杨辉三角
杨辉三角要求输出如下图形：
```
1

1    1

1    2    1

1    3    3    1

1    4    6    4    1

1    5   10   10    5    1
```

规律：`a[i][j] = a[i - 1][j - 1] + a[i - 1][j]`
```c
#define ROW 10
#define COL 10

/**
 * 核心算法
 * a[i][j] = a[i - 1][j - 1] + a[i - 1][j]
 */
void YangHui() {
  int x[ROW][COL] = { 0 };

  // 第一列和对角线赋值为 1
  for (int i = 0; i < ROW; i++) {
    x[i][0] = x[i][i] = 1;
  }

  // 遍历左下角，计算其他元素
  for (int i = 2; i < ROW; i++) {
    for (int j = 1; j < i; j++) {
      x[i][j] = x[i - 1][j - 1] + x[i - 1][j];
    }
  }

  // 输出左下角元素
  for (int i = 0; i < ROW; i++) {
    for (int j = 0; j <= i; j++) {
      printf("%5d", x[i][j]);
    }
    printf("\n");
  }
}
```

## 3、整数转二进制
10 -> 1010
```c
/**
 * 使用栈将整数转二进制
 */
void num2bit(int num) {
  std::stack<int> s;
  int r = 0;
  while (num) {
    r = num % 2;
    // 余数入栈
    s.push(r);
    num /= 2;
  }

  // 反向输出
  while(!s.empty()) {
    std::cout << s.top() << " ";
    s.pop();
  }
}
```

## 4、素数问题
素数：只能被 1 和自身整除的数
```c
/**
 * 只能被 1 和自身整除的数称为素数
 */
int is_primer(int num) {
  for (int i = 2; i <= sqrt(num); i++) {
    if (num % i == 0)
      return 0;
  }
  return 1;
}

/**
 * 输出 [num1, num2] 之间的素数
 */
void output_primer(int num1, int num2) {
  for (int i = num1; i <= num2; i++) {
    if (is_primer(i))
      printf("%d ", i);
  }
}
```

## 5、字符串转整数
```c
/**
 * "123456" -> 123456
 * 可增加对负数的 if 判断
 * 可增加对字符 0 - 9 的筛选
 */
long str2num(char *str) {
  long sum = 0;
  int len = strlen(str);

  for (int i = 0; i < len; i++) {
    sum = sum * 10 + (*str) - '0';
    str++;
  }

  return sum;
}
```










> {{ site.prompt }}

<div  align="center">
<img src="{{ site.url }}/images/wechart.jpg" width = "200" height = "200"/>
