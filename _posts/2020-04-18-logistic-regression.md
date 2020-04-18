---
title: 从 0 开始机器学习 - 逻辑回归原理与实战！
date: 2020-04-18 20:00:00
---
# 从 0 开始机器学习 - 逻辑回归原理与实战！
***
> 版权声明：本文为 {{ site.name }} 原创文章，可以随意转载，但必须在明确位置注明出处！

之前的文章学习了线性回归，这次来跟大家分享下我对逻辑回归的一些理解。

## 一、什么是分类问题？

这个其实很好理解，就比如你手里有一个苹果和一个橘子，我们的分类问题就是可以描述为如何写一个算法让计算机认出哪个是苹果，哪个是橘子。

分类问题的输出是不连续的离散值，比如设定程序输出 1 表示苹果，0 表示橘子。但我们之前学习的线性回归的输出是连续的，如预测房价，肯定不能用 0 和 1 来表示房价。

所以记住一点：分类问题输出离散值，线性回归问题输出连续值。

## 二、什么是逻辑回归？

今天要学习的这个逻辑回归是属于分类问题，你可能对「逻辑回归」有疑惑，既然是分类问题，为何要说成回归问题？干吗不叫逻辑分类问题？

我也觉得有点别扭，可谁让大师比我们早生出来呢？如果我们早点出生，发明这个算法，或许就命名为逻辑分类了，哈哈。

既然改变不了，我们就只能接受了，把他当成分类问题记住即可。

## 三、逻辑回归的假设函数

还记得之前线性回归的假设函数吗，就是预测的模型，我们用的是多项式，但在分类问题中我们就要换模型了，为啥？

很简单，我们从分类问题和线性回归问题的定义可以知道，线性回归问题输出连续值（房价），逻辑回归只输出离散值（0 1），所以模型的输出不一样，因此需要选择一个能输出离散值的函数 $g$：


$$
h_\theta(x)=g(\theta^TX)
$$


其中 $X$ 表示特征向量，$\theta^T$ 表示待学习的参数向量。

但在机器学习分类问题中，模型输出 0 或者 1 的前一步通常是确定 0 或者 1 的概率，而不是直接根据实例数据就输出 0 或 1，比如模型预测是苹果的概率是 90%，那是橘子的概率就是 10%（因为概率和为 1），进而模型认为该水果是苹果的可能性最大，所以输出 1 来表示当前识别的水果是苹果。

根据这个概率特性，我们的逻辑回归假设函数取一个常用的逻辑函数 Sigmoid Function：


$$
g(z)=\frac{1}{1+{e^{-z}}}
$$



```python
import numpy as np

def sigmoid(z):
    return 1 / (1 + np.exp(-z))
```



![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/sigmoid_fun.png)

使用这个函数来做为逻辑回归的假设函数，这样就能根据输入参数 $z$ 来输出 $y = 1$ 的可能性了，比如输出 $h_\theta(x) = 0.9$，就表示有 90% 的概率是苹果，有 10% 的概率是橘子。

## 四、逻辑回归的分类边界

在分类问题中存在分类（决策）边界（Decision Boundary）的概念，因为我们最终是要将数据用函数分类，体现在坐标系中就是函数曲线把数据分为 2 类，比如一类是苹果，一类是橘子。

理解分类边界的目的就是为了理解逻辑回归的假设函数是如何工作的。下面通过一个小例子说明下分类边界是如何得出的，其实也容易理解。

我们假设：

- $h_\theta(x) >= 0.5$ 时，预测 $y = 1$，苹果
- $h_\theta(x) < 0.5$ 时，预测 $y = 0$，橘子

从 Sigmoid 函数图像可以看出：

- $z > 0$：$g(z)$ 在 $(0.5, 1.0)$ 之间
- $z = 0$：$g(z) =  0.5$
- $z < 0$：$g(z)$ 在 $(0, 0.5)$ 之间

又因为 $h_\theta(x) = g(z)$，要注意这里 $z = \theta^Tx$，所以上面的假设可以替换为：

- $z = \theta^Tx > 0$ 时，预测 $y = 1$，苹果
- $z = \theta^Tx = 0$ 时，分类边界！
- $z = \theta^Tx < 0$ 时，预测 $y = 0$，橘子

用个图来直观的说明下：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/logistic_decision_boundary.png)

这个图已经说明的很详细了，中间的红线就是分类边界 $\theta^Tx = -3 + x_1 + x_2 = 0$，两边分别是大于 0 和小于 0 的情况，在实际应用中，经常把 $\theta^Tx >= 0$ 合并来表示 $y = 1$。

这个例子的分类边界是直线，其实分类边界也可以是非线性边界，比如一个圆：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/no_linear_boundary.png)

以上是两个简单的分类边界例子，实际使用中使用更加复杂的多项式可以拟合很复杂的分类边界。

## 五、代价函数和梯度下降

与线性回归一样，逻辑回归也需要定义代价函数来求出模型的最优参数 $\theta$，不过代价函数不能使用模型误差的平方和，因为将逻辑回归的假设函数带入平方误差的代价函数会导致代价函数不是凸函数，进而代价函数会产生许多局部最优值，对梯度下降求最优值会有很大影响。

由于这个缺点，所以我们需要重新定义逻辑回归的代价函数：


$$
J(\theta) = \frac{ 1 }{ m }\sum\limits_{i = 1}^m {Cost({h_\theta} (x^{(i)}), y^{(i)})}
$$


$Cost$ 函数如下：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/logistic_hx.png)

这个函数直观上不好理解，可以用曲线看下：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/logistic_cost_fun.png)

- $y = 1$ 时：当 $h_\theta(x) = 1$，$ Cost -> 0$；当 $h_\theta(x) ->0$ 时 $Cost ->$ 无穷大；
- $y = 0$ 时：当 $h_\theta(x) = 0$，$ Cost -> 0$；当 $h_\theta(x) ->1$ 时 $Cost ->$ 无穷大；

可以用一句话来理解这个代价函数：**当模型的预测与实际值差异越大，代价值越大。**

但是我们发现上面的代价函数是分开的，不方便梯度下降计算，能不能整合到一起呢？还真的可以：


$$
Cost(h_\theta( x ), y) = -y\times log(h_\theta(x)) - (1 - y)\times log(1 - h_\theta(x))
$$


当 $y = 1$ 时后面项为 0，当 $y = 0$ 时，前面项为 0，正好对应之前的分段表达式，再添加上标代入代价函数 $J(\theta)$ 中：


$$
J(\theta) = -\frac{1}{m}\sum\limits_{i = 1}^{m}{[{y^{(i)}}\log ({h_\theta}({x^{(i)}}))+( 1-{y^{(i)}})\log ( 1 - h_\theta({x^{(i)}}))]}
$$


有了代价函数，我们就可以利用之前学会的[梯度下降法](https://dlonng.com/posts/one-var-gd)来迭代求代价函数最小值啦！

## 六、逻辑回归实战

最后我们来用前面学习的逻辑回归技术来对 2 类数据进行分类！

### 6.1 数据准备

要分类的数据可视化如下，一共只有 2 个类别，所以只需要使用直线决策边界即可：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/logistic_data.png)

### 6.2 假设函数

```python
def sigmoid(z):
    return 1 / (1 + np.exp(-z))
```

### 6.3 代价函数

这里仍然使用向量化代码表示：

```python
# 逻辑回归代价函数
def cost_function(theta, X, y):
    # 向量化代码
    return np.mean(-y * np.log(sigmoid(X @ theta)) - (1 - y) * np.log(1 - sigmoid(X @ theta)))
```

### 6.4 梯度下降

梯度下降的原理在之前的[这篇文章](https://dlonng.com/posts/one-var-gd)有介绍：

```python
# 梯度计算
# return 梯度的一维数组
def gradient(theta, X, y):
    return (1 / len(X)) * X.T @ (sigmoid(X @ theta) - y)
```

### 6.5 训练参数

这里利用已有的优化手段来优化代价函数（损失函数）：

```python
import scipy.optimize as opt

# 用 opt.minimize 来训练逻辑回归的参数
# Newton-CG 是牛顿法家族的一种，利用损失函数二阶导数矩阵即海森矩阵来迭代优化损失函数
res = opt.minimize(fun = cost_function, x0 = theta, args = (X, y), method = 'Newton-CG', jac = gradient)
```

### 6.6 在训练集上预测

```python
# 计算 y 在训练集上的预测值
y_predict = predict(X, final_theta)

# 打印分类报告
print(classification_report(y, y_predict))
```

报告中的 f1-score 分数分别为 0.86 和 0.91，说明分类结果还是很不错的：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/logistic_class_report.png)

### 6.7 输出分类边界

我们用绘图库来绘制出预测的分类边界，可以发现分类边界能够比较好分开两个类别的数据：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/predict_data_boundary.png)

OK！今天登龙跟大家分享了逻辑回归的原理和实战编程，大家多多实践，早日学会！**文中完整注释代码**在我的仓库中：

[https://github.com/DLonng/AI-Notes/tree/master/MachineLearning](https://github.com/DLonng/AI-Notes/tree/master/MachineLearning)

大家下期见 :)


> {{ site.prompt }}

<div  align="center">
<img src="https://dlonng.com/images/wechart.jpg" width = "200" height = "200"/>