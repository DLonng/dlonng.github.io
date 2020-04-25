---
title: 从 0 开始机器学习 - 正则化技术原理与编程!
date: 2020-04-25 21:40:00
---
# 从 0 开始机器学习 - 正则化技术原理与编程!
***
> 版权声明：本文为 {{ site.name }} 原创文章，可以随意转载，但必须在明确位置注明出处！

之前学习了线性回归，逻辑回归和梯度下降法，今天学习的这个技术能够帮助我们训练的模型对未知的数据进行更好的预测 - 正则化技术！

快来一起学习学习，学习使我快乐 (*^▽^*)！

## 一、正则化是什么？

正则化（Regulariation）这 3 个字听起来挺高大上的，其实就是一种解决机器学习过拟合问题的技术，使用这项技术可以让我们在训练集上训练的模型对未知的数据也能很好地拟合。

机器学习模型对未知数据的拟合能力又称为泛化能力，泛化能力比较好的模型，对未知数据拟合的也比较不错，如果对训练数据产生过拟合（over-fitting）问题，那泛化能力也会变差。

当出现过拟合问题后，处理的方式有 2 种：

- 降维：减少特征数量，把模型多项式阶数降低，这样高阶项就少了，模型曲线就不会那么复杂
- 正则化：不直接减少特征，而是增大代价函数中特征参数的系数

我们今天要介绍的就是正则化技术，下面用个例子先来说明什么是过拟合与欠拟合。

## 二、过拟合 VS 欠拟合

### 2.1 线性回归例子

这里还以预测房价为例，分别解释以下 3 种情况：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/linear_fitting.png)

- 欠拟合：模型选择的特征数量太少（2 个），不能对训练数据很好地拟合，会产生高偏差
- 正常拟合：模型选择的特征数量合适（3 个），能对训练数据拟合较好
- 过拟合：模型选择的特征数量过多（5 个），对训练数据过度拟合，会导致高方差

在实际应用中，通常选择的特征会比较多，很容易出现过拟合，所以解决这个问题很有必要。

### 2.2 逻辑回归例子

逻辑回归问题同样会产生过拟合与欠拟合问题，比如这个分类问题：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/logit_fitting.png)

这里忘记注释类型了，不过原理一样：

- 欠拟合：用直线分类，一看就不合适，因为直观来看决策边界是圆弧形状
- 正常拟合：决策边界是圆弧形状，拟合的效果比较好
- 过拟合：决策边界分类的太严格了，在未知样本上的预测效果很差

简单总结下：模型的参数越多，使用的多项式次数（$x^n$）就越大，模型曲线就越复杂，这些高阶次的项会导致过拟合问题。

这个正则化技术要解决过拟合问题的实质就是：**减小高阶次项对模型整体的影响，以此来提高模型对未知样本的预测能力。**

因为我们在训练数据上训练出的模型，最终是要用到未知的样本中的，不然就失去工程应用的意义了。OK，那下面就来正式学习下这个技术的原理，其实很容易，就是在代价函数后面加上一个正则化项公式。

## 三、正则化原理

### 3.1 在假设函数中理解正则化

我还以预测房价的例子来说明正则化技术的原理，模型的假设函数如下：


$$
h_\theta( x )=\theta_0 + \theta_1 x_1 + \theta_2 x_{2}^2 + \theta_3 x_{3}^3 + \theta_4 x_{4}^4
$$


假如 $\theta_3$ 表示房屋厨房面积，$\theta_4$ 表示房屋的地理位置，这两个特征导致模型阶次太高（$x_3^3, x_4^4$），我想减少它俩对假设函数的影响，也就是说我想在代价函数求得最优值后，得到的最优特征向量中这 2 个参数尽可能**趋向于 0**，这样上面的模型就变成：


$$
h_\theta( x )=\theta_0 + \theta_1 x_1 + \theta_2 x_{2}^2 + 0 * x_{3}^3 + 0 * x_{4}^4
$$


注意了：这里只是将 $\theta_3 -> 0$，$\theta_4 -> 0$，而并不是让他们直接等于 0，因为我们是正则化不是降维，通过将这 2 个参数趋向于 0 使得他们的高阶次项可以忽略不计，就能得到减少 $x_3, x_4$ 对原模型的影响了。

用个机器学习的术语来说就是：通过正则化技术来**惩罚** $\theta_3$ 和 $\theta_4$ 这 2 个特征！让他们对模型预测产生的影响降低！

### 3.2 在代价函数中理解正则化

在应用中要惩罚参数需要通过对代价函数进行正则化，也就是在代价函数最小化的同时，将要惩罚的参数尽可能的设置为 0。

这里以线性回归代价函数为例，看下如何正则化 $\theta_3, \theta_4$：




$$
J(\theta) = \frac{1}{2m}[\sum\limits_{i=1}^m ( h_{\theta}(x^{(i)})-y^{(i)})^{2}  + 1000\theta _3^2 + 10000\theta _4^2]
$$




我们在代价函数后面加了 2 项 $1000\theta_3^2, 10000\theta_4^2$，因为我们最后要将代价函数最小化，因此也需要对添加的这两项最小化，然而因为 10000 系数比较大，这就导致为了将 $10000\theta_4^2$ 整体优化变小，就需要把参数 $\theta_4$ 设置的足够小，这样两者相乘的结果才能变的足够小，以此就完成了对 $\theta_4^2$ 的正则化，比如：


$$
10000 * (0.001)^2 = 0.01
$$


需要正则化的参数前的系数需要根据实际情况来合适的选择，这里举的例子设置的 1000 和 10000。

上面的例子指定了需要正则化的参数为 $\theta_3, \theta_4$，但是在实际的应用中，因为训练集的特征非常多，我们不知道需要对哪些参数正则化，所以就对所有的参数进行正则化，然后让优化算法来帮我们确定每个正则化参数前的系数，这样的代价函数如下：


$$
J(\theta) = \frac{1}{2m}[\sum\limits_{i=1}^m ( h_{\theta}(x^{(i)})-y^{(i)})^{2} + \lambda \sum\limits_{j=1}^{n}{\theta_j^2}]
$$


代码如下：

```python
# 线性回归正则化代价函数
# 正则化系数 lambda 设置为 1
def regularized_cost(theta, X, y, lamd = 1):
    # 不对 theta_0 正则化
    theta_one2n = theta[1:]

    # 正则化项, power 求平方，sum 求和
    regularized_term = (lamd / 2 * len(X)) * np.power(theta_one2n, 2).sum()
    
    # cost_function 是未加正则化的代价函数
    return cost_function(theta, X, y) + regularized_term
```



这就是最终的对代价函数进行正则化的公式，可以看出就是增加一个正则化项，其中系数 $\lambda$ 称为正则化系数，要注意$j = 1$ ，即没有对 $\theta_0$ 正则化，因为 $\theta_0$ 对应的特征是 $x_0$ ，而 $x_0 = 1$ 是我们人为加上的，所以没必要对 $\theta_0$ 正则化。

### 3.3 正则化系数的影响

在正则化过程中，系数 $\lambda$ 的选择非常重要，它会对整体产生如下影响：

- $\lambda$ 过大：所有的参数都被过度正则化了，导致特征参数都几乎为 0，假设函数就会变为 $h_\theta(x) = \theta_0$ 直线，即欠拟合
- $\lambda$ 过小：对参数的正则化程度不够，当参数较多时，仍然会导致过拟合
- $\lambda$ 合适：减少高次项的影响，正常拟合

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/lambda_regular.png)

OK！正则化的原理就学习完了，下面来实际使用下正则化技术。

## 四、正则化代码实战

这部分我打算分别跟大家分享下如何在线性回归和逻辑回归中使用正则化编程。

### 4.1 线性回归正则化

因为我们在代价函数中增加了正则化项：


$$
\frac{\lambda}{2m} \sum\limits_{j=1}^{n}{\theta_j^2}
$$


所以梯度下降的计算也多了一项正则化梯度（直接对正则化部分求导）：


$$
\frac{\lambda}{m} \sum\limits_{j=1}^{n}{\theta_j}
$$


因为不需要对 $\theta_0$ 正则化，所以梯度迭代的计算分为如下 2 部分：


$$
\theta_0:= \theta_0 - \alpha \frac{1}{m} \sum\limits_{i=1}^{m}{((h_\theta({x^{(i)}})-{y^{(i)}})x_0^{(i)}})
$$

$$
\theta_j:= \theta_j - \alpha [\frac{1}{m} \sum\limits_{i=1}^{m}{((h_\theta({x^{(i)}})-{y^{(i)}})x_0^{(i)}}) + \frac{\lambda}{m} \theta_j]
$$

$$
j = 1, 2, ... m
$$


我们发现上面第二个式子可以进一步合并 $\theta_j$：


$$
\theta_j:= (1 - \alpha \frac{\lambda}{m})\theta_j - \alpha [\frac{1}{m} \sum\limits_{i=1}^{m}{((h_\theta({x^{(i)}})-{y^{(i)}})x_0^{(i)}})]
$$




至此线性回归的正则化梯度公式就出来了，来用 Python 代码把它写出来：

```python
# 添加正则化项的线性回归梯度下降
def regularized_gradient(theta, X, y, lamd = 1):
    # 不对 theta_0 正则化
    theta_one2n = theta[1:]
    
    # 正则化梯度：(lambda / m) * theta_j 
    regularized_theta = (lamd / len(X)) * theta_one2n
    
    # 合并 theta_0
    regularized_term = np.concatenate([np.array([0]), regularized_theta])
    
    # 返回梯度：原梯度 + 正则化梯度
    return gradient(theta, X, y) + regularized_term 
```



### 4.2 逻辑回归正则化

逻辑回归代价函数的正则化项与线性回归相同：


$$
J(\theta) = \frac{1}{m}\sum\limits_{i = 1}^{m}{[-{y^{(i)}}\log ({h_\theta}({x^{(i)}}))-( 1-{y^{(i)}})\log ( 1 - h_\theta({x^{(i)}}))]} + \frac{\lambda}{2m} \sum\limits_{j=1}^{n}{\theta_j^2}
$$


```python
# 逻辑回归正则化代价函数
# lambda 设置为 1
def regularized_cost(theta, X, y, lamd = 1):
    theta_one2n = theta[1:]
    # lambda / (2 * m)
    regularized_term = (lamd / (2 * len(X))) * np.power(theta_one2n, 2).sum()
    
    # 返回加上正则化的总代价
    return cost_function(theta, X, y) + regularized_term
```



逻辑回归正则化梯度（$h_\theta(x)$ 不同）：


$$
\theta_0:= \theta_0 - \alpha \frac{1}{m} \sum\limits_{i=1}^{m}{((h_\theta({x^{(i)}})-{y^{(i)}})x_0^{(i)}})
$$



$$
\theta_j:= (1 - \alpha \frac{\lambda}{m})\theta_j - \alpha [\frac{1}{m} \sum\limits_{i=1}^{m}{((h_\theta({x^{(i)}})-{y^{(i)}})x_0^{(i)}})]
$$

$$
j = 1, 2, ..., m
$$




```python
# 添加正则化项的逻辑回归梯度下降
def regularized_gradient(theta, X, y, lamd = 1):
    # 不对 theta_0 正则化
    theta_one2n = theta[1:]
    
    # 计算正则化项
    regularized_theta = (lamd / len(X)) * theta_one2n
    
    # 加上 theta_0
    regularized_term = np.concatenate([np.array([0]), regularized_theta])
    
 		# 返回增加正则化后的总梯度
    return gradient(theta, X, y) + regularized_term 
```



上面代码中的 lamb 就是正则化系数 $\lambda$ ，在模型训练你可以试着更改它，然后测试下这个系数对模型参数的影响，以此来更加深入地理解正则化技术。



文中**完整可运行代码**链接：



[https://github.com/DLonng/AI-Notes/blob/master/MachineLearning/code/ex2-logistic-regression/my_logistic_regular.ipynb](https://github.com/DLonng/AI-Notes/blob/master/MachineLearning/code/ex2-logistic-regression/my_logistic_regular.ipynb)



OK！今天就分享这些，大家下期见，记得持续关注我哦 :）


> {{ site.prompt }}

<div  align="center">
<img src="https://dlonng.com/images/wechart.jpg" width = "200" height = "200"/>