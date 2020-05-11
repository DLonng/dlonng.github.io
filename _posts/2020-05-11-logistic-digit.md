---
title: 从 0 开始机器学习 - 逻辑回归识别手写字符！
date: 2020-05-11 15:54:00
---
# 从 0 开始机器学习 - 逻辑回归识别手写字符！
***
> 版权声明：本文为 {{ site.name }} 原创文章，可以随意转载，但必须在明确位置注明出处！

之前的逻辑回归文章：[从 0 开始机器学习 - 逻辑回归原理与实战！](https://dlonng.com/posts/logistic-regression)跟大家分享了逻辑回归的基础知识和分类一个简单数据集的方法。

今天登龙再跟大家分享下如何使用逻辑回归来分类手写的 [0 - 9] 这 10 个字符，数据集如下：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/logistic_0-9.png)

下面我就带着大家一步一步写出关键代码，完整的代码在我的 Github 仓库中：[logistic_reg](https://github.com/DLonng/AI-Notes/tree/master/MachineLearning/ex3-neural-network/logistic_reg)

## 一、加载手写字符数据

### 1.1 读取数据集

```python
raw_X, raw_y = load_data('ex3data1.mat')

# 5000 x 400
print(raw_X.shape)

# 5000
print(raw_y.shape)
```

数据集有 5000 个样本，每个样本是一个 20 x 20  = 400 像素的手写字符图像：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/20x20.png)

这个识别手写字符问题属于有监督学习，所以我们有训练集的真实标签 y，维度是 5000，表示训练集中 5000 个样本的真实数字：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/rawXy.jpg)

### 1.2 添加全 1 向量

老规矩，在训练样本的第一列前添加一列全 1 的向量（为了与 $\theta_0$ 相乘进行向量化表示）：

```python
# 添加第一列全 1 向量
X = np.insert(raw_X, 0, values = np.ones(raw_X.shape[0]), axis = 1)
# 5000 行 401 列
X.shape
```

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/add_ones.jpeg)

添加一列后，样本变为 **5000 行 401 列**。

### 1.3 向量化标签

把原标签（5000 行 1 列）变为（5000 行 10 列），相当于把每个真实标签用 10 个位置的向量替换：

```python
# 把原标签中的每一类用一个行向量表示
y_matrix = []

# k = 1 ... 10
# 当 raw_y == k 时把对应位置的值设置为 1，否则为 0
for k in range(1, 11):
    y_matrix.append((raw_y == k).astype(int))
```

改变后向量标签的每一行代表一个标签，只不过用 10 个位置来表示，比如数字 1 对应第一个位置为 1，数字 2 对应第二个位置为 1 ，以此类推，不过注意**数字 0 对应第 10 个位置为 1**：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/raw_y_to_10.jpg)

而每一列代表原始标签值中所有相同的字符，比如第一列表示所有数字 1 的真实标签值，第二列表示所有数字 2 的真实标签值，以此类推，第 10 列表示数字 0 的真实值：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/raw_y_colum.jpeg)

因为我们加载的是 `.mat` 类型的 Matlab 数据文件，而 Matlab 中索引是从 1 开始的，因此原数据集中用第 10 列表示数字 0，但是为了方便 Python 处理，我们这里把第 10 列表示的数字 0 移动到第一列，使得列数按照数字顺序 [0 - 9] 排列：

```python
# 因为 Matlab 下标从 1 开始，所以 raw_y 中用 10 表示标签 0
# 这里把标签 0 的行向量移动到第一行
y_matrix = [y_matrix[-1]] + y_matrix[:-1]
```

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/cloum_10_to_1.jpg)

这是原实验配的图片，原理是一样的，可以对比理解下（这里没有移动第 10 列哦）：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/raw_y_vec2.png)

为何要这样做呢？主要是为了完成后面一次预测多个数字的任务。

## 二、训练模型

逻辑回归和正则化的原理之前都讲过了，没看过的同学可以复习下：

- [从 0 开始机器学习 - 逻辑回归原理与实战！](https://dlonng.com/posts/logistic-regression)
- [从 0 开始机器学习 - 正则化技术原理与编程!](https://dlonng.com/posts/regularization)

这里我就直接放关键的函数，然后稍加解释下。

### 2.1 逻辑回归假设函数

假设函数使用常用的 sigmoid 函数：


$$
g(z)=\frac{1}{1+{e^{-z}}}
$$




```python
def sigmoid(z):
    return 1 / (1 + np.exp(-z))
```

### 2.2 逻辑回归代价函数


$$
J(\theta) = -\frac{1}{m}\sum\limits_{i = 1}^{m}{[{y^{(i)}}\log ({h_\theta}({x^{(i)}}))+( 1-{y^{(i)}})\log ( 1 - h_\theta({x^{(i)}}))]}
$$


```python
def cost(theta, X, y):
    return np.mean(-y * np.log(sigmoid(X @ theta)) - (1 - y) * np.log(1 - sigmoid(X @ theta)))
```

### 2.3 逻辑回归正则化代价函数


$$
J(\theta) = \frac{1}{m}\sum\limits_{i = 1}^{m}{[-{y^{(i)}}\log ({h_\theta}({x^{(i)}}))-( 1-{y^{(i)}})\log ( 1 - h_\theta({x^{(i)}}))]} + \frac{\lambda}{2m} \sum\limits_{j=1}^{n}{\theta_j^2}
$$


```python
def regularized_cost(theta, X, y, l=1):
    theta_j1_to_n = theta[1:]
    
    # 正则化代价
    regularized_term = (l / (2 * len(X))) * np.power(theta_j1_to_n, 2).sum()

    return cost(theta, X, y) + regularized_term
```

### 2.4 梯度计算


$$
J( \theta_0, \theta_1)' = 2 * \frac{1}{2m}\sum\limits_{i=1}^m \left( h_{\theta}(x_i)-y^{(i)} \right) * h_\theta(x_i)'
$$

$$
h_\theta(x_i)' = (\theta_0x_0 + \theta_1x_1)' = x_i
$$


```python
def gradient(theta, X, y):
    return (1 / len(X)) * X.T @ (sigmoid(X @ theta) - y)
```

### 2.5 正则化梯度

在原梯度后面加上正则化梯度即可：


$$
\frac{\lambda}{m} \sum\limits_{j=1}^{n}{\theta_j}
$$


```python
def regularized_gradient(theta, X, y, l=1):
    theta_j1_to_n = theta[1:]
    
    # 正则化梯度
    regularized_theta = (l / len(X)) * theta_j1_to_n

    # 不对 theta_0 正则化
    regularized_term = np.concatenate([np.array([0]), regularized_theta])

    return gradient(theta, X, y) + regularized_term
```

### 2.6 逻辑回归训练函数

使用 scipy.optimize 来优化：

```python
"""逻辑回归函数
    args:
        X: 特征矩阵, (m, n + 1)，第一列为全 1 向量
        y: 标签矩阵, (m, )
        l: 正则化系数

    return: 训练的参数向量
"""
def logistic_regression(X, y, l = 1):
    # 保存训练的参数向量，维度为特征矩阵的列数，即特征数 + 1
    theta = np.zeros(X.shape[1])

    # 使用正则化代价和梯度训练
    res = opt.minimize(fun = regularized_cost,
                       x0 = theta,
                       args = (X, y, l),
                       method = 'TNC',
                       jac = regularized_gradient,
                       options = {'disp': True})
    
    # 得到最终训练参数
    final_theta = res.x

    return final_theta
```

## 三、训练模型

我们先来训练模型，使得能识别单个数字 0，`y[0]` （5000 行 1 列）代表所有真实标签值为 0 的样本，参考之前讲的向量化标签：

```python
theta_0 = logistic_regression(X, y[0])
```

预测的结果 `theta_0`（401 行 1 列） 是手写字符 0 对应的参数向量。

## 四、预测训练集数字 0

我们用训练的 theta_0 参数来预测下训练集中**所有的**字符图像为 0 的准确度：

```python
def predict(x, theta):
    prob = sigmoid(x @ theta)
    return (prob >= 0.5).astype(int)
```

```python
# 字符 0 的预测值，也是 5000 行 1 列
y_pred = predict(X, theta_0)
```

`y_pred` 是 5000 行 1 列的向量，元素只有 0 和 1，1 表示样本预测值为数字 0，0 表示预测值不是数字 0。

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/y_pred_0.jpeg)

我们再把预测值和真实值进行比较，计算下误差的平均值作为输出精度：

```python
# 打印预测数字 1 的精度
print('Accuracy = {}'.format(np.mean(y[0] == y_pred)))

Accuracy = 0.9974
```

显示该模型识别训练集中手写数字 0 的图像正确率约为 99.74%！这只是分类一个数字，下面再来一次把 10 个数字都进行分类。

## 五、分类 10 个数字

上面只训练并预测了一个字符 0，我们可以使用 for 循环来训练全部的 10 个字符，每个字符的训练方法都和上面单个数字相同：

```python
# 训练 0 - 9 这 10 个类别的 theta_[0 -> 9] 参数向量
theta_k = np.array([logistic_regression(X, y[k]) for k in range(10)])
```

theta_k 是 10 个数字对应的参数向量（每行代表一个参数向量）：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/theta_k.jpg)

```python
# 10 行 401 列
print(theta_k.shape)
```

对特征矩阵进行预测，注意这里对 theta_k 进行了转置，是为了进行矩阵的乘法运算：

```python
# X(5000, 401), theta_k.T(401, 10)
prob_matrix = sigmoid(X @ theta_k.T)

# prob_matrix(5000, 10)
prob_matrix
```

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/pro_matrix.jpg)

打印下预测的矩阵（5000 行 10 列）：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/pro_matrix.png)

将每行中的最大一列的索引放入 y_pred 中，用来表示预测的数字：

```python
y_pred = np.argmax(prob_matrix, axis = 1)

# (5000, 1)
print(y_pred.shape)

y_pred
```

此时的 y_pred 变为 5000 行 1 列，每一行就是模型预测的数字识别结果，再把真实标签中的 10 替换为 0：

```python
# 用 0 代替 10
y_answer[y_answer == 10] = 0
```

打印出训练集中每个手写数字的预测精度：

```python
print(classification_report(y_answer, y_pred))
```

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/answer_pred.png)

可以看到每个字符在训练集上的预测效果都能达到 90% 以上，说明这个模型在训练集上的预测效果比较不错。

OK，今天就分享这些，希望大家多多实践！完整可运行代码链接：[logistic_reg](https://github.com/DLonng/AI-Notes/tree/master/MachineLearning/ex3-neural-network/logistic_reg)

学会了记得回来给我一个 Star 哦 (*^▽^*)！


> {{ site.prompt }}





![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/dlonng_qrcode.jpg#pic_center)