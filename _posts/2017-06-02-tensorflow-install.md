---
title: Install TensorFlow On Ubuntu 
date: 2017-06-02 15:00:00
---

# Install TensorFlow On Ubuntu
***
> 版权声明：本文为 {{ site.name }} 原创文章，可以随意转载，但必须在明确位置注明出处！ 

## 什么是 TensorFlow？
`TensorFlow` 是 `Google` 在 `GitHub` 上开源的一个**机器学习**的平台，用它可以来学习机器学习相关的技术，最近在 `Ubuntu` 下安装了这个框架，虽然不会搞人工智能，但是多了解一些总没有坏处:)。

## 安装过程
比较庆幸，一路安装没有出现任何问题，一方面因为安装过程比较简单，另一方面因为我的电脑配置了很多的开发工具，所以有些环境本来就有了，就比如 `python`，下面开始。

### 选择 CPU 支持的 TensorFlow 来安装
`TensorFlow` 支持 `CPU` 和 `GPU` 来运行，这里我选择了 `CPU`。

### 选择安装方式
官网建议我们使用 `virtualenv` 来安装 `TensorFlow`，它是一个虚拟的 `Python` 环境。


### 开始安装
安装过程比较简单，都是基于命令行的方式。
#### 安装 pip 和 virtualenv
```
sudo apt-get install python-pip python-dev python-virtualenv 
```

#### 创建 virtualenv 环境
```
mkdir ~/tensorflow
virtualenv --system-site-packages ~/tensorflow
```
#### 激活 virtualenv 环境
```
source ~/tensorflow/bin/activate
```
激活后命令行的标签变为下面的字符：
```
(tensorflow)$
```

#### 在已经激活的环境下安装 tensorflow
**注意**：你需要知道当前你的 `python` 的版本，在另一个命令行键入 `python --version` 来查看当前 `python` 版本。
```
(tensorflow)$ pip install --upgrade tensorflow      # Python 2.7
(tensorflow)$ pip3 install --upgrade tensorflow     # Python 3.n
```

到这里 `tensorflow` 就安装完成了，下面我们来测试一下。


## 测试 TensorFlow

我们首先需要启动 `tensorflow` 的环境：
```
source ~/tensorflow/bin/activate
```

如果你的命令行标签变为：
```
(tensorflow)$
```
则表示启动成功，可以用 `python` 编程了。

#### Hello TensorFlow
我们来用 `python` 编写一个 `Hello TensorFlow` 试试吧，首先启动 `python`
```
(tensorflow)$ python
```

之后就可以在 `python` 下写程序了：
```python
>>> import tensorflow as tf
>>> hello = tf.constant('Hello, TensorFlow!')
>>> sess = tf.Session()
>>> sess.run(hello)
'Hello, TensorFlow!'
```

成功输出 `Hello，TensorFlow`，安装成功 :)

#### 退出 TensorFlow 环境
在 `tensorflow` 环境下键入下面的命令来退出该环境：
```
(tensorflow)$ deactivate  
```


至此，安装结束。
