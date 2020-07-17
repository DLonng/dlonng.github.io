---
title: Ubuntu 16.04 安装 sublime-text3！
date: 2020-07-17 12:38:00
---
# Ubuntu 16.04 安装 sublime-text3！
***
> 版权声明：本文为 {{ site.name }} 原创文章，可以随意转载，但必须在明确位置注明出处！

我工作中频繁使用 sublime，目前的版本有点低，看起来很丑，所以简单地升级下，记录下步骤：

### 1. 卸载旧版本

```shell
sudo apt remove sublime-text
```

### 2. 安装新版本

```shell
wget -qO - https://download.sublimetext.com/sublimehq-pub.gpg | sudo apt-key add -
```

```shell
echo "deb https://download.sublimetext.com/ apt/stable/" | sudo tee /etc/apt/sources.list.d/sublime-text.list
```

```shell
sudo apt update
```

```shell
sudo apt install sublime-text
```

搞定！

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/sublime-text3-update.png)




> {{ site.prompt }}



![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/dlonng_qrcode.jpg#pic_center)