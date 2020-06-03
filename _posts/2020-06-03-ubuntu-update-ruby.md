---
title: Ubuntu16.04 更新 ruby-2.6
date: 2020-06-03 20:17:00
---
# Ubuntu16.04 更新 ruby-2.6！
***
> 版权声明：本文为 {{ site.name }} 原创文章，可以随意转载，但必须在明确位置注明出处！

博客预览需要用 ruby，当前是 2.3 版本：

```shell
ruby -v
```

需要更新 2.6，方法如下，先添加仓库：

```shell
sudo add-apt-repository ppa:brightbox/ruby-ng
sudo apt-get update
```

删除低版本的 ruby：

```shell
sudo apt-get purge --auto-remove ruby
```

安装 ruby-2.6 版本：

```shell
sudo apt-get install ruby2.6 ruby2.6-dev
```

搞定！


> {{ site.prompt }}



![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/dlonng_qrcode.jpg#pic_center)