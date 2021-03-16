---
title: Window10 安装 jekyll 简单步骤
date: 2021-03-15 20:00:00
---
# Window10 安装 jekyll 简单步骤
***
> 版权声明：本文为 {{ site.name }} 原创文章，可以随意转载，但必须在明确位置注明出处！



## 官方教程

- https://jekyllrb.com/docs/installation/windows/

## 下载安装包 2.4.10

官方吧 ruby 和 devkit 合成一个安装包了，不用分开安装：

- https://rubyinstaller.org/downloads/

## 安装

一路 next，选择自己的目录

最后一步安装 ridk install，弹出黑窗口选择 3 回车。

## 配置 gem

```
gem install jekyll bundler
```

期间出现 RubyGem 的版本错误，按照提示更新 Gem

```
gem update --system
```

## 启动博客

```
bundle exec jekyll serve -w
```

第一次启动提示缺少部分库：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/20210316104401.png)

使用下面的命令安装所有缺少的库：

```
bundle install
```

再次启动成功：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/20210316104509.png)






























> {{ site.prompt }}



![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/dlonng_qrcode.jpg#pic_center)