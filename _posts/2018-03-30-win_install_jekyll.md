---
title: Win 7 Install Jekyll
date: 2018-03-30 11:46:00
---

# Win 7 Install Jekyll
之前一直在用 ubuntu，但是因为毕设要求最后还是换回了 win 7 x64bit，自己的博客环境也要重新配置一下，这里记录下配置的基本步骤。

## Install Ruby
我安装的 ruby 版本是 [Ruby 2.3.3 (x64)](https://rubyinstaller.org/downloads/)，官网建议安装 2.4.x 版本，但是 2.4.x 版本需要下载另一个 MSVCToolkit，因为我的下载网速很慢，于是就安装了 2.3.3 的 x64 版本，但是也是可以正常使用的，不一定要选择新的版本。

安装要注意 2 点：

1. 默认路径安装，我安装在 `C:\Ruby23-x64`，ruby 安装完不是很大;
2. 勾选加入 PATH 环境变量，必须勾上：

	![Install Ruby]({{ site.url }}/images/install_jekyll/install_ruby.png)

## Install Devkit
下载 [DevKit-mingw64-64-4.7.2-20130224-1432-sfx](https://rubyinstaller.org/downloads/)，然后解压出来，我解压到 `D:\Devkit`，在 windows cmd 命令行（win + R）中执行下面的命令：

	cd Devkit 目录
	ruby dk.rb init

使用记事本打开 config.yml，在最后面加上 ruby 的安装目录 `- C:\Ruby23-x64`。

	notepad config.yml

**特别注意**：前面有个 `-`，后面还有个空格：

![ruby_install_devkit]({{ site.url }}/images/install_jekyll/ruby_install_devkit.png)

最后再执行：

	ruby dk.rb install

Devkit 安装完成

## Install Jekyll
启动 Windows cmd，查看 gem 是否安装成功，再安装 jekyll：

	# 确人 gem 已经安装
	gem -v
	# 开始安装
	gem install jekyll

等待安装完成。

## Install Bundle
[我的博客](http://cdeveloper.cn/)基于 jekyll，启动需要安装 bundler:

	cd 博客目录下
	gem install bundler
	bundle install

需要等待安装完成，我安装的时候没有出错，建议你安装的时候**科学上网**。

## 启动 jekyll

	bundle exec jekyll serve -w

出现下面的信息，表示 jekyll 启动成功：
![jekyll_start]({{ site.url }}/images/install_jekyll/jekyll_start.png)


浏览器打开：[http://localhost:4000/](http://localhost:4000/) 即可查看自己的博客啦！

## 最后
话说，我安装过程中一个错误也没有，灰常顺利 ==，如果你出现安装错误，建议 Google 解决问题，祝你安装成功。

> {{ site.prompt }}
