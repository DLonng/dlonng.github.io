---
title: 解决 Github 个人博客安全警告的问题
date: 2020-03-30 12:00:00
---
# 解决 Github 个人博客安全警告的问题
***
> 版权声明：本文为 {{ site.name }} 原创文章，可以随意转载，但必须在明确位置注明出处！

之前 Github 提示 Jekyll 博客存在安全漏洞：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/update_jekyll.png)

上网找了解决方法，发现是 jekyll 需要更新了，方法如下：

```shell
# jekyll 默认的源没有代理会很慢，建议换个国内的镜像
# 查看目前的镜像
gem source -l
# 更换镜像
gem sources --add https://gems.ruby-china.com/ --remove https://rubygems.org/
```

也可以直接修改 `Gemfile` 文件来更换源：

```shell
source "https://rubygems.org"
# 替换为
source "https://gems-china.org"
```

然后进入博客根目录，执行 jekyll 更新命令，坐等更新完毕就行了：

```shell
bundle update
```


> {{ site.prompt }}

<div  align="center">
<img src="https://dlonng.com/images/wechart.jpg" width = "200" height = "200"/>