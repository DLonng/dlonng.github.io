---
title: 阿里云 OSS + PicGo 博客图床超详细配置教程！
date: 2020-04-03 18:44:00
---
# 阿里云 OSS + PicGo 博客图床超详细配置教程！
***
> 版权声明：本文为 {{ site.name }} 原创文章，可以随意转载，但必须在明确位置注明出处！

自己经常写博客，总觉得把图片保存在别的平台不太稳，网上找了下方法，配置下自己的阿里云 OSS 图床服务，并使用 PicGo 客户端一键完成图片上传！

以下是详细配置步骤。

## 一、购买并配置阿里云 OSS 服务

### 1.1 购买阿里云对象存储服务 OSS

打开阿里云 OSS 产品链接：[https://www.aliyun.com/product/oss/](https://www.aliyun.com/product/oss/)

点击折扣套餐：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/oss_product.png)

跳转如下界面，基本配置按照默认即可，40 GB 的存储包足够图床使用了，服务比较便宜，我经常写博客，所以直接购买 1 年：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/taocan_oss.png)

这里购买的是存储包，传输图片还需要流量包，默认的流量是按使用量收费的，也可以购买套餐，但因为上传图片不需要太多流量，所以我就不买套餐，也要不了多少钱。



### 1.2 创建 Bucket

存储包买好后，点击主菜单进入**对象存储 OSS**：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/oss_service.png)

点击右上角**创建 Bucket**，这里显示我已经创建好了一个：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/create_bucket.png)

这里填好 Bucket 的名称、存储地区选择自己常驻的地区、标准存储、关闭冗余存储、不开通版本控制：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/config_bucket.png)

注意把读写权限设置为**公共读写**，其他选项保持默认即可，然后点击左下角确定：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/config_bucket2.png)

### 1.2 添加新用户

Bucket 创建好后，我们还需要创建一个新用户来访问我们的图床，先点击用户头像进入访问控制：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/fangwen_ctl.png)

点击用户，创建新用户：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/create_user.png)

填好用户名，选择编程访问，点击确定：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/config_user.png)

创建完成后，**把用户 ID 和 秘钥复制保存到记事本中**，等下配置 PicGo 客户端要使用：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/id_sercret.png)

返回用户列表，点击用户信息后面的添加权限，添加用户对 OSS 的访问权限：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/add_super.png)

点击管理对象存储服务（OSS），点击确定添加完成：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/oss_full.png)



以上阿里云 OSS 服务就配置好了，为了上传图片，我们还需要一个客户端，我用的是 PicGo。

## 二、安装并配置 PicGo

### 2.1 下载 PicGo

PicGo 是开源的，直接到 Github 下载对应平台安装包安装即可：[PicGo](https://github.com/Molunerfinn/PicGo/releases)

### 2.2 配置阿里云 OSS 

安装后打开 PicGo 图床设置，配置阿里云 OSS：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/picgo_aliyoss.png)

- keyId 和 Keysecret：填上刚才创建用户保存的的 id 和秘钥
- 设置存储空间名：就是创建的 Bucket 名称
- 存储区域名可以在 bucket 列表中的访问域名查看：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/oss-cn-shenzhen.png)

- 存储路径：默认 img/，自己也可以在 OSS 后台创建目录
- 网址后缀和自定义域名：不填

填完后，点击确定，也可以设为默认图床。

### 2.3 测试上传图片

拖拽图片或者点击上传图片，上传完毕会自动复制连接格式地址，直接粘贴到博客里就行了，非常方便！

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/upload_oss.png)

在后台可以查看管理上传的图片：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/oss_file.png)



我觉得使用图床才是保存自己图片的长久之计，毕竟是自己的数据，自己保存才最放心！


> {{ site.prompt }}

<div  align="center">
<img src="https://dlonng.com/images/wechart.jpg" width = "200" height = "200"/>