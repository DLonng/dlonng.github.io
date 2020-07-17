---
title: Ubuntu 16.04 解决机械硬盘挂载错误问题！
date: 2020-07-17 12:50:00
---
# Ubuntu 16.04 解决机械硬盘挂载错误问题！
***
> 版权声明：本文为 {{ site.name }} 原创文章，可以随意转载，但必须在明确位置注明出处！

新买了台电脑，但是只有 100G 固态，所以需要给主机加装了一个机械硬盘，但是装好后到 Ubuntu 内不能正确挂载：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/ubable_access_yingpan.png)

网上找了下方法，发现直接用 `ntfsfix` 工具修复即可，先安装它：

```shell
sudo apt-get install ntfs-3g
```

然后修复从挂载报错信息中查看要修复的硬盘分区，比如修复 `/dev/sdb1`，执行：

```
sudo ntfsfix /dev/sdb1
```

执行完即可正确挂载了，完美。

参考链接：

- https://askubuntu.com/questions/462381/cant-mount-ntfs-drive-the-disk-contains-an-unclean-file-system
- https://blog.csdn.net/lanxuezaipiao/article/details/25137351


> {{ site.prompt }}



![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/dlonng_qrcode.jpg#pic_center)