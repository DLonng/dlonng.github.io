---

title: 个人 jekyll 博客自定义配置：代码高亮、公式表格渲染、图片居中阴影
date: 2020-03-29 17:43:00
---
# 个人 jekyll 博客自定义配置：代码高亮、公式表格渲染、图片居中阴影
***
> 版权声明：本文为 {{ site.name }} 原创文章，可以随意转载，但必须在明确位置注明出处！

之前基于 Jekyll 模板的博客图片不能居中也没阴影效果、数学公式和表格也不能渲染、文字排版看起来也不太舒服，代码高亮不是很明显，所以今天找了点代码，简单配置了下，一并解决！

## 一、 图片居中加阴影

第一种方式是在 md 文档内部用居中的标签，这样图片就居中显示了：

```html
<div  align="center">
<img src="img_url"/>
</div>
```

不过这样需要对每个图片都操作一遍，太麻烦了，所以我直接在博客的主题样式文件 `style.css` 中加上了图片居中和添加阴影的效果，看起来比较美观：

```css
.markdown-body img { 
  /* 图片居中 */
  clear: both;
  margin: 0 auto;
  display: block;

  /* 添加图片阴影 */
  box-shadow:0px 1px 4px rgba(0,0,0,0.3),0 0 40px rgba(0,0,0,0.1) inset;
  webkit-box-shadow:0px 1px 4px rgba(0,0,0,0.3),0 0 40px rgba(0,0,0,0.1) inset;
  moz-box-shadow:0px 1px 4px rgba(0,0,0,0.3),0 0 40px rgba(0,0,0,0.1) inset;
  o-box-shadow:0px 1px 4px rgba(0,0,0,0.3),0 0 40px rgba(0,0,0,0.1) inset;
}
```

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/test_center.png)

这是居中加阴影的效果，还不错吧。

## 二、渲染数学公式

写机器学习博客的时候，需要大量使用公式，所以渲染公式很有必要，我在 `include/head.html` 中加上以下代码：

```css
<script src="https://cdn.mathjax.org/mathjax/latest/MathJax.js?config=TeX-AMS-MML_HTMLorMML" type="text/javascript"></script>
<script type="text/x-mathjax-config">
  MathJax.Hub.Config({
    tex2jax: {
      skipTags: ['script', 'noscript', 'style', 'textarea', 'pre'],
      inlineMath: [['$','$']]
    }
  });
</script>
```

这是公式渲染效果，也还可以：


$$
h_{\theta} \left( x \right)={\theta_{0}}+{\theta_{1}}{x_{1}}+{\theta_{2}}{x_{2}}+{\theta_{3}}{x_{3}}
$$

## 三、渲染表格

写博客的时候突然发现表格也没法渲染，所以又到网上找了些代码，然后写到 `style.css` 文件中：

```css
table {
  padding: 0; 
  width: 100%;
}

table tr {
  border-top: 1px solid #cccccc;
  background-color: white;
  margin: 0;
  padding: 0; 
}

table tr:nth-child(2n) {
  background-color: #f8f8f8; 
}

table tr th {
  font-weight: bold;
  border: 1px solid #cccccc;
  text-align: left;
  margin: 0;
  padding: 6px 13px; 
}

table tr td {
  border: 1px solid #cccccc;
  text-align: left;
  margin: 0;
  padding: 6px 13px; 
}

table tr th :first-child, table tr td :first-child {
  margin-top: 0; 
}

table tr th :last-child, table tr td :last-child {
  margin-bottom: 0; 
}
```

这样就能渲染表格了，还可以不是很丑：

|         正规方程          |        梯度下降         |
| :-----------------------: | :---------------------: |
|          不需要           | 需要选择学习率 $\alpha$ |
|       一次运算得出        |      需要多次迭代       |
| 特征数量 < 10000 可以接受 |  特征数量较大也能适用   |
|   只适用于线性回归模型    |     适用于各种模型      |

## 四、调整文字排版

正文部分字间距太小了，看起来很累，行间距也挺小，而且两端也没有对齐，也在 `style.css` 中统一调整下，添加如下代码：

```css
.markdown-body {
  /* 字间距 1 */
  letter-spacing: 1px;

  /* 行间距 1.6 */
  line-height: 1.6;

  /* 文字两端对齐 */
  text-align: justify;
}
```

## 五、代码高亮

之前的代码高亮不太显眼，这次换成 sublime 风格了，就是上面的代码风格，在网上找了下配置步骤：

### 5.1 安装 rouge

我的博客好像默认装过了，就没执行这步：

```shell
gem install kramdown
gem install rouge
```

### 5.2 配置 _config.yml

emmm...，这一步我测试加不加都行，可能我的 jekyll 刚升级过，默认支持这些，如果你的不行的话，就加上试试：

```shell
highlighter: rouge
markdown: kramdown
kramdown:
  input: GFM
  syntax_highlighter: rouge
```

### 5.3 生成 rouge css 风格文件

使用下面的命令生成需要的 css 文件，其实我就执行了这一行代码，可能前面的东西我之前都装过了：

```shell
#                css 主题           生成的文件名
rougify style monokai.sublime > syntax_monokai.css
```

支持的 css 主题可以使用 `help` 命令查看：

```shell
rougify help style
```

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/rouge_style.png)

然后将生成好的 css 文件拷贝到博客的 `style` 样式目录下，再到 `include/head.html` 文件头部引用这个新的 css 文件就行了：

```html
<link href='{{ site.url }}/stylesheets/syntax_monokai.css' rel='stylesheet' type='text/css' />
```

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/head_style.png)

当然你的目录名称可能跟我的不一样，但是基本都差不多的，找到 `style` 目录和 `head.html` 文件基本就对了，现在码字看起来就舒服多了 ^_^。


> {{ site.prompt }}

<div  align="center">
<img src="https://dlonng.com/images/wechart.jpg" width = "200" height = "200"/>