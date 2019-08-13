vim .vimrc:
```
Plugin 'Valloric/YouCompleteMe'
```
启动 vim，安装插件：
```
:PluginInstall
```
安装 YCM：
```
cd .vim/bundle/YouCompleteMe
```
安装必要的工具：
```
// Ubuntu 16.04 and later
sudo apt install build-essential cmake python3-dev

// Ubuntu 14.04:
sudo apt install build-essential cmake3 python3-dev

// Fedora 27 and later:
sudo dnf install cmake gcc-c++ make python3-devel
```

安装 C-Family 的语言支持：
```
python3 install.py --clangd-completer
```
安装成功：
```
xxx
xxx
Done installing Clangd
```