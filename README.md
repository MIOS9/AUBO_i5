# AUBO_i5
Configuration guide for AUBO_I5 (ROS and PYTHON SDK)

# ROS(Noetic) 安装指南

## setup 1. 安装 ROS(Noetic)
通过鱼香ROS一键配置ROS
`wget http://fishros.com/install -O fishros && . fishros`

## setup 2. 安装 anaconda
### 2.1 下载 anaconda  
Anaconda下载地址(清华镜像)：<https://mirrors.tuna.tsinghua.edu.cn/anaconda/archive/> 
### 2.2 安装 anaconda  
```
bash Anaconda2-4.3.0-Linux-x86_64.sh
```
* 进入注册信息页面，输入yes   
* 然后一直按回车，直到再次出现输入框，然后输入yes。  
* 确认好anaconda3的安装路径后，回车安装。  
**安装完成后，收到加入环境变量的提示信息，输入no**  
### 2.3 设置手动激活conda的工作空间  
```
sudo gedit ~/.bashrc
```  
在文件最后添加  
```
alias setconda='. ~/anaconda3/bin/activate'
```
这样，终端默认使用ROS的python路径，在需要时可以通过`setconda`来进入conda工作空间。

## setup 3. 配置 AUBO 工作空间
```
mkdir -p ~/aubo_ws/src
```
创建一个工作空间，并进入src文件夹
```
cd ~/aubo_ws/src
```
