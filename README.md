# AUBO_i5
Configuration guide for AUBO_I5 (ROS and PYTHON SDK)

# ROS(Noetic) 安装指南

## setup 1. 安装 ROS(Noetic)
通过鱼香ROS一键配置ROS
```
wget http://fishros.com/install -O fishros && . fishros
```

## setup 2. 配置 AUBO 工作空间
### 2.1 安装 git
```
sudo apt install git
```
### 2.2 克隆 AUBO_Robot 仓库
```
mkdir -p ~/aubo_ws/src
```
创建一个工作空间，并进入src文件夹，克隆仓库
```
cd ~/aubo_ws/src
git clone https://github.com/AuboRobot/aubo_robot
```  
### 2.3 安装ROS依赖
```
cd ~/aubo_ws/src
sudo rosdep init
rosdep update
```
看到`all required rosdeps installed successfully`后可以利用`rosdep`安装依赖。
```
rosdep install -y --from-paths . --ignore-src --rosdistro noetic -r
```

### 2.4 软连接 libaral.so
```
sudo gedit /etc/ld.so.conf
```
在文件的最后一行加上`/home/ipc-robot/aubo_ws/src/aubo_robot/aubo_robot/aubo_driver/lib/lib64/aubocontroller`，其中ipc-robot替换为你的用户名。

### 2.5 编译工作空间
```
cd ~/aubo_ws
catkin_make
```

# AUBO Python SDK 安装指南 
## setup 1. 安装 anaconda
### 1.1 下载 anaconda  
Anaconda下载地址(清华镜像)：<https://mirrors.tuna.tsinghua.edu.cn/anaconda/archive/> 
### 1.2 安装 anaconda  
```
bash Anaconda2-4.3.0-Linux-x86_64.sh
```
* 进入注册信息页面，输入yes   
* 然后一直按回车，直到再次出现输入框，然后输入yes。  
* 确认好anaconda3的安装路径后，回车安装。  
**安装完成后，收到加入环境变量的提示信息，输入no**  
### 1.3 设置手动激活conda的工作空间  
```
sudo gedit ~/.bashrc
```  
在文件最后添加  
```
alias setconda='. ~/anaconda3/bin/activate'
```
这样，终端默认使用ROS的python路径，在需要时可以通过`setconda`来进入conda工作空间。

## setup 2. 配置 libpyaubo
### 2.1 创建一个conda环境，python版本为3.11。
```
setconda
conda create -n pyaubo python=3.11
```


