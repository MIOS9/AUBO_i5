# AUBO i5 ROS(Noetic) 安装指南

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



