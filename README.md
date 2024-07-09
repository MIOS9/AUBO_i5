# AUBO i5 ROS(Noetic) Fixed 安装指南

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
### 2.2 克隆本仓库被工程师修补的驱动  
创建一个工作空间，并进入src文件夹
```
mkdir -p ~/aubo_ws/src
cd ~/aubo_ws/src
```
克隆官方仓库地址：`https://github.com/AuboRobot/aubo_robot`
```
git clone https://github.com/AuboRobot/aubo_robot.git
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
### 2.4 clone 本仓库，将lib64中的aubocontroller下的所有文件删除，用dependents替换。
### 2.5 软连接 libaral.so
```
sudo gedit /etc/ld.so.conf
```
在文件的最后一行加上`/home/ipc-robot/aubo_ws/src/aubo_robot/aubo_robot/aubo_driver/lib/lib64/aubocontroller`，其中ipc-robot替换为你的用户名。
```
sudo /sbin/ldconfig -v
```
### 2.6 编译工作空间
```
cd ~/aubo_ws
catkin_make
```



