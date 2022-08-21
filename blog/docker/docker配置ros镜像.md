# docker配置ros镜像

# 一、docker入门问题

## 0.常用命令

```
docker image ls //查看本地镜像
docker image rm 501  //删除本地镜像
docker pull ubuntu:20.04 //拉取镜像
docker run -it --rm ubuntu:20.04 bash //利用镜像创建并启动一个容器
docker container ls -a //查看所有已经创建的包括终止状态的容器
docker rm -f 9e8 //删除一个容器
docker container prune //删除所有处于终止状态的容器
docker stop 9e8 //停止一个容器
docker ps -a //查看所有容器
docker start 9e8 //启动一个已经停止的容器
docker restart 9e8 //重启容器
docker attach 9e8 //进入容器，退出容器终端，容器不停止
docker exec -it 9e8 /bin/bash //推荐docker exec 命令，退出容器终端，容器不停止
```

```
lsb_release -a //查看ubuntu版本

```



## 1.安装问题

假设本机系统是ubuntu18.04。

### (1) 想安装ubuntu20.04

这个是没有桌面的系统：

```bash
docker pull ubuntu:20.04
docker run -it --rm ubuntu:20.04 bash  //利用镜像创建并启动一个容器
```

如果需要带桌面的系统，需要拉取带桌面的镜像：

https://hub.docker.com/r/dorowu/ubuntu-desktop-lxde-vnc

```
docker pull dorowu/ubuntu-desktop-lxde-vnc:focal //focal即20.04

docker run -d --name ubuntu-desktop-lxde-vnc -p 6080:80 -p 5900:5900 -e VNC_PASSWORD=000000 -v /dev/shm:/dev/shm dorowu/ubuntu-desktop-lxde-vnc


```

这里的容器暴露了两个端口

6080：是web版的vnc，可以在浏览器上直接访问桌面环境

```
浏览器输入http://localhost:6080/
或http://127.0.0.1:6080/
```

5900：是使用客户端工具连接的端口

### (2) 想单独安装ros kinetic

安装跟本机匹配的ros,就不需要额外的ubuntu系统了。

```
docker pull osrf/ros:kinetic-desktop-full
docker run -it osrf/ros:kinetic-desktop-full bash
```

但是没有界面，鱼香ros的不知道是怎么实现的，可以在本机显示rviz等。

### (3) 想安装ros noetic

https://hub.docker.com/r/arvinskushwaha/ros-noetic-desktop-vnc

https://blog.csdn.net/robinfoxnan/article/details/125682167

```
docker pull arvinskushwaha/ros-noetic-desktop-vnc:latest
docker run -d  -p 6080:80  -p 5900:5900 --name q11 arvinskushwaha/ros-noetic-desktop-vnc
```

https://hub.docker.com/r/tiryoh/ros-desktop-vnc

```
docker pull tiryoh/ros-desktop-vnc:noetic
docker run -p 6080:80 -p 5900:5900 --name noetic1 --shm-size=1g tiryoh/ros-desktop-vnc
```



### (4) 界面：浏览器 or vnc

## 2.vscode使用

### (1) 本地目录如何跟镜像目录关联

run -v 容器文件挂载

```
docker run -p 6080:80 -p 5900:5900 --name noetic1 --shm-size=1g -v /home/gao/docker_data/noetic1:/home/ubuntu/workspace tiryoh/ros-desktop-vnc
```

### (2) vscode打开容器

左侧docker插件->CONTAINERS->ros-desktop-vnc, 右键“Attach vscode”；

在新的vscode窗口中打开工作目录；

### (3) vscode编译



## 3.制作自己的镜像

# 二、docker-compose



# 十、资源

https://github.com/yeasy/docker_practice

https://www.runoob.com/docker/docker-command-manual.html

https://new.qq.com/omn/20211118/20211118A0BAUO00.html
