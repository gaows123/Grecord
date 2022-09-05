# 创建ros功能包--用于测试global_planner plugin

参考资料：

http://www.autolabor.com.cn/book/ROSTutorials/di-5-zhang-ji-qi-ren-dao-hang/51-tfzuo-biao-bian-huan.html

https://github.com/xyf27323/hybrid_astar_planner/tree/main/test_the_plugin

## 1.导航之坐标系

### 1.1 map地图坐标系

一般改坐标系为固定坐标系(fixed frame)，一般与机器人坐在的世界坐标系一致。建图过程中，以机器人起点作为map坐标系的原点。

### 1.2 base_link机器人本体(基座)坐标系

与机器人中心重合，坐标原点一般为机器人旋转中心。

### 1.3 base_footprint坐标系

base_footprint表示机器人base_link原点在地面上的投影，区别base_link之处是其“z”坐标不同。
一般为了模型不陷入地面，base_footprint的“z”坐标比base_link高。

### 1.4 odom坐标系

odom坐标系的原点就是机器人上电时所处的位置，所以在建图时odom坐标系和map坐标系是重合的，后面随着建图的进行，由于误差存在，odom坐标系和map坐标系会慢慢偏离。 而导航时，机器人起始位置一般不会与建图时的起始位置完全相同，而map坐标系是始终不变的，所以在导航开始时，odom坐标系和map坐标系一般都不是重合的

当里程计和传感器同时使用时，map和odom都是机器人模型根坐标系的父级，不符合坐标变换中“单继承”的原则，所以一般将转换关系设置为：map->odom->base_link或base_footprint.

### 1.5 base_laser

激光雷达的坐标系，与激光雷达的安装点有关，其与base_link的tf为固定的.

## 2.tf坐标变换

ROS中坐标系为右手坐标系，食指指向x轴，中指指向y轴

在ROS中坐标变换最初对应的是tf，不过在 hydro 版本开始, tf 被弃用，迁移到 tf2,后者更为简洁高效，tf2对应的常用功能包有:

tf2_geometry_msgs:可以将ROS消息转换成tf2消息。

tf2: 封装了坐标变换的常用消息。

tf2_ros:为tf2提供了roscpp和rospy绑定，封装了坐标变换常用的API。

### 2.1 坐标msg消息

msg:`geometry_msgs/TransformStamped`和`geometry_msgs/PointStamped`，前者用于传输坐标系相关位置信息，后者用于传输某个坐标系内坐标点的信息。

### 2.1静态坐标变换

所谓静态坐标变换，是指两个坐标系之间的相对位置是固定的。

参考：http://www.autolabor.com.cn/book/ROSTutorials/di-5-zhang-ji-qi-ren-dao-hang/51-tfzuo-biao-bian-huan/512-jing-tai-zuo-biao-bian-huan.html

发布方：相当于设置好坐标变换关系（父和子的关系），然后广播器发布出去，随时被调用。

接收方：生成一个坐标点(相对于子级坐标系)，转换到父级坐标系下。TF相当于一个桥梁，给定父级坐标系的名称，就可以根据发布方广播出去的坐标关系进行转换。

### 2.2 动态坐标变换

所谓动态坐标变换，是指两个坐标系之间的相对位置是变化的。

参考：http://www.autolabor.com.cn/book/ROSTutorials/di-5-zhang-ji-qi-ren-dao-hang/51-tfzuo-biao-bian-huan/513-dong-tai-zuo-biao-bian-huan.html

### 2.3 多坐标变换

一个父级两个儿子，已知大儿子相对于父亲的关系，二儿子相对于父亲的关系，求大儿子在二儿子中的坐标。

（1）首先创建相对关系发布方(需要发布两个坐标相对关系)，采用static_transform_publisher，可以代码发布，也可以直接launch文件发布。

（2）解析son1中相对son2的坐标

```
buffer.lookupTransform("son2","son1",ros::Time(0));
```

（3）坐标点解析

```
 psAtSon2 = buffer.transform(ps,"son2");
```

## 1.创建功能包plugin_test

### 1.1 创建工作空间并初始化

```
mkdir -p global_plugin_test/src
cd global_plugin_test
catkin_make
```

### 1.2 创建功能包

```
cd src
catkin_create_pkg plugin_test roscpp rospy std_msgs
```

## 2.编写代码

