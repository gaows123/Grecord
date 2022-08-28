# ROS navigation stack

# 1.navigation梳理

## 1.1 navigation 概述

![img](navigation学习.assets/move_baseaction=AttachFile&do=get&target=overview-16458590925312-16458611036814.png)

**核心：**

- move_base包

**输入：**

- `/tf`:`map_frame`、`odom_frame`、`base_frame`以及机器人各关节之间的完成的一棵tf树。
- `/odom`:里程计信息
- `/scan`或`/pointcloud`:传感器的输入信息，最常用的是激光雷达(sensor_msgs/LaserScan类型)，也有用点云数据(sensor_msgs/PointCloud)的。
- `/map`：地图，可以由SLAM程序来提供，也可以由`map_server`来指定已知地图。

以上四个Topic是必须持续提供给导航系统的，下面一个是可随时发布的topic：

- `move_base_simple/goal`:目标点位置。

**输出：**

- `/cmd_vel`:`geometry_msgs/Twist`类型，为每一时刻规划的速度信息。

包含的package:

|          包名          |                             功能                             |
| :--------------------: | :----------------------------------------------------------: |
|          amcl          |                             定位                             |
|   fake_localization    |                             定位                             |
|       map_server       |                           提供地图                           |
|       move_base        |                         路径规划节点                         |
|        nav_core        | 路径规划的接口类，包括base_local_planner、base_global_planner和recovery_behavior三个接口 |
|   base_local_planner   |        实现了Trajectory Rollout和DWA两种局部规划算法         |
|   dwa_local_planner    |                  重新实现了DWA局部规划算法                   |
|     parrot_planner     |                  实现了较简单的全局规划算法                  |
|         navfn          |                实现了Dijkstra和A*全局规划算法                |
|     global_planner     |              重新实现了Dijkstra和A*全局规划算法              |
| clear_costmap_recovery |                 实现了清除代价地图的恢复行为                 |
|    rotate_recovery     |                     实现了旋转的恢复行为                     |
|  move_slow_and_clear   |                   实现了缓慢移动的恢复行为                   |
|       costmap_2d       |                         二维代价地图                         |
|       voxel_grid       |                             三维                             |
|     robot_pose_ekf     |                    机器人位姿的卡尔曼滤波                    |

## 1.2 nav_core

nav_core包提供了机器人导航的通用接口，现在主要提供了**BaseGlobalPlanner**,**BaseLocalPlanner**

和**RecoveryBehavior**接口。所有希望用做move_base插件的规划器和修复器都必须继承自这些接口。

接口的作用一般是为了统一不同规划器的输出、输入，使得后续程序可以适应不同规划器。

![nav_core](navigation学习.assets/nav_core-16458701697078.png)

### 1.2.1 BaseGlobalPlanner

使用此接口的全局规划器有：

- [global_planner](http://wiki.ros.org/global_planner) - 重新实现了navfn. (pluginlib name: "global_planner/GlobalPlanner")
- [navfn](http://wiki.ros.org/navfn) - 基于栅格的全局规划器 (pluginlib name: "navfn/NavfnROS")
- [carrot_planner](http://wiki.ros.org/carrot_planner) - 简单的全局规划器，让机器人尽可能的靠近目标点，即使目标点有障碍物 (pluginlib name: "carrot_planner/CarrotPlanner")

#### 1.2.1.1 类继承关系图

![Inheritance graph](navigation学习.assets/classnav__core_1_1_base_global_planner__inherit__graph-16458615399375.png)

#### 1.2.1.2 GlobalPlanner的协作图

![classglobal__planner_1_1_global_planner__coll__graph](navigation学习.assets/classglobal__planner_1_1_global_planner__coll__graph.png)

#### 1.2.1.3 CarrotPlanner的协作图

![classcarrot__planner_1_1_carrot_planner__coll__graph](navigation学习.assets/classcarrot__planner_1_1_carrot_planner__coll__graph.png)

#### 1.2.1.4 Navfn的协作图

![classnavfn_1_1_navfn_r_o_s__coll__graph](navigation学习.assets/classnavfn_1_1_navfn_r_o_s__coll__graph.png)

### 1.2.2 BaseLocalPlanner

使用该接口的局部规划器有：

- [base_local_planner](http://wiki.ros.org/base_local_planner) - 实现了DWA和Trajectory Rollout的local control
- [dwa_local_planner](http://wiki.ros.org/dwa_local_planner) - 重新实现了DWA，更干净易懂
- [eband_local_planner](http://wiki.ros.org/eband_local_planner) -EB算法
- [teb_local_planner](http://wiki.ros.org/teb_local_planner) - TEB算法，在线轨迹优化
- [mpc_local_planner](http://wiki.ros.org/mpc_local_planner) -MPC算法

这些局部规划器需要在各自的源文件中注册成为插件：

```c++
//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(dwa_local_planner::DWAPlannerROS, nav_core::BaseLocalPlanner)
    
//register this planner as a BaseLocalPlanner plugin 注册局部规划器为一个插件
PLUGINLIB_EXPORT_CLASS(base_local_planner::TrajectoryPlannerROS, nav_core::BaseLocalPlanner)
```

#### 1.2.2.1 类继承关系图

下图是navigation工程中的类继承关系，eb,teb,mpc并不在该工程中.

![Inheritance graph](navigation学习.assets/classnav__core_1_1_base_local_planner__inherit__graph-16458615581556.png)

#### 1.2.2.2 TrajectoryPlannerROS的协作图

![classbase__local__planner_1_1_trajectory_planner_r_o_s__coll__graph](navigation学习.assets/classbase__local__planner_1_1_trajectory_planner_r_o_s__coll__graph.png)

#### 1.2.2.3 DWAPlannerRos的协作图

![classdwa__local__planner_1_1_d_w_a_planner_r_o_s__coll__graph](navigation学习.assets/classdwa__local__planner_1_1_d_w_a_planner_r_o_s__coll__graph.png)

### 1.2.3 RecoveryBehavior

使用该接口的修复机制：

- [clear_costmap_recovery](http://wiki.ros.org/clear_costmap_recovery) 
- [rotate_recovery](http://wiki.ros.org/rotate_recovery) 

这些恢复行为也要在各自的源文件中注册成为插件：

```c++
PLUGINLIB_EXPORT_CLASS(clear_costmap_recovery::ClearCostmapRecovery, nav_core::RecoveryBehavior)

// register this planner as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(move_slow_and_clear::MoveSlowAndClear, nav_core::RecoveryBehavior)

// register this planner as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(rotate_recovery::RotateRecovery, nav_core::RecoveryBehavior)
```



#### 1.2.3.1 类继承关系图：



### ![Inheritance graph](navigation学习.assets/classnav__core_1_1_recovery_behavior__inherit__graph-16458616098277.png)

#### 1.2.3.2 ClearCostmapRecovery协作图

![classclear__costmap__recovery_1_1_clear_costmap_recovery__coll__graph](navigation学习.assets/classclear__costmap__recovery_1_1_clear_costmap_recovery__coll__graph.png)



#### 1.2.3.4 RotateRecovery协作图

![classrotate__recovery_1_1_rotate_recovery__coll__graph](navigation学习.assets/classrotate__recovery_1_1_rotate_recovery__coll__graph.png)

#### 1.2.3.5 MoveSlowAndClear协作图

![classmove__slow__and__clear_1_1_move_slow_and_clear__coll__graph](navigation学习.assets/classmove__slow__and__clear_1_1_move_slow_and_clear__coll__graph.png)

## 1.3 move_base

### 1.3.1 概述

move_base类图：

包含两个枚举，一个class

![move_base](navigation学习.assets/move_base.png)

**输入：**

- `/tf`:`map_frame`、`odom_frame`、`base_frame`以及机器人各关节之间的完成的一棵tf树。
- `/odom`:里程计信息
- `/scan`或`/pointcloud`:传感器的输入信息，最常用的是激光雷达(sensor_msgs/LaserScan类型)，也有用点云数据(sensor_msgs/PointCloud)的。
- `/map`：地图，可以由SLAM程序来提供，也可以由`map_server`来指定已知地图。

以上四个Topic是必须持续提供给导航系统的，下面一个是可随时发布的topic：

- `move_base_simple/goal`:目标点位置。

**输出：**

- `/cmd_vel`:`geometry_msgs/Twist`类型，为每一时刻规划的速度信息。

![image-20220226230704499](navigation学习.assets/image-20220226230704499.png)

move_base需要选择插件，包括三种插件：`base_local_planner`、`base_global_planner`和`recovery_behavior`，如果不指定，系统会指定默认值。

**base_local_planner插件：**

- base_local_planner: 实现了Trajectory Rollout和DWA算法
- dwa_local_planner: 实现了DWA算法，base_local_planner的改进版

**base_global_planner插件：**

- carrot_planner: 简单的全局规划算法
- navfn: Dijkstra和A*全局规划算法
- global_planner: 重新实现了Dijkstra和A*全局规划算法，navfn的改进版

**recovery_behavior插件：**

- clear_costmap_recovery: 清除代价地图的恢复行为
- rotate_recovery: 旋转的恢复行为
- move_slow_and_clear: 缓慢移动的恢复行为

### 1.3.2 构造函数

```c++
//订阅rviz下发的目标点
ros::NodeHandle simple_nh("move_base_simple");
goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&MoveBase::goalCB, this, _1));

//下发命令(话题)给基座
vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
current_goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("current_goal", 0 );

//新建一个action server,回调函数是executeCb()
as_ = new MoveBaseActionServer(ros::NodeHandle(), "move_base", boost::bind(&MoveBase::executeCb, this, _1), false);

// 新建planner线程，入口函数为MoveBase::planThread
planner_thread_ = new boost::thread(boost::bind(&MoveBase::planThread, this));

// 初始化全局规划器
try {
    planner_ = bgp_loader_.createInstance(global_planner);
    planner_->initialize(bgp_loader_.getName(global_planner), planner_costmap_ros_);
} catch (const pluginlib::PluginlibException& ex) {
  exit(1);
}

// 创建局部规划器
try {
  tc_ = blp_loader_.createInstance(local_planner);
  ROS_INFO("Created local_planner %s", local_planner.c_str());
  tc_->initialize(blp_loader_.getName(local_planner), &tf_, controller_costmap_ros_);
} catch (const pluginlib::PluginlibException& ex) {
  exit(1);
}

//启动了两个服务
//advertise a service for getting a plan
make_plan_srv_ = private_nh.advertiseService("make_plan", &MoveBase::planService, this);
//advertise a service for clearing the costmaps
clear_costmaps_srv_ = private_nh.advertiseService("clear_costmaps", &MoveBase::clearCostmapsService, this);


//加载恢复行为
if(!loadRecoveryBehaviors(private_nh)){
  loadDefaultRecoveryBehaviors();
}

//动态参数服务器，可以动态修改参数
dsrv_ = new dynamic_reconfigure::Server<move_base::MoveBaseConfig>(ros::NodeHandle("~"));

dynamic_reconfigure::Server<move_base::MoveBaseConfig>::CallbackType cb = boost::bind(&MoveBase::reconfigureCB, this, _1, _2);

dsrv_->setCallback(cb);

```

### 1.3.3  planThread()

全局路径规划器线程

```c++
ROS_DEBUG_NAMED("move_base_plan_thread","Starting planner thread...");
ros::NodeHandle n;
boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
while(n.ok()){
    // 确认是否要运行路径规划器(这里已经加锁)
    while(wait_for_wake || !runPlanner_){
    // 暂时关闭路径规划线程
        ROS_DEBUG_NAMED("move_base_plan_thread","Planner thread is suspending");
        // 注意planner_cond_.wait(lock)是在等待条件满足。
        // 如果条件不满足，则释放锁，将线程置为waiting状态，继续等待；
        // 如果条件满足，则重新获取锁，结束wait，继续向下执行
        planner_cond_.wait(lock);
        wait_for_wake = false;
    }
    ros::Time start_time = ros::Time::now();

    // 该开始规划了，复制路径规划器的目标点(注意这里在上次循环中加锁了)，然后在这次解锁。
    geometry_msgs::PoseStamped temp_goal = planner_goal_;
    lock.unlock();
    ROS_DEBUG_NAMED("move_base_plan_thread","Planning...");

    // 运行路径规划器，它的主要函数是makePlan
    planner_plan_->clear();
    //makePlan()中调用的是planner_->makePlan(start, goal, plan)。
    bool gotPlan = n.ok() && makePlan(temp_goal, *planner_plan_);
}
```

### 1.3.4 executeCb()

 // 控制(局部规划)的主要函数

```c++
void MoveBase::executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal){
	while(n.ok()) {
        //局部规划(导航到目标点)的主要调用executeCycle函数
        bool done = executeCycle(goal);
    }
}
```

```c++
bool MoveBase::executeCycle(geometry_msgs::PoseStamped& goal){
    // step xx move_base状态机，处理导航的控制逻辑
    switch(state_){
      // step x.x 如果是路径规划状态，计算路径
      case PLANNING:
      	// 加锁，唤醒路径规划器线程
      	break;
      // step x.x 如果是控制状态，尝试计算出有效的下发速度命令
      case CONTROLLING:
        if(tc_->computeVelocityCommands(cmd_vel)){}
        break;
	  // step x.x 用提供的恢复行为来清理空间，主要有三个恢复行为
      case CLEARING:
        recovery_behaviors_[recovery_index_]->runBehavior();
        break;
      default;
    }
}
```

## 1.4 RecoveryBehavior子类详解

父类RecoveryBehavior提供恢复行为的接口，navigation stack调用的所有恢复行为模块都要实现这个接口。

<img src="navigation学习.assets/recovery_behavior.png" alt="recovery_behavior" style="zoom:50%;" />

### 1.4.1 ClearCostmapRecovery类

#### 1.4.1.1 initialize()

```c++
// 该插件将costmap中给定半径（reset_distance_默认值3.0）范围之内的区域(正方形)进行清理，即将栅格状态更新为未知信息
PLUGINLIB_EXPORT_CLASS(clear_costmap_recovery::ClearCostmapRecovery, nav_core::RecoveryBehavior)；
//默认是3.0米的正方形，重置为未知状态
private_nh.param("reset_distance", reset_distance_, 3.0);
//清理的是障碍物层
clearable_layers_default.push_back( std::string("obstacles") );
```

#### 1.4.1.2 runBehavior()

```c++
//调用clear()函数进行清理
if (affected_maps_ == "global" || affected_maps_ == "both")
{
    clear(global_costmap_);
}

if (affected_maps_ == "local" || affected_maps_ == "both")
{
    clear(local_costmap_);
}
```

#### 1.4.1.3 clear()

```c++
//获取地图插件
std::vector<boost::shared_ptr<costmap_2d::Layer> >* plugins = costmap->getLayeredCostmap()->getPlugins();
//获取机器人位姿
double x = pose.pose.position.x;
double y = pose.pose.position.y;
//遍历插件地图
for (pluginp = plugins->begin(); pluginp != plugins->end(); ++pluginp) {
	clearMap(costmap, x, y);
}
```

#### 1.4.1.4 clearMap()

将正方形区域的栅格更新为**未知信息**

### 1.4.2 RotateRecovery类

#### 1.4.2.1 initialize()

```c++
// 默认会模拟仿真每一度(degree)旋转的情况,0.017弧度
private_nh.param("sim_granularity", sim_granularity_, 0.017);
private_nh.param("frequency", frequency_, 20.0);
```

#### 1.4.2.2 runBehavior()

### 1.4.3 MoveSlowAndClear类

将正方形区域的栅格更新为**自由区域**。所以有碰撞风险，需要速度很慢才行。

#### 1.4.3.1 runBehavior()

```c++
//获取机器人位姿
global_costmap_->getRobotPose(global_pose);
local_costmap_->getRobotPose(local_pose);

//根据pose确定要清除的区域大小(正方形)，clearing_distance_是关键的参数
std::vector<geometry_msgs::Point> global_poly, local_poly;

// 清除全局代价地图中特定区域
std::vector<boost::shared_ptr<costmap_2d::Layer> >* plugins;
plugins= global_costmap_->getLayeredCostmap()->getPlugins();
costmap->setConvexPolygonCost(global_poly, costmap_2d::FREE_SPACE);

// 清除局部代价地图中特定区域
plugins = local_costmap_->getLayeredCostmap()->getPlugins();
costmap->setConvexPolygonCost(local_poly, costmap_2d::FREE_SPACE);

//设置机器人速度
//limit the speed of the robot until it moves a certain distance
setRobotSpeed(limited_trans_speed_, limited_rot_speed_);
limit_set_ = true;
//10Hz的频率去check距离
distance_check_timer_ = private_nh_.createTimer(ros::Duration(0.1), &MoveSlowAndClear::distanceCheck, this);


```

## 1.5 costmap_2d

### 1.5.1 分层地图简介

分为有标准层、新功能层、人机交互层三大类，共七层代价地图。

常用的有三层Inflation,Obstacle,Static。Master是管理这三层的。首先updateBounds更新各层的边界，然后updateValues更新value。

- 完整的分层地图如下：

![image-20220306223841081](navigation学习.assets/image-20220306223841081.png)

- 分层地图更新的过程：

![image-20220306221333703](navigation学习.assets/image-20220306221333703.png)

- 分层地图的分类

```mermaid
graph LR
	A(标准层) --- A1(Static Map Layer)
	A---A2(Obstacles Layer)
	A---A3(Inflation Layer)
	B(新功能层)---B1(Sonar Layer //声呐)
	B---B2(Caution Zones Layer //尽量不去的区域)
	B---B3(Claustrophobic Layer //额外膨胀)
	C(人机交互层)---C1(Proxemic Layer //远离行人)
	C---C2(Hallway Layer //靠左靠右)
	C---C3(Wagon Ruts Layer //人走过的区域代价降低)
	costmap---A
	costmap---B
	costmap---C
```

### 1.5.2 类继承关系

<img src="navigation学习.assets/costmap_2d.png" alt="costmap_2d" style="zoom:50%;" />

### 1.5.3 实现机制
