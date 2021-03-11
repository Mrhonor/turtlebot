# assignment5 下位机节点

**usage**:

启动turtlebot节点
```
roslaunch turtlebot_bringup minimal.launch
```


启动机器人的下位机节点
```
//如果没有catkin_make和source请先编译并source devel/setup.bash，如果是zsh环境改用source devel/setup.zsh
roslaunch aruco_listener aruco_listener.launch robot:=robot1
```

启动rviz可以观看轨迹，frameid选base，topic选/robot1/aruco_listener/trajectory
