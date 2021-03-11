# assignment6 下位机节点

**usage**:


启动turtlebot和摄像头节点
```
roslaunch turtlebot_bringup minimal.launch
roslaunch freenect_launch freenect-registered-xyzrgb.launch
```
启动机器人的下位机节点
```
//如果没有catkin_make和source请先编译并source devel/setup.bash，如果是zsh环境改用source devel/setup.zsh
roslaunch aruco_listener aruco_listener.launch
```
