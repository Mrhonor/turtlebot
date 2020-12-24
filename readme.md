# assignment4 下位机节点

**usage**:

首先请将*assignment4.launch*文件放于turtlebot_gaze/launch文件目录下。

可以通过以下命令找到对应路径
```
roscd turtlebot_gaze/launch
```

启动gazebo仿真器，world_file请改为assignment4_world的路径
```
roslaunch turtlebot_gaze assignment4.launch world_file:=assignment4_world
```
启动3个机器人的下位机节点
```
//如果没有catkin_make和source请先编译并source devel/setup.bash，如果是zsh环境改用source devel/setup.zsh
roslaunch aruco_listener aruco_listener.launch robot:=robot1
roslaunch aruco_listener aruco_listener.launch robot:=robot2
roslaunch aruco_listener aruco_listener.launch robot:=robot3
```
