# TurtleBotTelecontroller

https://blog.csdn.net/xiaocainiaodeboke/article/details/51476557

```shell
roscore
roslaunch turtlebot_bringup minimal.launch
rosrun turtle_tele_control tele_control.py
rosrun kinect2_bridge kinect2_bridge _reg_method:=cpu
rosrun turtle_tele_control turtle_tele_broadcast.py
```

