# r1mini_gui_teleop

## Description

This project is about demonstrating how to implement GUI based controller for R1mini robot.

## Installation

### ROS QT Installation

Follow instruction below to install ROS-QT.
https://ros-qtc-plugin.readthedocs.io/en/latest/_source/How-to-Install-Users.html

### Additional packages

Additional packages can be apt intalled

```
$ sudo apt install ros-melodic-qt-gui
$ sudo apt install ros-melodic-qt-gui-app
```

## catkin make

If your catkin workspace is /catkin_ws under /home folder, cd to the src folder
```
$ cd ~/catkin_ws/src
```
Then git clone this source
```
$ git clone <this source link>
```
Back to the /cakin_ws folder and try catkin_make from there.
```
$ catkin_make
```
If there is no error, you can run this project by
```
$ rosrun r1mini_gui_teleop r1mini_gui_teleop
```
Don't forget to run roscore before run any ROS packages.
And you will see below window.

![main_window](img/main_window.png)

Enjoy!

Project r1mini_gui_teleop written by Kyuhyong
2021(c)OMOROBOT INC.
