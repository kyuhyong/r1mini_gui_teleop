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
$ sudo apt install -y libqt4-dev ros-melodic-qt-build ros-melodic-qt-gui
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

## Trouble shooting

### Parse error at "BOOST_JOIN"

If error like this happens, refer to below document.
https://answers.ros.org/question/233786/parse-error-at-boost_join/

Edit has_binary_operator.hpp by

```
$ sudo gedit /usr/include/boost/type_traits/detail/has_binary_operator.hpp
```

And try to find code below

```
namespace BOOST_JOIN(BOOST_TT_TRAIT_NAME,_impl) {
...
}
```
And change as 
```
#ifndef Q_MOC_RUN
namespace BOOST_JOIN(BOOST_TT_TRAIT_NAME,_impl) {
#endif

....

#ifndef Q_MOC_RUN
}
#endif

```

### Error with cv_bridge

$ sudo nano /opt/ros/melodic/share/cv_bridge/cmake/cv_bridgeConfig.cmake

Change /usr/include/opencv to /usr/**local**/include/opencv

```
if(NOT "include;/usr/include;/usr/include/opencv " STREQUAL " ")
  set(cv_bridge_INCLUDE_DIRS "")
  set(_include_dirs "include;/usr/include;/usr/local/include/opencv")
  if(NOT "https://github.com/ros-perception/vision_opencv/issues " STREQUAL " ")
    set(_report "Check the issue tracker 'https://github.com/ros-perception/vis$
  elseif(NOT "http://www.ros.org/wiki/cv_bridge " STREQUAL " ")
    set(_report "Check the website 'http://www.ros.org/wiki/cv_bridge' for info$
  else()
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
