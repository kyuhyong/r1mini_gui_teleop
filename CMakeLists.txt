##############################################################################
# CMake
##############################################################################
cmake_minimum_required(VERSION 2.8.0)
project(r1mini_gui_teleop)

##############################################################################
# Catkin
##############################################################################
# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS
    qt_build
    roscpp
    std_msgs
    geometry_msgs
    sensor_msgs
    cv_bridge
    image_transport
    message_generation
)
find_package(OpenCV REQUIRED)

add_service_files(
  FILES
  Battery.srv
  Calg.srv
  Color.srv
  Onoff.srv
  ResetOdom.srv
)
generate_messages(DEPENDENCIES std_msgs)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything. 
catkin_package(
    CATKIN_DEPENDS
    message_runtime
    std_msgs
    geometry_msgs
    tf
)

##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml
rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/r1mini_gui_teleop/*.hpp)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# Binaries
##############################################################################
add_executable(r1mini_gui_teleop ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(r1mini_gui_teleop ${QT_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(r1mini_gui_teleop ${${PROJECT_NAME}_EXPORTED_TARGETS})
install(TARGETS r1mini_gui_teleop RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

