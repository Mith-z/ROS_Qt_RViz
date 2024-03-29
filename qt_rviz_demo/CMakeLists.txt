##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(qt_rviz_demo)

#add
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(TARGET_NAME qt_rviz_demo)

#添加c++11的支持
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")

#添加python支持
#include_directories(/usr/include/python2.7)
#link_directories(/usr/lib/python2.7/config-x86_64-linux-gnu)
#find_package(PythonLibs 2.7 REQUIRED)


##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS rviz roscpp rospy sensor_msgs
    cv_bridge
    std_msgs
    image_transport
    pcl_ros
    rosbag
    )

set(PCL_INCLUDE_DIRS /usr/include/pcl-1.8)
find_package(PCL 1.8 REQUIRED)
include_directories(include${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

find_package(Boost REQUIRED QUIET)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})


#set(OpenCV_DIR /usr/share/OpenCV)
#message(STATUS "version: ${OpenCV_VERSION}")
#find_package(OpenCV REQUIRED)
#include_directories(${OpenCV_INCLUDE_DIRS})

#add_executable(pcl_create src/pcl_create.cpp)
#target_link_libraries(pcl_create ${catkin_LIBRARIES} ${PCL_LIBRARIES})

#add
find_package(Qt5 REQUIRED Core Widgets Multimedia Charts)
set(QT_LIBRARIES Qt5::Widgets Qt5::Multimedia Qt5::Charts)
include_directories(
    ${catkin_INCLUDE_DIRS}
    /usr/include/QGRE
    /opt/ros/melodic/include
    )
link_directories(/home/mith/software/qt/5.14.2/gcc_64/lib)
#link_directories(/opt/ros/melodic/lib/python2.7/dist-packages lib)

#ros message
add_message_files(
  FILES
  Person.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything. 
catkin_package()

##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml
#delete
#rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/qt_rviz_demo/*.hpp *.h
    include/qt_rviz_demo/UIPromoteClass/*.h *.hpp)

#change
QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp
    src/UIPromoteClass/*.cpp)

file(GLOB_RECURSE PY_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS scripts/*.py)

# 安装 Python 脚本文件
catkin_install_python(PROGRAMS ${PY_SOURCES}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)

##############################################################################
# Binaries
##############################################################################
add_definitions(-DMY_SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(${TARGET_NAME} ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(${TARGET_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} Qt5::Charts curl)
install(TARGETS ${TARGET_NAME} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

