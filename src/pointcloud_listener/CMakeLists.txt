cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_listener)

# 使用C++14标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找catkin包和其他依赖项
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  pcl_ros
  pcl_conversions
)

# 查找PCL库
find_package(PCL REQUIRED)

catkin_package(

  # 这里可以声明包的包含目录、库、依赖等
  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs pcl_ros pcl_conversions
)
include_directories(include)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS} # 包含PCL的头文件目录
)

# 添加PCL定义
add_definitions(${PCL_DEFINITIONS})

# 构建pointcloud_converter节点
add_executable(pointcloud_converter src/pointcloud_converter.cpp)
target_link_libraries(pointcloud_converter ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(pointcloud_converter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# # 构建pointcloud_converter2节点
# add_executable(pointcloud_converter2 src/pointcloud_converter2.cpp)
# target_link_libraries(pointcloud_converter2 ${catkin_LIBRARIES} ${PCL_LIBRARIES})
# add_dependencies(pointcloud_converter2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 构建pointcloud_processor节点
add_executable(pointcloud_processor src/pointcloud_processor.cpp)
target_link_libraries(pointcloud_processor ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(pointcloud_processor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 构建pointcloud_processor节点
add_executable(pointcloud_processor2 src/pointcloud_processor.cpp)
target_link_libraries(pointcloud_processor2 ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(pointcloud_processor2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 安装Python脚本
catkin_install_python(PROGRAMS scripts/pointcloud_listener.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
