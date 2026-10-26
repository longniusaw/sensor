cmake_minimum_required(VERSION 3.8)
project(lidarsensor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_program(CCACHE_FOUND ccache)
find_package(tf2_ros REQUIRED)
if(CCACHE_FOUND)
    set(CMAKE_C_COMPILER_LAUNCHER ccache)
    set(CMAKE_CXX_COMPILER_LAUNCHER ccache)
endif()
####################################################
# 编译雷达过滤节点
add_executable(scan_filter_node
  src/scan_filter_node.cpp
)

# 为节点配置 ROS 2 依赖
ament_target_dependencies(scan_filter_node
  rclcpp
  sensor_msgs
)

# 安装可执行文件，让 ros2 run 能够找到它
install(TARGETS
  scan_filter_node
  DESTINATION lib/${PROJECT_NAME}
)
#####################################################
# 编译障碍物检测节点
add_executable(lidar_obstacle_detector
  src/lidar_obstacle_detector.cpp
)


# 为障碍物检测节点配置 ROS 2 依赖
ament_target_dependencies(lidar_obstacle_detector
  rclcpp
  sensor_msgs
)


# 安装障碍物检测节点
install(TARGETS
  lidar_obstacle_detector
  DESTINATION lib/${PROJECT_NAME}
)
######################################################
# 编译障碍物检测节点2
add_executable(lidar_obstacledetect_node
  src/lidar_obstacledetect_node.cpp
)


# 为障碍物检测节点配置 ROS 2 依赖
ament_target_dependencies(lidar_obstacledetect_node
  rclcpp
  sensor_msgs
  visualization_msgs
)


# 安装障碍物检测节点2
install(TARGETS
lidar_obstacledetect_node
  DESTINATION lib/${PROJECT_NAME}
)
#####################################################
# 编译bcr_bot数据接收节点
add_executable(lidar_data_receive
  github_bcr_bot/lidar/lidarinput.cpp
)

# 为该数据接收节点配置 ROS 2 依赖
ament_target_dependencies(lidar_data_receive
  rclcpp
  sensor_msgs
  visualization_msgs
  geometry_msgs  
)

# 安装数据接收节点
install(TARGETS
lidar_data_receive
  DESTINATION lib/${PROJECT_NAME}
)

#####################################################
#编译激光雷达ROR（半径离群点去除算法）节点
add_executable(lidarROR
  github_bcr_bot/lidar/lidarROR.cpp
)
#为该节点配置ROS2依赖
ament_target_dependencies(
  lidarROR
  rclcpp
  sensor_msgs
  visualization_msgs
  geometry_msgs  
)
#安装ROR节点
install(TARGETS
  lidarROR
  DESTINATION lib/${PROJECT_NAME}

)
#####################################################
#编译坐标系初步转换节点
add_executable(map_to_laser
  github_bcr_bot/TF2/receivemap.cpp
)

#为该节点配置ROS2依赖
ament_target_dependencies(
  map_to_laser
  rclcpp
  sensor_msgs
  visualization_msgs
  tf2_ros  
)

#安装坐标系转换节点
install(TARGETS
  map_to_laser
  DESTINATION lib/${PROJECT_NAME}

)

#####################################################
#编译imu数据接收节点
add_executable(imu_receiver_node
  github_GazeboCookBook/imu/imu_receiver_node.cpp
)
#配置依赖
ament_target_dependencies(imu_receiver_node
  rclcpp
  sensor_msgs
)


#安装imu数据接收节点
install(TARGETS
  imu_receiver_node
  DESTINATION lib/${PROJECT_NAME}

)

#####################################################





if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
