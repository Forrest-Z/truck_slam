cmake_minimum_required(VERSION 3.5)
project(localization_mapping)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

set(PROJECT_CMAKE_DIR ${PROJECT_SOURCE_DIR}/cmake)
set(PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
set(PROJECT_NODE_DIR ${PROJECT_SOURCE_DIR}/src/node)
set(PROJECT_UTILS_DIR ${PROJECT_SOURCE_DIR}/src/utils)

set(WORK_SPACE_PATH ${PROJECT_SOURCE_DIR})
configure_file(
  ${PROJECT_SOURCE_DIR}/include/localization_mapping/global_defination/global_defination.h.in
  ${PROJECT_SOURCE_DIR}/include/localization_mapping/global_defination/global_defination.h
)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL 1.10 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(glog REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GTSAM REQUIRED)
find_package(msg_interface REQUIRED)
find_package(OpenMP REQUIRED)
# find_package(robot_localization REQUIRED)

include_directories(
  ${PROJECT_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIRS}
)

set(3rdParty_libs 
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  gtsam
  glog
)


## utils lib
add_library(utils_lib
  ${PROJECT_UTILS_DIR}/tools/tools.cpp
  ${PROJECT_UTILS_DIR}/model/registration/icp_registration.cpp
  ${PROJECT_UTILS_DIR}/model/registration/ndt_registration.cpp
  ${PROJECT_UTILS_DIR}/model/scan_context/scan_context_manager.cpp
)
ament_target_dependencies(utils_lib rclcpp sensor_msgs nav_msgs msg_interface)
target_compile_options(utils_lib PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(utils_lib ${3rdParty_libs} ${OpenMP_CXX_FLAGS})


## node executable
add_executable(data_pretreat_node 
  ${PROJECT_SOURCE_DIR}/src/data_pretreat/data_pretreat.cpp  
  ${PROJECT_SOURCE_DIR}/src/data_pretreat/data_pretreat_flow.cpp
  ${PROJECT_NODE_DIR}/data_pretreat_node.cpp
  )
ament_target_dependencies(data_pretreat_node rclcpp std_msgs sensor_msgs)
target_link_libraries(data_pretreat_node utils_lib)

add_executable(front_end_node 
  ${PROJECT_SOURCE_DIR}/src/front_end/front_end.cpp  
  ${PROJECT_SOURCE_DIR}/src/front_end/front_end_flow.cpp
  ${PROJECT_NODE_DIR}/front_end_node.cpp
  )
ament_target_dependencies(front_end_node rclcpp std_msgs sensor_msgs)
target_compile_options(front_end_node PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(front_end_node utils_lib ${OpenMP_CXX_FLAGS})

add_executable(back_end_node 
  ${PROJECT_SOURCE_DIR}/src/back_end/back_end.cpp  
  ${PROJECT_SOURCE_DIR}/src/back_end/back_end_flow.cpp
  ${PROJECT_NODE_DIR}/back_end_node.cpp
  )
ament_target_dependencies(back_end_node rclcpp std_msgs sensor_msgs tf2_ros msg_interface)
target_link_libraries(back_end_node utils_lib)

add_executable(loop_closure_node 
  ${PROJECT_SOURCE_DIR}/src/loop_closure/loop_closure.cpp  
  ${PROJECT_SOURCE_DIR}/src/loop_closure/loop_closure_flow.cpp
  ${PROJECT_NODE_DIR}/loop_closure_node.cpp
  )
ament_target_dependencies(loop_closure_node rclcpp std_msgs sensor_msgs msg_interface)
target_link_libraries(loop_closure_node utils_lib)

add_executable(imu_preintegration_node 
  ${PROJECT_SOURCE_DIR}/src/imu_preintegration/imu_preintegration.cpp
  ${PROJECT_NODE_DIR}/imu_preintegration_node.cpp
  )
ament_target_dependencies(imu_preintegration_node rclcpp std_msgs tf2 tf2_ros sensor_msgs nav_msgs geometry_msgs msg_interface)
target_link_libraries(imu_preintegration_node utils_lib)

add_executable(map_matching_node 
  ${PROJECT_SOURCE_DIR}/src/map_matching/map_matching.cpp  
  ${PROJECT_SOURCE_DIR}/src/map_matching/map_matching_flow.cpp
  ${PROJECT_NODE_DIR}/map_matching_node.cpp
  )
ament_target_dependencies(map_matching_node rclcpp std_msgs sensor_msgs  geometry_msgs tf2_ros msg_interface)
target_link_libraries(map_matching_node utils_lib)

install(TARGETS data_pretreat_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS front_end_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS back_end_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS loop_closure_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS imu_preintegration_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS map_matching_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
