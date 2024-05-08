cmake_minimum_required(VERSION 3.8)
project(final_group1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(rclpy REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(ariac_msgs REQUIRED)
# find_package(std_srvs REQUIRED)

set(ARIAC_INCLUDE_DEPENDS
  rclcpp 
  std_msgs
  ariac_msgs
  tf2_ros
  geometry_msgs
  tf2
  orocos_kdl
  tf2_ros
  tf2_geometry_msgs
  moveit_msgs
  geometric_shapes
  moveit_ros_planning_interface
  robot_commander_msgs
)

# find all dependencies
foreach(dependency IN ITEMS ${ARIAC_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()


install(DIRECTORY
  include
  launch
  config
  dataset
  meshes
  DESTINATION share/${PROJECT_NAME}
)
# # add include directories
include_directories(include)

# # set executables
add_executable(start_ariac_competition src/start_ariac_competition.cpp)
ament_target_dependencies(start_ariac_competition ${ARIAC_INCLUDE_DEPENDS}) 

add_executable(moveit_cpp_python src/ariac_cpp_node.cpp src/main_cpp_python.cpp)
ament_target_dependencies(moveit_cpp_python ${ARIAC_INCLUDE_DEPENDS})


# # install executables to lib/${PROJECT_NAME}
install(TARGETS
  start_ariac_competition
  moveit_cpp_python
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


# Python Executables

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  script/ariac_python_node.py
  script/ariac_bonus_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  launch/final_group1.launch.py
  launch/final_group1_bonus.launch.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
