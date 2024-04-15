cmake_minimum_required(VERSION 3.8)
project(rwa4_group1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ariac_msgs REQUIRED)
find_package(std_srvs REQUIRED)


install(DIRECTORY
  include
  launch
  config
  dataset
  DESTINATION share/${PROJECT_NAME}
)
# # add include directories
include_directories(include)

# # set executables
add_executable(start_ariac_competition src/start_ariac_competition.cpp)


ament_target_dependencies(start_ariac_competition rclcpp std_msgs ariac_msgs std_srvs) 

# # install executables to lib/${PROJECT_NAME}
install(TARGETS
  start_ariac_competition
  DESTINATION lib/${PROJECT_NAME}
)


# Python Executables

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  script/order_management.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  launch/rwa4_group1.launch.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
