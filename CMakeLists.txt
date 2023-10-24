cmake_minimum_required(VERSION 3.8)
project(beginner_tutorials)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

set(srv_files
    "srv/ChangeString.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
)

# Publisher and subscriber
add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker 
  rclcpp 
  std_msgs 
  example_interfaces 
  geometry_msgs 
  tf2 
  tf2_ros
)

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)

# target_link_libraries(mylib PUBLIC "${PROJECT_NAME}_msg_srv__rosidl_typesupport_cpp")
# rosidl_target_interfaces(talker ${PROJECT_NAME}_msg_srv "rosidl_typesupport_cpp")

rosidl_target_interfaces(talker
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# if(BUILD_TESTING)
#   find_package(ament_cmake_gtest REQUIRED)
#   ament_add_gtest(${PROJECT_NAME}_tutorial_test test/tutorial_test.cpp)
#   target_include_directories(${PROJECT_NAME}_tutorial_test PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>
#   )
#   ament_target_dependencies(${PROJECT_NAME}_tutorial_test
#     std_msgs
#     rclcpp 
#     geometry_msgs
#     rclcpp
#     tf2_ros
#     tf2
#   )
#   target_link_libraries(${PROJECT_NAME}_tutorial_test beginner_tutorials)
# endif()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
