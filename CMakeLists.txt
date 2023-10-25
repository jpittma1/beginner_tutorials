cmake_minimum_required(VERSION 3.8)
project(beginner_tutorials)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
    rclcpp
    rclcpp_action
    std_msgs
)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(turtlesim REQUIRED)
find_package(rosidl_default_generators REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# include_directories(include)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ChangeString.srv"
)

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs tf2 tf2_ros)

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

# add_executable(client src/change_string_client.cpp)
# ament_target_dependencies(client rclcpp std_msgs)

install(TARGETS
  talker
  listener
  # client
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}
)

rosidl_target_interfaces(talker
${PROJECT_NAME} "rosidl_typesupport_cpp")

# rosidl_target_interfaces(client
# ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(listener
${PROJECT_NAME} "rosidl_typesupport_cpp")

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  # the following lines skip linters
  set(ament_cmake_cppcheck_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Run all lint tests in package.xml except those listed above
  ament_lint_auto_find_test_dependencies()

  # Add a gtest executable
  set(TESTFILES
    test/integration_test.cpp)
  ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES})
  ament_target_dependencies(${PROJECT_NAME}_test ${THIS_PACKAGE_INCLUDE_DEPENDS})
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})


ament_package()
