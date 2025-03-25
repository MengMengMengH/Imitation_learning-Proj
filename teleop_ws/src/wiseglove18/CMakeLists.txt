cmake_minimum_required(VERSION 3.8)
project(wiseglove18)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)


find_library(WISEGLOVE_LIB wiseglove)
if (NOT WISEGLOVE_LIB)
  message(FATAL_ERROR "wiseglove library not found")
endif()


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

add_executable(glove_data_pub src/glove_data_pub.cpp)
ament_target_dependencies(glove_data_pub rclcpp)
ament_target_dependencies(glove_data_pub std_msgs)
target_link_libraries(glove_data_pub ${WISEGLOVE_LIB})

install(TARGETS
  glove_data_pub
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
