cmake_minimum_required(VERSION 3.8)
project(qt_mainwin_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Widgets Core Gui Charts)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/UI)

add_executable(qtControl_node src/main.cpp
                              src/main_window.cpp
                              include/qt_mainwin_control/main_window.hpp
)


target_include_directories(qtControl_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)

target_compile_features(qtControl_node PUBLIC c_std_99 cxx_std_20)


target_link_libraries(qtControl_node 
    Qt6::Widgets
    Qt6::Core
    Qt6::Gui
    Qt6::Charts
)

ament_target_dependencies(qtControl_node
  "rclcpp"
  "std_msgs"
)

install(TARGETS qtControl_node
  DESTINATION lib/${PROJECT_NAME}
)

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
