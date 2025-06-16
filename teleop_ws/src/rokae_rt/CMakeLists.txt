cmake_minimum_required(VERSION 3.20)
project(rokae_rt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Using static/shared libraries option
option(XCORE_LINK_SHARED_LIBS "Example executables link shared library" OFF)
# Compile option: whether using xMateModel library, supports Linux x86_64 and Windows 64bit
option(XCORE_USE_XMATE_MODEL "The library of kinematics and dynamics calculation for cobot" OFF)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)

add_subdirectory(lib)

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

add_executable(rokae_rt_node
  src/main.cpp
)

target_link_libraries(rokae_rt_node
  Rokae::Rokae
  Eigen3::Eigen
)


target_include_directories(rokae_rt_node PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(rokae_rt_node
  rclcpp
)

install(TARGETS
  rokae_rt_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
