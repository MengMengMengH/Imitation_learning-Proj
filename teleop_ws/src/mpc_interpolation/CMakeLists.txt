cmake_minimum_required(VERSION 3.8)
project(mpc_interpolation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(drake REQUIRED)
find_package(qpOASES REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Widgets Charts)



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

add_executable(mpc_interpolation_node src/interpolation.cpp
                                      src/mpc_spline.cpp
)
add_executable(cubic_spline_node src/cubic_spline.cpp
                                 src/mpc_spline.cpp
)

target_link_directories(mpc_interpolation_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_directories(cubic_spline_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(mpc_interpolation_node PUBLIC c_std_99 cxx_std_20)
target_compile_features(cubic_spline_node PUBLIC c_std_99 cxx_std_20)

target_link_libraries(mpc_interpolation_node 
    qpOASES
    Eigen3::Eigen
    Qt6::Widgets
    Qt6::Charts
)

target_link_libraries(cubic_spline_node
    qpOASES
    Eigen3::Eigen
    Qt6::Widgets
    Qt6::Charts
)

ament_target_dependencies(mpc_interpolation_node
  "rclcpp"
  "Eigen3"
  "qpOASES"
  "std_msgs"
)
ament_target_dependencies(cubic_spline_node
  "Eigen3"
  "qpOASES"
  "std_msgs"
)

install(TARGETS mpc_interpolation_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS cubic_spline_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()