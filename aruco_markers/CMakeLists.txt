cmake_minimum_required(VERSION 3.8)
project(aruco_markers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc aruco)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(aruco_markers_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Create a library for the node implementation
add_library(aruco_markers_lib src/aruco_markers.cpp src/utils.cpp)
target_include_directories(aruco_markers_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(aruco_markers_lib PUBLIC
  ${OpenCV_LIBRARIES}
  ${aruco_markers_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  cv_bridge::cv_bridge
  image_transport::image_transport
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs_library
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::tf2_ros
)

# Create the executable for the node
add_executable(aruco_markers src/main.cpp)
target_include_directories(aruco_markers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(aruco_markers aruco_markers_lib ${OpenCV_LIBRARIES})

# Install targets
install(TARGETS aruco_markers_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS aruco_markers
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include/
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # Skip linters for now
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_aruco_markers test/test_aruco_markers.cpp)
  target_include_directories(test_aruco_markers PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
  )
  target_link_libraries(test_aruco_markers aruco_markers_lib ${OpenCV_LIBRARIES})
endif()

ament_package()