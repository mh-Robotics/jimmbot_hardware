cmake_minimum_required(VERSION 3.8)
project(jimmbot_hardware)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(jimmbot_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)

find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
  pkg_check_modules(LIBUSB libusb-1.0)
endif()

# ── can_msg_wrapper library ──────────────────────────────────────────────────
add_library(can_msg_wrapper src/can_msg_wrapper.cpp)
target_include_directories(can_msg_wrapper PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(can_msg_wrapper PUBLIC
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${jimmbot_msgs_TARGETS}
)

# ── hardware_interface node ──────────────────────────────────────────────────
add_executable(hardware_interface src/hardware_interface.cpp)
target_include_directories(hardware_interface PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(hardware_interface PUBLIC
  can_msg_wrapper
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${jimmbot_msgs_TARGETS}
)

# ── extended_joy node ────────────────────────────────────────────────────────
add_executable(extended_joy src/extended_joy.cpp)
target_include_directories(extended_joy PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(extended_joy PUBLIC
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  sensor_msgs::sensor_msgs_library
  ${jimmbot_msgs_TARGETS}
)

# ── kinect_aux node ──────────────────────────────────────────────────────────
add_executable(kinect_aux src/kinect_aux.cpp)
target_include_directories(kinect_aux PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${LIBUSB_INCLUDE_DIRS}
)
target_link_libraries(kinect_aux PUBLIC
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  sensor_msgs::sensor_msgs_library
  ${LIBUSB_LIBRARIES}
)

# ── ydlidar_g4_node ──────────────────────────────────────────────────────────
find_package(ydlidar_sdk QUIET)
if(ydlidar_sdk_FOUND)
  add_executable(ydlidar_g4_node src/ydlidar_g4_node.cpp)
  target_include_directories(ydlidar_g4_node PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(ydlidar_g4_node PUBLIC
    rclcpp::rclcpp
    ${sensor_msgs_TARGETS}
    sensor_msgs::sensor_msgs_library
    ydlidar_sdk::ydlidar_sdk
  )
else()
  message(STATUS "ydlidar_sdk not found — skipping ydlidar_g4_node")
endif()

# ── install targets ──────────────────────────────────────────────────────────
install(
  TARGETS
    can_msg_wrapper
    hardware_interface
    extended_joy
    kinect_aux
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(ydlidar_sdk_FOUND)
  install(TARGETS ydlidar_g4_node
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config install launch udev
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
