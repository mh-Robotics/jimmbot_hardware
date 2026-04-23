cmake_minimum_required(VERSION 3.8)
project(jimmbot_bringup)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY install launch udev
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
