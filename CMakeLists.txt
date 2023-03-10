cmake_minimum_required(VERSION 3.8)
project(xrm_zmp_interface)

find_package(ament_cmake_auto REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

ament_auto_add_executable(xrm_zmp_interface
  src/xrm_zmp_interface.cpp
  src/xrm_zmp_interface_node.cpp
  src/zmp/HevCnt.cpp
  src/zmp/vehicle_util.cpp
)

target_link_libraries(
  xrm_zmp_interface
  ${PROJECT_SOURCE_DIR}/lib/libCANUSB.so
  ${PROJECT_SOURCE_DIR}/lib/libhevcontrol.so
)

include_directories(
  include
  ${PROJECT_SOURCE_DIR}/include
)

target_include_directories(xrm_zmp_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(
  DIRECTORY lib/
  DESTINATION lib/${PROJECT_NAME}
)

install (TARGETS
  xrm_zmp_interface
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
