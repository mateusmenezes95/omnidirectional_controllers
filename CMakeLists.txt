cmake_minimum_required(VERSION 3.5)
project(omnidirectional_controllers)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp_lifecycle
  rclcpp
  tf2_msgs
  tf2_ros
  tf2
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(odometry SHARED)

target_sources(odometry 
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src/odometry.cpp
)

target_include_directories(odometry
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(kinematics SHARED)

target_sources(kinematics 
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src/kinematics.cpp
)

target_include_directories(kinematics
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(${PROJECT_NAME} SHARED
  src/omnidirectional_controller.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(${PROJECT_NAME}
  odometry
  kinematics
)

target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(controller_interface omnidirectional_plugin.xml)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)

  ament_lint_auto_find_test_dependencies()

  ament_add_gmock(
    test_odometry
    test/test_odometry.cpp
    src/odometry.cpp
  )

  target_link_libraries(test_odometry
    odometry
    kinematics
  )

  ament_add_gmock(
    test_kinematics
    test/test_kinematics.cpp
    src/kinematics.cpp
  )

  target_link_libraries(test_kinematics
    odometry
    kinematics
  )

  target_include_directories(test_odometry PRIVATE include)
  ament_target_dependencies(test_odometry)

  target_include_directories(test_kinematics PRIVATE include)
  ament_target_dependencies(test_kinematics)
endif()

ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_package()
