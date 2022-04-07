cmake_minimum_required(VERSION 3.5)
project(gazebo_tutorials)


# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wno-inconsistent-missing-override")
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.


# Depend on system install of Gazebo
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)

#warning: variable CMAKE_CXX_FLAGS is modified
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

#warning: use of link_directories() is strongly discouraged
MESSAGE(STATUS "GAZEBO_INSTALL_LIB_DIR: ${GAZEBO_INSTALL_LIB_DIR}")
link_directories(${gazebo_dev_LIBRARY_DIRS})
link_directories(${GAZEBO_INSTALL_LIB_DIR})
include_directories(${Boost_INCLUDE_DIR} ${GAZEBO_INCLUDE_DIRS})


add_library(${PROJECT_NAME} SHARED src/simple_world_plugin.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  
ament_target_dependencies(
  ${PROJECT_NAME}
  "rclcpp"
  "gazebo_ros"
)
ament_export_libraries(${PROJECT_NAME})

link_directories(${GAZEBO_INSTALL_LIB_DIR})

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME})
  
ament_package()
