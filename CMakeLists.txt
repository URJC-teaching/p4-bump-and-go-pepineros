cmake_minimum_required(VERSION 3.8)
project(p4-bump-and-go-pepineros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kobuki_ros_interfaces REQUIRED)

set(dependencies
    rclcpp
    geometry_msgs
    kobuki_ros_interfaces
)

add_library(bumpgo_behavior SHARED
  src/bumpgo/bumpgo_behavior.cpp
)
ament_target_dependencies(bumpgo_behavior ${dependencies})
target_include_directories(bumpgo_behavior PUBLIC include)

add_executable(bumpgo_node src/bumpgo_main.cpp)
ament_target_dependencies(bumpgo_node ${dependencies})
target_link_libraries(bumpgo_node bumpgo_behavior)

install(TARGETS
  bumpgo_node
  bumpgo_behavior
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})
ament_package()
