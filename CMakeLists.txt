cmake_minimum_required(VERSION 3.8)
project(p4-bump-and-go-pepineros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


set(dependencies
    rclcpp
    geometry_msgs
    tf2_geometry_msgs
    tf2
    tf2_ros
)

include_directories(include)
add_library(${PROJECT_NAME} SHARED
  src/go_fwd_and_turn/Controller.cpp
  src/go_fwd_and_turn/Controller2.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(bumpgo_node src/bumpgo_node.cpp)
ament_target_dependencies(bumpgo_node ${dependencies})
target_link_libraries(bumpgo_node ${PROJECT_NAME})



install(TARGETS
  bumpgo_node
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
