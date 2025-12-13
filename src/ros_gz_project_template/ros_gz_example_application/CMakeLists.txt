cmake_minimum_required(VERSION 3.8)
project(ros_gz_example_application)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Trouver les dépendances
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


# 2. Créer l'exécutable
add_executable(robot_mover src/robot_mover.cpp) # exécutable déplacement robot avant
add_executable(lidar_reader src/lidar_reader.cpp) # exécutable lidar
add_executable(autonomous_robot src/autonomous_robot.cpp) # --- ROBOT AUTONOME ---
add_executable(autonomous_robot_artur src/autonomous_robot_artur.cpp)
add_executable(autonomous_robot_greg src/autonomous_robot_greg.cpp)

# 3. Lier les dépendances à l'exécutable
ament_target_dependencies(robot_mover rclcpp geometry_msgs) 
ament_target_dependencies(lidar_reader rclcpp sensor_msgs)
ament_target_dependencies(autonomous_robot rclcpp geometry_msgs sensor_msgs)
ament_target_dependencies(autonomous_robot_artur rclcpp geometry_msgs sensor_msgs)
ament_target_dependencies(autonomous_robot_greg rclcpp geometry_msgs sensor_msgs)

# 4. Installer l'exécutable pour qu'il soit accessible via 'ros2 run'
install(TARGETS
  robot_mover
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  lidar_reader
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  autonomous_robot
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  autonomous_robot_artur
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  autonomous_robot_greg
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()