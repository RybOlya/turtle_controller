cmake_minimum_required(VERSION 3.5)
project(turtle_controller)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add executable for the turtle controller node
add_executable(turtle_controller src/turtle_controller.cpp)
ament_target_dependencies(turtle_controller
  "geometry_msgs"
  "rclcpp"
)
# Link the turtle controller node against the required libraries
target_link_libraries(turtle_controller ${rclcpp_LIBRARIES} ${geometry_msgs_LIBRARIES})

# Install the turtle controller node
install(TARGETS turtle_controller
  DESTINATION lib/${PROJECT_NAME})

# Install the launch file
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# Install the params file
install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME})

# Install the package.xml file
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME})

# Add ament macros
ament_export_dependencies(rclcpp geometry_msgs)
ament_package()