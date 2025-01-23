cmake_minimum_required(VERSION 3.8)
project(my_robot_interfaces)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# Generate interfaces (messages, services, and actions)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PS4Service.srv"        # Path to the .srv file
  "action/MoveCircle.action" # Path to the .action file
  "srv/GetCoordinates.srv"
  
  DEPENDENCIES std_msgs        # Any dependencies for message types used
)

# Export dependencies for runtime
ament_export_dependencies(rosidl_default_runtime)

# Install package
ament_package()
