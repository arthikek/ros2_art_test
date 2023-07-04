cmake_minimum_required(VERSION 3.8)
project(ros2_art_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
"srv/OddEvenCheck.srv"
"srv/CornerCamera.srv"
"action/Navigate.action"
"action/Function.action"
ADD_LINTER_TESTS
DEPENDENCIES std_msgs sensor_msgs geometry_msgs action_msgs
sensor_msgs


)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(rpm src/rpm.cpp)
ament_target_dependencies(rpm rclcpp std_msgs)

add_executable(publisher src/publisher.cpp)
ament_target_dependencies(publisher rclcpp std_msgs)

add_executable(subscriber src/subscriber.cpp)
ament_target_dependencies(subscriber rclcpp std_msgs)

add_executable(speed src/speed.cpp)
ament_target_dependencies(speed rclcpp std_msgs)

add_executable(service_server src/service_server.cpp)
ament_target_dependencies(service_server rclcpp std_msgs)
target_link_libraries(service_server ${cpp_typesupport_target})

add_executable(service_client src/service_client.cpp)
ament_target_dependencies(service_client rclcpp std_msgs)
target_link_libraries(service_client ${cpp_typesupport_target})

add_executable(image_service_client src/image_service_client.cpp)
ament_target_dependencies(image_service_client rclcpp std_msgs cv_bridge sensor_msgs OpenCV )
target_link_libraries(image_service_client ${cpp_typesupport_target})

add_executable(image_service_server src/image_service_server.cpp)
ament_target_dependencies(image_service_server rclcpp std_msgs cv_bridge sensor_msgs OpenCV)
target_link_libraries(image_service_server ${cpp_typesupport_target})

add_executable(action_server src/action_server.cpp)
ament_target_dependencies(action_server rclcpp  rclcpp_action  std_msgs cv_bridge sensor_msgs OpenCV)
target_link_libraries(action_server ${cpp_typesupport_target})

add_executable(action_client src/action_client.cpp)
ament_target_dependencies(action_client rclcpp geometry_msgs action_msgs rclcpp_action) 
target_link_libraries(action_client ${cpp_typesupport_target})

add_executable(action_server_robot src/action_server_robot.cpp)
ament_target_dependencies(action_server_robot rclcpp geometry_msgs action_msgs rclcpp_action  ) 
target_link_libraries(action_server_robot ${cpp_typesupport_target})

add_executable(action_counter_client src/action_counter_client.cpp)
ament_target_dependencies(action_counter_client rclcpp geometry_msgs action_msgs rclcpp_action  ) 
target_link_libraries(action_counter_client ${cpp_typesupport_target})

 install(TARGETS
  action_counter_client
  action_server_robot
  action_client
  action_server
  image_service_client
  image_service_server
  service_client
  service_server
  rpm
  publisher
  subscriber
  speed
  DESTINATION lib/${PROJECT_NAME}
 
 )


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
