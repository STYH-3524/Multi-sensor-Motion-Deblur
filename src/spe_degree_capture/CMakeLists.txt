cmake_minimum_required(VERSION 3.8)
project(spe_degree_capture)

# 1. 寻找依赖（新增了 ament_cmake）
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# 2. 海康 MVS SDK 配置
set(MVS_SDK_PATH "/opt/MVS")
include_directories(${MVS_SDK_PATH}/include)
link_directories(${MVS_SDK_PATH}/lib/64)

# 3. 生成可执行文件
# 注意：确认你的 capture.cpp 是在根目录还是 src 目录下
add_executable(capture_node src/capture.cpp) 

# 4. 包含头文件
target_include_directories(capture_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  ${OpenCV_INCLUDE_DIRS}
)

# 5. 处理依赖 (用这行替代冗长的手动链接)
# 它会自动处理 rclcpp 和 std_msgs 的头文件和库
ament_target_dependencies(capture_node rclcpp std_msgs)

# 6. 链接非 ROS 库（OpenCV 和 海康 SDK）
target_link_libraries(capture_node
  ${OpenCV_LIBRARIES}
  MvCameraControl 
)

# 7. 安装规则
install(TARGETS capture_node
  DESTINATION lib/${PROJECT_NAME})

# 8. 【最关键的一行】告诉 ROS 2 这是一个功能包
ament_package()
