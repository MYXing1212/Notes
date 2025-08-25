ament_target_dependencies( fixed_frame_tf2_broadcaster geometry_msgs rclcpp tf2_ros )

1. `ament_target_dependencies(...)`  
    • 这是 `ament_cmake` 提供的宏，等价于帮你调用 `find_package()` 和 `target_link_libraries()`，省得自己写一串 include 目录和库文件。  
    • 宏里列出的 `geometry_msgs`、`rclcpp`、`tf2_ros` 都是 ROS 2 的包名。  
    • 作用：  
    – 自动把这些包的 include 目录加到编译 flags；  
    – 自动把它们的库文件链接到 `fixed_frame_tf2_broadcaster` 这个目标上；  
    – 如果以后这些包的依赖变了，这里不需要改。