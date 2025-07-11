在 ROS 2 中启动 `rqt_graph`（用于可视化节点和话题拓扑结构的工具）非常简单，以下是几种方法：

---

### **方法 1：直接运行 `rqt_graph`**
1. 打开一个终端，确保 ROS 2 环境已初始化（`source /opt/ros/<distro>/setup.bash`，如 `foxy`、`humble` 等）。
2. 直接运行以下命令：
   ```bash
   rqt_graph
   ```
   - 这会自动启动 `rqt_graph` 并显示当前运行的节点和话题连接。

---

### **方法 2：通过启动文件启动**
如果你想在 ROS 2 启动文件中集成 `rqt_graph`，可以在 `launch.py` 文件中添加 `Node` 配置：
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 启动 rqt_graph
        Node(
            package='rqt_graph',
            executable='rqt_graph',
            name='rqt_graph',
            output='screen'  # 可选：将日志输出到终端
        ),
        # 其他节点...
    ])
```
然后运行：
```bash
ros2 launch your_package launch_file.py
```

---

### **方法 3：动态启动**
如果 ROS 2 系统已经在运行，你可以直接启动 `rqt_graph` 查看实时拓扑：
1. 确保至少有一个 ROS 2 节点在运行（例如 `ros2 run demo_nodes_cpp talker`）。
2. 新开一个终端，运行：
   ```bash
   ros2 run rqt_graph rqt_graph
   ```
   或：
   ```bash
   rqt_graph
   ```

---

### **常见问题**
1. **`rqt_graph` 不显示节点/话题？**
   - 确保有节点正在运行（如 `ros2 run demo_nodes_cpp talker` 和 `ros2 run demo_nodes_cpp listener`）。
   - 检查节点是否正确发布/订阅话题（`ros2 topic list`）。

2. **`rqt_graph` 未安装？**
   - 安装 `rqt_graph`：
     ```bash
     sudo apt install ros-<distro>-rqt-graph
     ```
     例如（ROS 2 Humble）：
     ```bash
     sudo apt install ros-humble-rqt-graph
     ```

3. **界面卡顿或空白？**
   - 尝试刷新（点击左上角 **Refresh** 按钮）。
   - 确保 ROS 2 网络通信正常（如多机通信时配置 `ROS_DOMAIN_ID`）。

---

### **效果示例**
运行后，你会看到一个类似下图的交互式界面：
```
+-------------+       +-------------+
|   Node A    | ----> |   Node B    |
| (Publisher) |       |(Subscriber) |
+-------------+       +-------------+
```
你可以拖动节点、隐藏无关话题，或通过 **File > Save** 保存图像。

希望这能帮到你！如果有其他问题，欢迎继续提问。