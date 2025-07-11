---


---
## 更高级的launch手法
[Managing large projects — ROS 2 Documentation: Kilted documentation](https://docs.ros.org/en/kilted/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html)

1. 分而治之，分成几个小的launch文件，最后加一个总的launch文件，提高launch组织的复用性

### **3. 关键文件示例**
#### **(1) 模块化 Launch 文件 (`sensors.launch.py`)**
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    lidar_node = Node(
        package='lidar_driver',
        executable='lidar_node',
        parameters=[{'scan_topic': '/scan'}]
    )
    camera_node = Node(
        package='camera_driver',
        executable='camera_node',
        parameters=[{'frame_rate': 30}]
    )
    return LaunchDescription([lidar_node, camera_node])
```

#### **(2) 顶层聚合 Launch 文件 (`top_level.launch.py`)**
```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    sensors_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('my_robot_project'),
            'launch', 'common', 'sensors.launch.py'
        ])
    )
    robot_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('my_robot_project'),
            'launch', 'robots', 'robot_a.launch.py'
        ])
    )
    return LaunchDescription([sensors_launch, robot_launch])
```

#### **(3) 参数共享 (`config/params.yaml`)**
```yaml
# 共享参数示例
lidar:
  scan_topic: "/scan"
  range_max: 10.0

camera:
  frame_rate: 30
  resolution: "1080p"
```

---

### **4. 构建与运行**
#### **(1) 修改 `package.xml`**
确保添加 `launch` 和 `launch_ros` 依赖：
```xml
<exec_depend>launch</exec_depend>
<exec_depend>launch_ros</exec_depend>
```

#### **(2) 修改 `setup.py`**
安装 Launch 文件和配置：
```python
data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ('share/' + package_name + '/config', glob('config/*.yaml')),
]
```

#### **(3) 运行顶层 Launch 文件**
```bash
ros2 launch my_robot_project top_level.launch.py
```

---

### **5. 官方推荐实践**
1. **模块化设计**  
   - 将不同功能（如传感器、导航）拆分到独立 Launch 文件。
2. **参数复用**  
   - 通过 YAML 文件集中管理参数，避免硬编码。
3. **条件启动**  
   - 使用 `DeclareLaunchArgument` 和 `IfCondition` 实现动态配置。
4. **命名空间**  
   - 为多机器人场景添加命名空间：
     ```python
     Node(..., namespace='robot1')
     ```

---

### **6. 故障排查**
- **找不到 Launch 文件**：检查 `setup.py` 中的 `data_files` 配置。
- **参数未生效**：确认 YAML 文件路径在 Launch 文件中正确引用。
- **命名冲突**：使用 `remappings` 或命名空间解决 Topic 冲突。

如果需要更具体的示例（如多机器人协同），可以参考 [ROS 2 Launch 高级文档](https://docs.ros.org/en/kilted/How-To-Guides/Launch-files-migration-guide.html)。