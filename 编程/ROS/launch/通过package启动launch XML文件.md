在 **ROS 2 Jazzy** 中使用 `.xml` 格式的 Launch 文件需要遵循特定的语法规则，因为 ROS 2 主要支持 **Python (.py)** 和 **XML (.xml)** 两种格式的 Launch 文件，但默认推荐使用 Python。以下是详细的使用方法：

---

## **1. 创建 `.xml` Launch 文件**
在 ROS 2 包的 `launch` 目录下创建一个 `.xml` 文件（例如 `my_launch.xml`）：
```bash
mkdir -p ~/ros2_ws/src/my_package/launch
touch ~/ros2_ws/src/my_package/launch/my_launch.xml
```

### **示例：`my_launch.xml`**
```xml
<launch>
  <!-- 启动一个节点 -->
  <node pkg="my_package" exec="my_node" name="my_node" output="screen" />

  <!-- 设置参数 -->
  <param name="my_param" value="42" />

  <!-- 启动另一个节点 -->
  <node pkg="rviz2" exec="rviz2" name="rviz2" args="-d $(find-pkg-share my_package)/config/rviz_config.rviz" />
</launch>
```

---

## **2. 修改 `package.xml`**
确保 `package.xml` 包含 `launch` 文件的依赖：
```xml
<exec_depend>ament_cmake</exec_depend>  <!-- 如果是 C++ 包 -->
<exec_depend>ament_python</exec_depend>  <!-- 如果是 Python 包 -->
<exec_depend>launch</exec_depend>      <!-- 必须添加 -->
```

---

## **3. 修改 `CMakeLists.txt`（仅 C++ 包）**
如果包是 C++ 编写的，需要在 `CMakeLists.txt` 中安装 Launch 文件：
```cmake
# 安装 launch 文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
```

---

## **4. 编译并运行**
```bash
cd ~/ros2_ws
colcon build --packages-select my_package
source install/setup.bash
```

### **运行 Launch 文件**
```bash
ros2 launch my_package my_launch.xml
```

---

## **5. 关键语法说明**
| 语法 | 说明 |
|------|------|
| `<launch>` | Launch 文件的根标签 |
| `<node>` | 启动节点，`pkg` 是包名，`exec` 是可执行文件名 |
| `<param>` | 设置参数 |
| `$(find-pkg-share pkg_name)` | 获取包的共享目录路径（替代 ROS 1 的 `$(find pkg)`） |
| `<arg>` | 定义参数（需在命令行或 `<include>` 中传递） |

---

## **6. 进阶用法**
### **(1) 传递参数**
```xml
<launch>
  <arg name="use_sim" default="false" />
  <node pkg="my_package" exec="my_node" name="my_node">
    <param name="use_sim" value="$(var use_sim)" />
  </node>
</launch>
```
运行时指定参数：
```bash
ros2 launch my_package my_launch.xml use_sim:=true
```

### **(2) 包含其他 Launch 文件**
```xml
<launch>
  <include file="$(find-pkg-share another_package)/launch/another_launch.xml" />
</launch>
```

### **(3) 条件启动**
```xml
<launch>
  <if value="$(var use_sim)">
    <node pkg="gazebo" exec="gazebo" name="gazebo" />
  </if>
</launch>
```

---

## **7. 常见问题**
### **Q1: `ros2 launch` 找不到 `.xml` 文件**
- 确保 `launch` 目录正确安装到 `share/${PROJECT_NAME}/`。
- 检查 `package.xml` 和 `CMakeLists.txt` 配置。

### **Q2: 参数传递失败**
- 使用 `$(var arg_name)` 语法，而不是 ROS 1 的 `$(arg arg_name)`。

### **Q3: `find-pkg-share` 报错**
- 确保包已正确编译并通过 `colcon build`。

---

## **8. 对比 ROS 1 和 ROS 2 的 Launch 文件**
| 功能 | ROS 1 (`*.launch`) | ROS 2 (`*.xml`) |
|------|--------------------|-----------------|
| 启动节点 | `<node pkg="pkg" type="exec" />` | `<node pkg="pkg" exec="exec" />` |
| 参数替换 | `$(arg var)` | `$(var var)` |
| 包路径 | `$(find pkg)` | `$(find-pkg-share pkg)` |
| 条件语句 | `<if>` / `<unless>` | `<if>` / `<unless>` |

---

## **总结**
- **ROS 2 Jazzy 支持 `.xml` Launch 文件**，但语法与 ROS 1 不同。
- **关键改动**：`exec` 替代 `type`，`$(var)` 替代 `$(arg)`，`find-pkg-share` 替代 `find`。
- **推荐**：对于复杂逻辑，优先使用 **Python Launch 文件**（`.launch.py`），灵活性更高。

如果需要更复杂的逻辑（如循环、动态参数），建议参考 [ROS 2 Launch 官方文档](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Launch-Main.html)。