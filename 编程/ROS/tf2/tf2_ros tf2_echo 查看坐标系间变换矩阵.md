这条命令

```
ros2 run tf2_ros tf2_echo world turtle1
```

的作用是：

> **让 tf2 工具把当前系统中从坐标系 `world` 到坐标系 `turtle1` 的实时变换（平移 + 旋转）持续打印到终端。**

---

### 详细说明

- **`ros2 run tf2_ros tf2_echo`**：  
  启动 `tf2_echo` 节点，它是 ROS 2 自带的 **TF2 调试工具**，用来查看任意两个坐标系之间的变换关系。

- **参数顺序**：  
  `tf2_echo <source_frame> <target_frame>`  
  注意顺序是 **“从哪个 frame，到哪个 frame”**。  
  因此 `world turtle1` 表示 **“以 world 为参考，turtle1 的位置与姿态”**。

- **输出内容**：  
  终端会每秒刷新一次，显示形如

  ```
  At time 1688xxxx.xxx
  - Translation: [x, y, z]
  - Rotation: in Quaternion [qx, qy, qz, qw]
  ```

  随着 turtle1 移动，这些数值会实时变化 。

---

### 常见用途

- 验证 **turtle1 的 TF 广播器**是否正常工作。  
- 在 **rviz2** 或其他节点需要 turtle1 位姿前，先人工确认数据。  
- 调试坐标系名称拼写、时间同步等问题。