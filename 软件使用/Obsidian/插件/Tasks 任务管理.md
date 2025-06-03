以下是关于 **Obsidian Tasks 插件** 的详细使用指南，帮助您高效管理任务：

---

### **1. 安装与启用**
- **安装**：
  1. 打开 Obsidian → 设置 → 社区插件 → 浏览 → 搜索 "Tasks" → 安装。
  2. 启用插件后，建议重启 Obsidian。

Ctrl + P之后，输入task即可找到Create or edit task任务的功能
  ![[Pasted image 20250603095803.png]]


---

### **2. 基础语法**
在 Markdown 文件中用以下格式创建任务：
````markdown
```tasks
# 查询条件（可选）
```
````
或直接创建任务项：
```markdown
- [ ] 这是一个待办任务 📅 2023-10-10 ⏳ 2023-10-08
```

---

### **3. 核心功能详解**
#### **(1) 创建任务**
- **基础任务**：
  ```markdown
  - [ ] 写周报
  ```
- **带日期**：
  ```markdown
  - [ ] 开会 📅 2023-10-15
  ```
- **循环任务**：
  ```markdown
  - [ ] 每周复盘 🔁 every week
  ```

#### **(2) 任务属性**
| 属性       | 语法示例            | 说明                     |
|------------|---------------------|--------------------------|
| 截止日期   | `📅 YYYY-MM-DD`     | 使用 `📅` 或 `due:`       |
| 开始日期   | `⏳ YYYY-MM-DD`     | 到期前不显示             |
| 优先级     | `🔺high`/`🔻low`   | 高/中/低优先级           |
| 标签       | `#会议`             | 用 `#` 分类              |
| 循环       | `🔁 every weekday`  | 支持 daily/weekly/monthly|

#### **(3) 任务查询**
在代码块中筛选任务：
````markdown
```tasks
# 查询今日到期任务
due today

# 查询未完成的高优先级任务
not done 
priority is high

# 查询特定标签任务
tags include #项目A
```
````

---

### **4. 高级技巧**
- **看板视图**：  
  通过 `group by` 创建看板：
  ````markdown
  ```tasks
  group by priority
  group by due
  ```
  ````
- **任务计数**：  
  在文档中显示未完成任务数：  
  ````markdown
  ```tasks
  not done
  count
  ```
  ````
- **自定义样式**：  
  在 CSS 片段中修改任务显示样式：
  ```css
  .task-list-item {
    color: red;
  }
  ```

---

### **5. 同步与备份**
- **同步**：任务数据存储在 Markdown 文件中，与 Obsidian 同步机制（如 Git/Syncthing）兼容。
- **导出**：用 `Dataview` 插件导出为表格：
  ````markdown
  ```dataview
  TASK FROM "path/to/folder"
  ```
  ````

---

### **6. 常见问题**
- **日期不识别**：确保日期格式为 `YYYY-MM-DD`。
- **查询无效**：检查是否在代码块中使用 ````tasks````。
- **移动端支持**：Tasks 插件全平台兼容。

---

通过以上步骤，您可以快速将 Obsidian 变成强大的任务管理系统。如需更复杂的查询，参考 [Tasks 官方文档](https://schemar.github.io/obsidian-tasks/)。