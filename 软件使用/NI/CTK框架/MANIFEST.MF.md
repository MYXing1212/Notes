在基于 **OSGi**（如CTK框架）或 **Java模块化系统** 中，`MANIFEST.MF` 文件是一个关键的**元数据配置文件**，用于定义模块（Bundle/Plugin）的属性和依赖关系。它的作用类似于软件的“身份证”和“说明书”，以下是通俗易懂的解析：
![[ctk框架之五（插件清单文件）.png]]

---

### **1. 核心作用**
#### **(1) 模块身份标识**
- **唯一ID**：指定模块的名称、版本、开发者等信息。
  ```plaintext
  Bundle-SymbolicName: com.example.imageviewer
  Bundle-Version: 1.0.0
  ```
  *类比*：就像APP的包名和版本号（如微信 `com.tencent.mm`）。

#### **(2) 依赖声明**
- **显式声明依赖**：说明该模块需要哪些其他模块或服务。
  ```plaintext
  Require-Bundle: org.ctk.core;bundle-version="2.0.0"
  Import-Package: com.example.logservice;version="1.5.0"
  ```
  *类比*：APP运行时需要调用的系统API（如需要GPS权限）。

#### **(3) 功能导出**
- **暴露接口**：声明该模块提供的可被其他模块使用的类或服务。
  ```plaintext
  Export-Package: com.example.imageviewer.api;version="1.0.0"
  ```
  *类比*：微信开放给其他APP调用的分享接口。

---

### **2. 典型内容示例**
```plaintext
Manifest-Version: 1.0
Bundle-ManifestVersion: 2
Bundle-Name: Image Viewer Plugin
Bundle-SymbolicName: com.example.imageviewer
Bundle-Version: 1.0.0
Bundle-Activator: com.example.Activator
Require-Bundle: org.ctk.core;bundle-version="2.0.0"
Export-Package: com.example.imageviewer.ui;version="1.0.0"
Import-Package: com.example.logservice;version="1.5.0"
```

---

### **3. 为什么需要它？**
| 场景                 | 作用                                                                 |
|----------------------|----------------------------------------------------------------------|
| **模块加载**         | 框架通过该文件识别插件是否合法、是否满足依赖。                       |
| **依赖解析**         | 确保所有依赖的模块/包已存在，避免运行时错误。                        |
| **热插拔**           | 允许动态安装/卸载模块，框架通过元数据管理生命周期。                  |
| **版本控制**         | 精确指定依赖版本，避免兼容性问题（如 `Bundle-Version`）。            |

---

### **4. 在CTK/OSGi中的实际应用**
#### **(1) 插件开发**
- 每个插件（`.jar` 或 `.so`）必须包含 `MANIFEST.MF`。
- 文件需放在 `META-INF/` 目录下。

#### **(2) 框架行为**
1. **启动时**：CTK框架读取所有插件的 `MANIFEST.MF`，构建依赖关系图。
2. **运行时**：根据文件中的 `Export-Package` 和 `Import-Package` 动态解析服务。

---

### **5. 类比理解**
| `MANIFEST.MF` 条目      | 类比现实场景                          |
|-------------------------|---------------------------------------|
| `Bundle-SymbolicName`   | APP的包名（如 `com.tencent.mm`）      |
| `Require-Bundle`        | APP依赖的第三方库（如需要支付宝SDK）  |
| `Export-Package`        | APP提供的公开功能（如微信支付接口）   |
| `Bundle-Version`        | APP的版本号（如微信8.0.2）           |

---

### **6. 常见问题**
- **Q：如果依赖不满足会怎样？**  
  A：框架会拒绝加载该插件，并报错（如 `Missing requirement`）。

- **Q：文件格式有何要求？**  
  A：必须使用严格的键值对，每行不超过72字符，换行需以空格开头。

---

### **总结**
`MANIFEST.MF` 是模块化程序的**核心配置文件**，它：  
1. **声明身份**（名称/版本）  
2. **管理依赖**（需要什么、提供什么）  
3. **确保安全加载**（避免冲突或缺失依赖）  

在CTK/OSGi中，它是实现 **插件动态加载** 和 **服务解耦** 的基石。