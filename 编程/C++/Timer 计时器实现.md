
Header Only
析构时自动打印执行时间

```cpp
#pragma once
#include <chrono>
#include <iostream>
#include <string>

class Timer {
public:
    // 构造函数：记录开始时间，可选指定标签
    explicit Timer(std::string name = "Block")
        : name_(std::move(name)),
        start_(std::chrono::high_resolution_clock::now()) {
    }

    // 析构函数：自动计算并打印耗时
    ~Timer() {
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start_);
        std::cout << name_ << " took: " << duration.count() << " μs\n";
    }

    // 禁止拷贝和移动
    Timer(const Timer&) = delete;
    Timer& operator=(const Timer&) = delete;

private:
    std::string name_;
    std::chrono::time_point<std::chrono::high_resolution_clock> start_;
};

```

要实现 **三角面片模型的高效渲染**，并支持 **部分三角形高亮**，可以结合 **VAO（顶点数组对象）、VBO（顶点缓冲区对象）、EBO（元素缓冲区对象）** 进行优化，同时利用 **Shader（着色器）** 动态控制高亮效果。  

---

## **1. 核心思路**
1. **使用 VAO + VBO + EBO 管理模型数据**  
   - **VBO** 存储顶点数据（位置、颜色、法线等）。  
   - **EBO** 存储三角形索引（减少顶点重复）。  
   - **VAO** 记录顶点属性配置，提高渲染效率。  

2. **标记高亮三角形**  
   - 方法 1：**额外属性标记**（在顶点或索引数据中标记高亮三角形）。  
   - 方法 2：**多 EBO 分离渲染**（普通 EBO + 高亮 EBO）。  
   - 方法 3：**Shader 动态计算高亮**（传入高亮索引，在片段着色器判断）。  

3. **Shader 控制高亮效果**  
   - 在片段着色器 (`Fragment Shader`) 中，根据标记数据（如 `highlight` 属性）调整颜色（如变红/变亮）。  

---

## **2. 实现方案（推荐方法 2：多 EBO 分离渲染）**
### **(1) 数据准备**
- **顶点数据（VBO）**  
  ```cpp
  float vertices[] = {
      // 位置           // 颜色（默认）
      -0.5f, -0.5f, 0.0f, 0.5f, 0.5f, 0.5f,  // 顶点 0
       0.5f, -0.5f, 0.0f, 0.5f, 0.5f, 0.5f,  // 顶点 1
       0.0f,  0.5f, 0.0f, 0.5f, 0.5f, 0.5f,  // 顶点 2
      // ... 更多顶点
  };
  ```
- **索引数据（EBO）**  
  ```cpp
  // 普通三角形索引
  unsigned int indices[] = {
      0, 1, 2,  // 普通三角形 0
      3, 4, 5,  // 普通三角形 1
      // ...
  };
  
  // 高亮三角形索引（单独存储）
  unsigned int highlightIndices[] = {
      6, 7, 8,  // 高亮三角形 0
      9, 10, 11,// 高亮三角形 1
      // ...
  };
  ```

### **(2) 初始化 VAO/VBO/EBO**
```cpp
unsigned int VAO, VBO, EBO, highlightEBO;
glGenVertexArrays(1, &VAO);
glGenBuffers(1, &VBO);
glGenBuffers(1, &EBO);
glGenBuffers(1, &highlightEBO);  // 单独的高亮 EBO

// 绑定 VAO
glBindVertexArray(VAO);

// 填充 VBO（顶点数据）
glBindBuffer(GL_ARRAY_BUFFER, VBO);
glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

// 填充普通 EBO
glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

// 填充高亮 EBO
glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, highlightEBO);
glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(highlightIndices), highlightIndices, GL_STATIC_DRAW);

// 配置顶点属性（位置 + 颜色）
glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);  // 位置
glEnableVertexAttribArray(0);
glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));  // 颜色
glEnableVertexAttribArray(1);

// 解绑 VAO（避免意外修改）
glBindVertexArray(0);
```

### **(3) 渲染逻辑**
```cpp
// 绑定 VAO（自动关联 VBO 和顶点属性）
glBindVertexArray(VAO);

// 1. 先渲染普通三角形（使用普通 EBO）
glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
glDrawElements(GL_TRIANGLES, sizeof(indices)/sizeof(unsigned int), GL_UNSIGNED_INT, 0);

// 2. 再渲染高亮三角形（使用高亮 EBO）
glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, highlightEBO);
glDrawElements(GL_TRIANGLES, sizeof(highlightIndices)/sizeof(unsigned int), GL_UNSIGNED_INT, 0);

// 解绑 VAO
glBindVertexArray(0);
```

### **(4) Shader 控制高亮（可选）**
如果希望高亮效果更灵活（如闪烁、变色），可以在 **片段着色器** 中处理：
```glsl
#version 330 core
in vec3 vColor;
out vec4 FragColor;

uniform bool isHighlight;  // 是否高亮

void main() {
    if (isHighlight) {
        FragColor = vec4(1.0, 0.0, 0.0, 1.0);  // 高亮时变红
    } else {
        FragColor = vec4(vColor, 1.0);  // 普通颜色
    }
}
```
在渲染高亮部分时，设置 `isHighlight = true`：
```cpp
shader.setBool("isHighlight", true);
glDrawElements(GL_TRIANGLES, ...);  // 渲染高亮部分
shader.setBool("isHighlight", false);
```

---

## **3. 优化建议**
1. **批处理（Batching）**  
   - 如果高亮部分频繁变化，可以用 **动态 EBO**（`GL_DYNAMIC_DRAW`）更新索引数据。  
2. **实例化渲染（Instancing）**  
   - 如果高亮部分较多，可以用 **实例化渲染（glDrawElementsInstanced）** 提高性能。  
3. **Stencil Buffer（模板缓冲）**  
   - 用模板测试标记高亮区域，再叠加高亮颜色（适合复杂高亮效果）。  

---

## **4. 总结**
| 方法 | 适用场景 | 优点 | 缺点 |
|------|---------|------|------|
| **多 EBO 分离渲染** | 高亮部分固定 | 高效，直接渲染 | 需额外 EBO |
| **Shader 动态判断** | 高亮部分动态变化 | 灵活 | 需额外 uniform 变量 |
| **Stencil Buffer** | 复杂高亮（描边、发光） | 视觉效果强 | 实现较复杂 |

推荐 **多 EBO + Shader 控制**，平衡性能和灵活性。如果有更复杂的需求（如动态高亮），可以结合 **计算着色器（Compute Shader）** 或 **GPU 粒子系统** 进一步优化。 🚀