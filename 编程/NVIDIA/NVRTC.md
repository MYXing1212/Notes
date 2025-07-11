### **NVRTC 是什么？**
**NVRTC**（NVIDIA Runtime Compilation）是 NVIDIA 提供的 **运行时 CUDA 编译库**，允许开发者在程序运行时动态编译 CUDA C++ 代码（如字符串形式的 CUDA 内核），生成 PTX 或二进制代码（如 `cubin` 或 `LTOIR`），并立即加载到 GPU 执行。  

### NVRTC V12.5 Documentation
[Contents — NVRTC 12.5 documentation](https://docs.nvidia.com/cuda/archive/12.5.0/nvrtc/contents.html)

#### **核心功能**
1. **运行时编译**：无需提前用 `nvcc` 编译 `.cu` 文件，直接在内存中编译 CUDA 代码。
2. **动态代码生成**：适用于需要运行时生成或修改内核的场景（如参数化模板、JIT 优化）。
3. **轻量级**：相比静态编译（`nvcc`），减少依赖，适合插件化架构。

---

### **如何查看当前 NVRTC 版本？**
#### **方法 1：使用 `nvrtcVersion` API**
在代码中直接调用 NVRTC 提供的版本查询函数：
```cpp
#include <nvrtc.h>
#include <iostream>

int main() {
    int major, minor;
    nvrtcVersion(&major, &minor);
    std::cout << "NVRTC 版本: " << major << "." << minor << std::endl;
    return 0;
}
```
- **输出示例**：
  ```plaintext
  NVRTC 版本: 11.2
  ```

#### **方法 2：检查 CUDA Toolkit 版本**
NVRTC 是 CUDA Toolkit 的一部分，其版本通常与 CUDA 主版本一致：
```bash
nvcc --version
```
- **输出示例**：
  ```plaintext
  nvcc: NVIDIA (R) Cuda compiler version 11.5.119
  ```
  - 此例中，NVRTC 版本约为 `11.5`。

#### **方法 3：查找动态库版本（Linux/Windows）**
- **Linux**：
  ```bash
  strings /usr/local/cuda/lib64/libnvrtc.so | grep "NVRTC_"
  ```
- **Windows**：
  ```powershell
  strings "C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v11.5\bin\nvrtc64_115_0.dll" | findstr "NVRTC_"
  ```
- **输出示例**：
  ```plaintext
  NVRTC_11.5.50
  ```

---

### **NVRTC 的典型使用流程**
```cpp
#include <nvrtc.h>
#include <cuda.h>

int main() {
    // 1. 创建程序对象
    nvrtcProgram prog;
    const char* kernel_code = R"(
        __global__ void add(int *a, int *b, int *c) {
            int tid = blockIdx.x * blockDim.x + threadIdx.x;
            c[tid] = a[tid] + b[tid];
        }
    )";
    nvrtcCreateProgram(&prog, kernel_code, "add.cu", 0, NULL, NULL);

    // 2. 编译选项（如架构）
    const char* opts[] = {"--gpu-architecture=compute_89"};

    // 3. 编译代码
    nvrtcResult res = nvrtcCompileProgram(prog, 1, opts);

    // 4. 获取 PTX 代码
    size_t ptxSize;
    nvrtcGetPTXSize(prog, &ptxSize);
    char* ptx = new char[ptxSize];
    nvrtcGetPTX(prog, ptx);

    // 5. 加载到 GPU
    CUmodule module;
    cuModuleLoadData(&module, ptx);

    // ...（执行内核等其他操作）

    // 6. 清理资源
    nvrtcDestroyProgram(&prog);
    delete[] ptx;
    return 0;
}
```

---

### **常见问题**
#### **Q1: NVRTC 和 `nvcc` 有什么区别？**
| 特性                | NVRTC                          | NVCC                     |
|---------------------|--------------------------------|--------------------------|
| **编译时机**         | 运行时动态编译                 | 静态编译（构建时）       |
| **输入**            | 字符串形式的 CUDA 代码         | `.cu` 源文件             |
| **输出**            | PTX/cubin/LTOIR               | 可执行文件或对象文件     |
| **依赖**            | 仅需 CUDA 运行时               | 需完整 CUDA Toolkit      |

#### **Q2: NVRTC 支持哪些 CUDA 特性？**
- 支持大多数 CUDA C++ 语法（如 `__global__`、`__shared__`）。
- 部分特性需显式启用（如 C++17：`--std=c++17`）。
- 不支持完整的宿主代码编译（如不能编译 `main()` 函数）。

#### **Q3: 如何调试 NVRTC 编译错误？**
```cpp
size_t logSize;
nvrtcGetProgramLogSize(prog, &logSize);
char* log = new char[logSize];
nvrtcGetProgramLog(prog, log);
std::cerr << "编译错误:\n" << log << std::endl;
```

---

### **总结**
- **NVRTC 版本**：通过 `nvrtcVersion()` 或 CUDA Toolkit 版本推断。
- **核心用途**：运行时动态编译 CUDA 代码，适合灵活的内核生成。
- **调试技巧**：始终检查 `nvrtcGetProgramLog` 的输出。