
CUDA 11.8 + PyTorch 2.6.0 + Python 3.10

#### 1.     LINK : fatal error LNK1181: 无法打开输入文件“aio.lib”
      test.c
      LINK : fatal error LNK1181: 无法打开输入文件“cufile.lib”

![[Pasted image 20250522145123.png]]
解决方法，增加环境变量
![[Pasted image 20250522150125.png]]

#### 2. 
Collecting deepspeed (from -r requirements.txt (line 19))
  Using cached https://pypi.tuna.tsinghua.edu.cn/packages/e1/27/f857569546dc97f64169df1cc35d1ec5161766c4a0ec32f2af5d39614a51/deepspeed-0.16.8.tar.gz (1.5 MB)
  Preparing metadata (setup.py) ... error
  error: subprocess-exited-with-error

  × python setup.py egg_info did not run successfully.
  │ exit code: 1
  ╰─> [21 lines of output]
      [2025-05-22 14:59:59,511] [INFO] [real_accelerator.py:239:get_accelerator] Setting ds_accelerator to cuda (auto detect)
      [2025-05-22 14:59:59,805] [INFO] [real_accelerator.py:239:get_accelerator] Setting ds_accelerator to cuda (auto detect)
      test.c
      LINK : fatal error LNK1181: 无法打开输入文件“aio.lib”
      test.c
      LINK : fatal error LNK1181: 无法打开输入文件“cufile.lib”
      Traceback (most recent call last):
               ext_modules.append(builder.builder())
        File "C:\Users\mingyi.xing\AppData\Local\Temp\pip-install-hsbw5tm_\deepspeed_c91d6a8f6b9b48dcabf8c4d68c4c188f\op_builder\builder.py", line 730, in builder
          extra_link_args=self.strip_empty_entries(self.extra_ldflags()))
        File "C:\Users\mingyi.xing\AppData\Local\Temp\pip-install-hsbw5tm_\deepspeed_c91d6a8f6b9b48dcabf8c4d68c4c188f\op_builder\inference_cutlass_builder.py", line 74, in extra_ldflags
          import dskernels
      ModuleNotFoundError: No module named 'dskernels'
      DS_BUILD_OPS=1
       [WARNING]  Skip pre-compile of incompatible evoformer_attn; One can disable evoformer_attn with DS_BUILD_EVOFORMER_ATTN=0
       [WARNING]  Skip pre-compile of incompatible fp_quantizer; One can disable fp_quantizer with DS_BUILD_FP_QUANTIZER=0
      [WARNING]  Skip pre-compile of incompatible gds; One can disable gds with DS_BUILD_GDS=0
       [WARNING]  Filtered compute capabilities [['6', '0'], ['6', '1'], ['7', '0']]
      [end of output]

  note: This error originates from a subprocess, and is likely not a problem with pip.
error: metadata-generation-failed

解决方法 增加环境变量
	set DS_BUILD_OPS=0


#### 问题3
File "C:\Users\mingyi.xing\AppData\Local\Temp\pip-install-y3u9sfvq\deepspeed_3554f53ca9e044d2ba309a27f947748c\setup.py", line 208, in \<module>
          if command_exists('git') and not is_env_set('DS_BUILD_STRING'):
        File "C:\Users\mingyi.xing\AppData\Local\Temp\pip-install-y3u9sfvq\deepspeed_3554f53ca9e044d2ba309a27f947748c\setup.py", line 163, in command_exists
          result = subprocess.Popen(safe_cmd, stdout=subprocess.PIPE)
        File "C:\Users\mingyi.xing\AppData\Local\Programs\Python\Python310\lib\subprocess.py", line 971, in __init__
          self._execute_child(args, executable, preexec_fn, close_fds,
        File "C:\Users\mingyi.xing\AppData\Local\Programs\Python\Python310\lib\subprocess.py", line 1456, in _execute_child
          hp, ht, pid, tid = _winapi.CreateProcess(executable, args,
      FileNotFoundError: [WinError 2] 系统找不到指定的文件。
      DS_BUILD_OPS=0
      Install Ops={'async_io': False, 'fused_adam': False, 'cpu_adam': False, 'cpu_adagrad': F 如何修复

<span style="font-size: 20px; color: orange">解决方法</span> 
	在PyCharm中配置Git.exe的路径到环境变量中
![[Pasted image 20250522155100.png|725]]

根据你提供的错误信息，问题主要集中在两个方面：

1. **`cl: 命令行 warning D9002 :忽略未知选项“-g”`**  
   这是因为在编译过程中，`-g` 是 GCC 编译器的选项，用于生成调试信息，但 Microsoft Visual C++ 编译器（cl.exe）不支持这个选项。

2. **`fatal error C1083: 无法打开包括文件: “stddef.h”: No such file or directory`**  
   这是因为编译器找不到 `stddef.h` 文件，这通常是由于环境配置不正确或缺少必要的头文件。

### 修复步骤

#### 1. 修复 `-g` 选项问题
`-g` 选项是 GCC 编译器的调试信息生成选项，而 Windows 的 MSVC 编译器不支持它。你需要修改代码的 `setup.py` 文件，移除 `-g` 选项。

- 找到 `setup.py` 文件（通常在需要编译的模块目录下）。
- 打开 `setup.py` 文件，查找与编译器选项相关的代码，例如：
  ```python
  extra_compile_args=['-g']
  ```
- 将 `-g` 选项移除或替换为 MSVC 支持的调试选项（如 `/Zi`）：
  ```python
  extra_compile_args=['/Zi']
  ```

#### 2. 修复 `fatal error C1083: 无法打开包括文件: “stddef.h”` 问题
这个错误通常是因为编译器找不到标准头文件 `stddef.h`。这可能是由于以下原因之一：
- 编译器环境变量未正确配置。
- 缺少必要的 Windows SDK 或 C++ 标准库。

##### a. 确保 Visual Studio 和 Windows SDK 安装正确
- 打开 Visual Studio Installer。
- 确保安装了以下组件：
  - **C++ 开发工具**（包括 MSVC 和 Windows SDK）。
  - **Windows 10 SDK**（或更高版本）。

##### b. 配置环境变量
确保 Visual Studio 的编译器路径和 Windows SDK 路径已添加到环境变量中。可以通过以下步骤检查和配置：
1. 打开“系统属性” > “高级” > “环境变量”。
2. 在“系统变量”中，确保 `PATH` 包含以下路径：
   ```
   C:\Program Files\Microsoft Visual Studio\2022\Professional\VC\Tools\MSVC\14.44.35207\bin\Hostx64\x64
   C:\Program Files (x86)\Windows Kits\10\Include\<version>\ucrt
   ```
   其中 `<version>` 是你的 Windows SDK 版本号。

##### c. 使用 Visual Studio 的开发人员命令提示符
为了确保环境变量正确，建议使用 Visual Studio 的开发人员命令提示符来运行编译命令：
1. 打开“开始”菜单，找到 **“Visual Studio 2022”** > **“x64 Native Tools Command Prompt for VS 2022”**。
2. 在该命令提示符中运行你的编译命令。

### 3. 在 PyCharm 中运行编译命令
如果你希望在 PyCharm 中运行编译命令，可以确保 PyCharm 使用正确的终端环境：
1. 打开 PyCharm，进入 **“设置”** > **“工具”** > **“终端”**。
2. 在“Shell path”中，指定 Visual Studio 的开发人员命令提示符路径：
   ```
   C:\Program Files\Microsoft Visual Studio\2022\Professional\Common7\Tools\VsDevCmd.bat
   ```

### 4. 测试修复
在修复上述问题后，重新运行编译命令：
```bash
python setup.py install
```
如果一切正常，编译应该能够成功完成。

### 注意事项
- 如果问题仍然存在，可以尝试更新 Visual Studio 和 Windows SDK 到最新版本。
- 如果你使用的是 Python 的虚拟环境，确保虚拟环境中的 Python 解释器与系统中的 Python 解释器版本一致。
- 如果需要进一步调试，可以查看 `setup.py` 文件中的编译器选项，确保它们与你的开发环境兼容。


这个错误表明 PyCharm 在编译 `ball_query.cpp` 时，MSVC 编译器找不到关键头文件 `stddef.h`，通常是由于 **Visual Studio 环境配置不完整** 或 **路径未被正确识别** 导致。以下是详细解决方案：

---

### **1. 确认 Visual Studio 2022 安装完整**
- **步骤**：
  1. 打开 **Visual Studio Installer**。
  2. 找到已安装的 `Visual Studio 2022 Professional`，点击 **修改 (Modify)**。
  3. 确保勾选以下组件：
     - **"使用 C++ 的桌面开发"** (Desktop development with C++)
     - **"Windows 10/11 SDK"**（版本需与 CUDA 11.8 兼容，如 10.0.22000.0）
     - **"MSVC v143 - VS 2022 C++ x64/x86 生成工具"**（必须包含）
  4. 点击 **修改** 完成安装。

---

### **2. 检查头文件路径**
`stddef.h` 应位于以下路径中（根据实际 MSVC 版本调整）：
```
C:\Program Files\Microsoft Visual Studio\2022\Professional\VC\Tools\MSVC\14.44.35207\include\
```
**若文件缺失**：
- 重新安装 Visual Studio（步骤 1）。
- 或从其他正常机器复制 `stddef.h` 到该目录。

---

### **3. 在 PyCharm 中配置 MSVC 环境**
#### **(1) 设置正确的编译器路径**
1. 打开 PyCharm → `File → Settings → Build, Execution, Deployment → Toolchains`。
2. 选择 **Visual Studio**，确保路径指向：
   ```
   C:\Program Files\Microsoft Visual Studio\2022\Professional\VC\Auxiliary\Build\vcvarsall.bat
   ```
3. 在 `Environment variables` 中添加：
   ```
   INCLUDE=C:\Program Files\Microsoft Visual Studio\2022\Professional\VC\Tools\MSVC\14.44.35207\include;C:\Program Files (x86)\Windows Kits\10\Include\10.0.22000.0\ucrt
   LIB=C:\Program Files\Microsoft Visual Studio\2022\Professional\VC\Tools\MSVC\14.44.35207\lib\x64;C:\Program Files (x86)\Windows Kits\10\Lib\10.0.22000.0\ucrt\x64
   ```

#### **(2) 使用 VS 开发人员终端**
1. 在 PyCharm 的终端中，手动运行 VS 开发人员命令提示符：
   ```bash
   "C:\Program Files\Microsoft Visual Studio\2022\Professional\Common7\Tools\VsDevCmd.bat" -arch=x64
   ```
2. 然后在激活的环境中执行编译命令（如 `pip install` 或 `python setup.py build`）。

---

### **4. 验证环境配置**
在 PyCharm 的终端中运行以下命令，确认路径正确：
```bash
cl.exe /nologo /E ball_query.cpp  # 应无报错
echo %INCLUDE%                   # 检查包含路径
```

---

### **5. 常见问题排查**
#### **问题 1：PyCharm 未继承系统环境变量**
- **解决**：
  1. 在 PyCharm 的 `Run/Debug Configurations` 中，手动添加环境变量：
     ```
     PATH=C:\Program Files\Microsoft Visual Studio\2022\Professional\VC\Tools\MSVC\14.44.35207\bin\Hostx64\x64;%PATH%
     ```
  2. 重启 PyCharm。

#### **问题 2：Windows SDK 版本不匹配**
- **检查路径**：
  确保 `C:\Program Files (x86)\Windows Kits\10\Include\` 下的 SDK 版本与 `INCLUDE` 变量一致（如 `10.0.22000.0`）。

#### **问题 3：CUDA 与 MSVC 版本冲突**
- **CUDA 11.8 要求**：  
  - 必须使用 **VS 2022 (MSVC v143)**，避免使用更高版本（如 17.5+）。
  - 若已安装高版本 VS，需降级或通过 `vcvarsall.bat` 强制指定工具链：
    ```bash
    "C:\Program Files\Microsoft Visual Studio\2022\Professional\VC\Auxiliary\Build\vcvarsall.bat" x64 -vcvars_ver=14.34
    ```

---

### **6. 终极解决方案**
如果问题持续，尝试以下步骤：
1. **完全卸载并重装 Visual Studio 2022**（确保勾选所有 C++ 组件）。
2. **使用 Conda 环境**（隔离 Python 和编译器依赖）：
   ```bash
   conda create -n cuda_env python=3.10
   conda activate cuda_env
   conda install -c conda-forge cudatoolkit=11.8
   pip install torch==2.0.1+cu118 --index-url https://download.pytorch.org/whl/cu118
   ```
3. **在 PyCharm 中绑定 Conda 环境**：
   - `File → Settings → Project → Python Interpreter` → 选择 Conda 环境。

---

### **总结**
| 步骤 | 操作 |
|------|------|
| 1. 检查 VS 安装 | 确保 C++ 组件和 SDK 已安装 |
| 2. 配置 PyCharm | 设置编译器路径和环境变量 |
| 3. 使用 VS 终端 | 通过 `VsDevCmd.bat` 激活环境 |
| 4. 验证路径 | 运行 `cl.exe /E ball_query.cpp` |

如果仍报错，请提供：
1. 完整的编译命令和错误日志。
2. `cl.exe /?` 和 `nvcc --version` 的输出。