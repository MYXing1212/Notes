要在 **Windows 11 的 VS Code** 中配置运行 Python 脚本时**自动使用你自定义的命令**（例如 `.\isaaclab.bat -p path/XXX.py`），可以通过 **VS Code 的 `tasks.json` 或 `launch.json`** 实现。但更简单、直接的方式是使用 **VS Code 的“Run”按钮自定义终端命令**。

下面是两种推荐方法：

---

## ✅ 方法一：使用 `tasks.json`（推荐，适用于任意脚本）

### 步骤 1：打开你的项目文件夹

确保 `isaaclab.bat` 在项目根目录（或你知道它的相对路径）。

### 步骤 2：创建 `.vscode/tasks.json`

在项目根目录下创建 `.vscode/tasks.json` 文件（如果不存在）：

```json
{
  "version": "2.0.0",
  "tasks": [
    {
      "label": "Run with isaaclab.bat",
      "type": "shell",
      "command": ".\\isaaclab.bat",
      "args": [
        "-p",
        "${file}"
      ],
      "group": "build",
      "presentation": {
        "echo": true,
        "reveal": "always",
        "focus": false,
        "panel": "new"
      },
      "options": {
        "cwd": "${workspaceFolder}"
      }
    }
  ]
}
```

> 🔍 说明：
> 
> - `${file}`：当前打开的 Python 文件路径（如 `E:\project\test.py`）
> - `cwd`：工作目录设为项目根目录（确保 `isaaclab.bat` 能被找到）
> - `panel: "new"`：每次运行新开终端，避免混杂

### 步骤 3：运行脚本

1. 打开你要运行的 `XXX.py`
2. 按下 **`Ctrl+Shift+P`** → 输入 **`Tasks: Run Task`**
3. 选择 **`Run with isaaclab.bat`**

✅ 效果：VS Code 会执行

```cmd
.\isaaclab.bat -p E:\your_project\XXX.py
```

---

## ✅ 最终效果

|操作|结果|
|---|---|
|打开 `demo.py` → `Ctrl+Shift+P` → Run Task → 选你的任务|执行 `.\isaaclab.bat -p demo.py`|
|绑定快捷键后按 `Ctrl+F5`|一键运行当前文件|
|使用集成终端|输出显示在 VS Code 底部面板|

---