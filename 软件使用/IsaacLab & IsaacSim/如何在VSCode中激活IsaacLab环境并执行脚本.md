
在VsCode的终端中执行下列命令

```bash
 .\isaaclab.bat -p .\scripts\environments\list_envs.py
```


在VSCode的终端Terminal中激活IsaacLab的python环境
在IsaacLab的文件夹下创建脚本activate_isaac.bat, 然后在Terminal中执行此脚本，即可激活python环境
```bash
@echo off
echo Activating Isaac Lab Python Environment...

set ISAAC_PYTHON=E:\Isaac\IssacLab\_isaac_sim\kit\python
set ISAAC_SCRIPTS=E:\Isaac\IssacLab\_isaac_sim\kit\python\Scripts

rem 将 Isaac 路径添加到 PATH 最前面
set PATH=%ISAAC_SCRIPTS%;%ISAAC_PYTHON%;%PATH%

rem 设置 PYTHONPATH
set PYTHONPATH=%ISAAC_PYTHON%

echo Isaac Lab Python is now active!
echo.
echo Python: %ISAAC_PYTHON%\python.exe
echo Pip: %ISAAC_SCRIPTS%\pip.exe
echo.
python --version
pip --version
cmd /k
```