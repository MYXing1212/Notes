### 第一步先install
否则会报错
```
 File "E:\Isaac\IssacLab\source\IsaacLabTutorial\scripts\skrl\train.py", line 103, in <module>
    import isaac_lab_tutorial.tasks  # noqa: F401
    ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
ModuleNotFoundError: No module named 'isaac_lab_tutorial'
```

安装的步骤是
```bash
cd E:\Isaac\IssacLab\source\IsaacLabTutorial
pip install -e .
```