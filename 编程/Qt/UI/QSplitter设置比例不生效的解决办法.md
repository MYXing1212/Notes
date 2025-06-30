
```cpp
	// 添加左右部件到上层分割器
	upperSplitter->addWidget(leftWidget);
	upperSplitter->addWidget(rightWidget);

	QList<int> upperSplitterSizes;
	upperSplitterSizes << 10000 << 10000;  // 如果比例是4：3的话，这里改成 40000 30000
	upperSplitter->setSizes(upperSplitterSizes);
```