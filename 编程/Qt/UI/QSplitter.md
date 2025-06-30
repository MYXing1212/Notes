![[Pasted image 20250616153828.png]]

```cpp
//MainWindow布局
// 中央区域使用QSplitter（左右分栏）
QSplitter* centralSplitter = new QSplitter(Qt::Horizontal);
QWidget* rightPanel = new QTextEdit("Properties");
centralSplitter->addWidget(m_pMeasureRenderWidget);
centralSplitter->addWidget(rightPanel);
m_pMainWindow->addStackedCentralWidget(centralSplitter);
//作为非插件的独立软件则显示
//m_pMainWindow->activateCentralWidget(m_pMeasureRenderWidget);
m_pMainWindow->addDockWidget(ads::LeftDockWidgetArea, m_pDataInfoPageDockWidget);
m_pMainWindow->addDockWidget(ads::LeftDockWidgetArea, m_pCmdDockWidget);
m_pMainWindow->addDockWidget(ads::RightDockWidgetArea, m_pDataPropertyDockWidget);
```