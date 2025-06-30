```cpp
//初始化UI
void MeasureModulesImpl::SLOT_InitMeasureUI()
{
	m_pMainWindow = getMainWindowInterface();
	if (!m_pMainWindow) {
		return;
	}

	GetControllerCore()->Init();

	// 初始化按钮组
	std::for_each(arrRibbonBtnInitInfo, arrRibbonBtnInitInfo + zgutils::ArrayEleNum(arrRibbonBtnInitInfo), [this](const zg_inspect::RibbonBtnInitInfo& item) {
		auto* pAct = m_pMainWindow->getActionByMenuID(QString::fromStdString(item.szBtnName));
		if (!pAct) {
			return;
		}

		m_mapRibbonBtn[item.eBtnId] = pAct;

		pAct->setCheckable(item.bCheckable);
		pAct->setChecked(item.bChecked);
		pAct->setEnabled(item.bEnabled);
		pAct->setData(QVariant((uint32_t)item.eBtnId));
		connect(pAct, SIGNAL(triggered(bool)), this, SLOT(SlotRibbonBtnClicked(bool)));
	});


	//初始化三维视口CentralWidget
	m_pMeasureRenderWidget = new MeasureRenderWidget(MeasureRenderWidget::WgtType::Base);

	//视图分割
	if (m_pWgtAlign != nullptr)
	{
		return;
	}

	if (m_pWgtAlign == nullptr)
{
	m_pWgtAlign = new MyWidget();
}

if (m_pWgtAlign->layout() != nullptr)
{
	delete m_pWgtAlign->layout();
}

//重新给当前窗口布局
auto layGrid = new QGridLayout();
layGrid->setMargin(0);//布局边距
m_pWgtAlign->setLayout(layGrid);

auto m_spliter = new QSplitter(Qt::Horizontal, 0);

m_pMsAlignedLeft = new RenderAlignedWidget(MeasureRenderWidget::WgtType::AlignLeft);
//m_pMsAlignedLeft->setObjectName("left");
//m_pMsAlignedLeft->setProperty("FLAG_ATT",0);//设置分割属性为可分割
//m_pMsAlignedLeft->setStyleSheet("#part_1{border:1px solid green}");//只设置QWidget样式作为窗口边界
auto lyLeft = new QGridLayout();
m_pMsAlignedLeft->setLayout(lyLeft);

m_pMsAlignedRight = new RenderAlignedWidget(MeasureRenderWidget::WgtType::AlignRight);
//m_pMsAlignedRight->setObjectName("right");
//m_pMsAlignedRight->setProperty("FLAG_ATT",0);
//m_pMsAlignedRight->setStyleSheet("#part_2{border:1px solid green}");
auto lyRight = new QGridLayout();
m_pMsAlignedRight->setLayout(lyRight);

//设置另一视图指针
m_pMsAlignedLeft->SetOtherRender(m_pMsAlignedRight);
m_pMsAlignedRight->SetOtherRender(m_pMsAlignedLeft);

m_spliter->addWidget(m_pMsAlignedLeft);
m_spliter->addWidget(m_pMsAlignedRight);
m_spliter->setStretchFactor(0, 1);//分割的水平窗口的水平比例为1:1
m_spliter->setStretchFactor(1, 1);
m_spliter->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);//大小策略
m_spliter->setHandleWidth(2);//分割线的宽度,间隔宽度
m_spliter->setChildrenCollapsible(false);//不允许把分割出的子窗口拖小到0，最小值被限定为sizeHint或maxSize/minSize
m_spliter->setOpaqueResize(true);//实时显示调整大小
layGrid->addWidget(m_spliter);

auto* pw = WgtOvlFactory::Create(m_pWgtAlign, WgtOvlFactory::Type::PtAlign, QWidgetOverlay::PosMark::TopCenter, 0, 10, 0, 0);
pw->setObjectName("WgtOvlPtAlign");
pw->Connect<QPushButton>("btn_exit", &QPushButton::clicked, [&] {
	m_pMsAlignedLeft->ExitAlignment();
});
pw->Connect<QPushButton>("btn_clear", &QPushButton::clicked, [&] {
	m_pMsAlignedLeft->ClearLastPt();
});
	pw->Connect<QPushButton>("btn_apply", &QPushButton::clicked, [&] {
		m_pMsAlignedLeft->ApplyAlign();
	});
	QObject::connect(m_pWgtAlign, &MyWidget::resized, [=](QSize newSize) {
		pw->AdjustPos();
	});

	m_pDataInfoPageWidget = new DataInfoPageWidget();//树视图控件
	m_pDataInfoPageDockWidget = new ads::CDockWidget(GetCommonString(CommonString::CS_NAVIGATION));
	m_pDataInfoPageDockWidget->setObjectName("DataInfoPageWidget");
	m_pDataInfoPageDockWidget->setWidget(m_pDataInfoPageWidget);
	// To Set Extended SubModules

	m_pCmdWidget = new CommandWidget();
	connect(m_pCmdWidget, &CommandWidget::SignalSizeChanged, this, [&] {
		m_pCmdDockWidget->dockAreaWidget()->setMinimumWidth(std::max(m_pCmdWidget->sizeHint().width() + 20, 300));
		});
	m_pCmdDockWidget = new ads::CDockWidget(GetCommonString(CommonString::CS_COMMAND_DIALOG));
	m_pCmdDockWidget->setObjectName("CommandDialog");
	m_pCmdDockWidget->setWidget(getCommandWidget());
	//实例化对话框资源
	m_pEmptyWidget = new QWidget();
	getCommandWidget()->addWidget(m_pEmptyWidget);

	m_cmdManager = new CCommandManager();

	//树视图控件
	m_pDataPropertyWidget = new DataPropertyWidget();
	m_pDataPropertyDockWidget = new ads::CDockWidget(GetCommonString(CommonString::CS_PROPERTY));
	m_pDataPropertyDockWidget->setObjectName("DataPropertyPageWidget");
	m_pDataPropertyDockWidget->setWidget(m_pDataPropertyWidget);
	m_pDataPropertyWidget->SetContainerWidget(m_pDataPropertyDockWidget);
	
	//MainWindow布局
	m_pMainWindow->addStackedCentralWidget(m_pMeasureRenderWidget);
	//作为非插件的独立软件则显示
	//m_pMainWindow->activateCentralWidget(m_pMeasureRenderWidget);
	m_pMainWindow->addDockWidget(ads::LeftDockWidgetArea, m_pDataInfoPageDockWidget);
	m_pMainWindow->addDockWidget(ads::LeftDockWidgetArea, m_pCmdDockWidget);
	m_pMainWindow->addDockWidget(ads::RightDockWidgetArea, m_pDataPropertyDockWidget);
}

```