
### 问题原因
可能是父窗口的Layout策略又影响了子Widget中控件的尺寸

### 终极方案
重写自定义控件的resizeEvent方法，再次设定一遍控件的size

```cpp
void MaskToolBarWidget::resizeEvent(QResizeEvent* event)
{
  QWidget::resizeEvent(event);
	ui->PushButton_clear_point->setFixedSize(30, 30);
	ui->PushButton_delete_point->setFixedSize(30, 30);
	ui->circlePushButton->setFixedSize(30, 30);
	ui->linePushButton->setFixedSize(30, 30);
	ui->pushButton_Comeback->setFixedSize(30, 30);
	ui->pushButton_left->setFixedSize(30, 30);
	ui->rectanglePushButton->setFixedSize(30, 30);
}
```