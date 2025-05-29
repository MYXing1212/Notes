
**📅 DateTime**  2025-05-29 10:19           **👤 Author    Xing Mingyi  


# 软件设置

### 1. 修改粘贴图片默认存储路径 
配合[[File Explorer++]]插件使用，隐藏列表中粘贴图片项
![[Pasted image 20250526150914.png]]

### 2. 禁用编辑器自动格式化（可选）

 在 Obsidian 中，输入 `1.` 后按空格会自动转换为有序列表格式，这是 Markdown 的标准行为。如果你想去掉这种自动转换，

【设置】-> 【Editor】-> Smart lists 关闭

![[Pasted image 20250526141746.png]]


### 3. 更改主题 （可选）
![[Pasted image 20250526150331.png]]

![[Pasted image 20250526150403.png]]

- 主题启用之后图片自动居中
- 字体添加
---


# 安装插件
### 1. Editing Toolbar
![[Pasted image 20250528091601.png]]

### 2. Git 
参见 [[Git 笔记同步]]

### 3. BART 
方便从Github中安装一些插件（Obsidian插件浏览中搜不到的）
参见 [[BART —— 拖动Note调整顺序]] 实现可以在左侧的目录中自由拖动Note，改变Note的排序位置（默认不支持拖动排序）

### 4. Image Toolkit + Mousewheel Image Zoom
图片查看和简单编辑
![[Pasted image 20250527155618.png|650]]

### 5. [[Advanced Codeblock]]
代码块显示行号
部分代码行高亮（不常用）

### 6. [[Latex Suite 加速公式输入]]
公式编辑 

### 7. Recent Files 
显示最近使用的文件
![[Pasted image 20250527155756.png]]

### 8. Style Settings
配合Blue Topaz使用，甚好
![[Pasted image 20250527160820.png|362]]

### 9.   Insert New Line
配合设置快捷键，在当前行之前插入行，在当前行之后插入行；安装并启用插件后，见下一节配置快捷键内容。

### 10. Templates [[插入模板]] 
插入模板，提高效率

### 11. [[File Explorer++]]
置顶 / 隐藏
用法一：隐藏左侧列表中的粘贴图片，操作见链接
用法二：不关注文件夹/笔记，Hide；关注的置顶

### 11. Enhancing Mindmap （可选）
思维导图

### 12. [[Excalidraw]] （可选）
绘图 

### 13. Auto Link Title （可选）
[[Auto Link Title]] 需重启Obsidian生效
将粘贴的URL网址，自动解析其Title，并替换





---
# 配置快捷键

### 1. 切换source mode;
![[Pasted image 20250526135602.png]]

### 2. 在当前行前插入行 / 在当前行后插入行
需先安装Insert New Line插件
![[Pasted image 20250526142736.png]]

---

# 注意事项  
## 1. 请不要在公共仓库内上传较大的文件
建议将大文件保存在网盘，公共仓库只保留文件链接

## 2. 更新公共仓库的标准动作
推荐使用SourceTree进行更新
Step 1   查看本地仓库有无未提交更改，若有更改，先本地提交。防止拉取远端时，将本地修改冲掉；
Step 2  拉取远端，可勾上变基；
Step 3  推送远端；

## 3. 公共仓库仅推送[[研究方向]]相关的技术资料
典型内容包括
1）代码片段，最好能实现某一具体的功能；
2）解bug，配环境踩坑记录；
3）文献阅读笔记；
4）好用的软件、在线工具等，软件使用心得技巧；
5）推荐的学习资料，比如CSDN博客/资源，Github项目，给出链接即可；

## 4. 不推送公共仓库的几种情况
1. 个人文件建议建立个人私人仓库；
2. 在公共仓库对应的本地目录，需要添加文件，但仅自己使用，不推送远端的情况。
可右键笔记，选择添加至.gitignore

![[Pasted image 20250529100800.png]]


## 5. 鼓励写笔记时链接其他笔记内容


