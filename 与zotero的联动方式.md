[与zotero的联动方式](https://huaweicloud.csdn.net/638f11fbdacf622b8df8e767.html?spm=1001.2101.3001.6650.1&utm_medium=distribute.pc_relevant.none-task-blog-2%7Edefault%7EBlogCommendFromBaidu%7Eactivity-1-125705200-blog-138642131.235%5Ev43%5Epc_blog_bottom_relevance_base7&depth_1-utm_source=distribute.pc_relevant.none-task-blog-2%7Edefault%7EBlogCommendFromBaidu%7Eactivity-1-125705200-blog-138642131.235%5Ev43%5Epc_blog_bottom_relevance_base7&utm_relevant_index=2)

- zotero通过Better-BibTeX软件生成JSON文件导入至Obsidian文件目录，Obsidian插件Bibnotes Formatter读取JSON文件信息在用户命令下生成指定文献的md格式笔记，笔记中包含有`{{localLabraryLink}}`（zotero中文献条目链接）和`{{file}}`(zoterp中文献pdf链接)可以从Obsidian中直接打开zotero查看文献，而zotero通过MarkDBConnect插件搜索Obsidian中的`@{{citeKey}}`格式文件名将zotero中的文献条目与Obsidian中的文献笔记关联实现联动。

![[Pasted image 20250525140940.png|400]]


- 准备工作：
	- ### zotero-better-bibtex
	- ### Bibnotes Formatter
	- ### MarkDBConnect


- 实现联动