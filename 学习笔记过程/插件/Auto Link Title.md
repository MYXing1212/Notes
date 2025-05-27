### **Obsidian Auto Link Title 插件用法详解**

**Auto Link Title** 是 Obsidian 的一个实用插件，它能自动将粘贴的 URL 转换为包含网页标题的 Markdown 链接格式（如 `[网页标题](URL)`），提升笔记整理效率。

---

## **1. 安装插件**
1. 打开 Obsidian → **设置** → **社区插件** → **浏览**。
2. 搜索 **"Auto Link Title"** → 点击 **安装**。
3. 安装后 **启用插件**，并重启 Obsidian。

---

## **2. 基本用法**
### **自动转换粘贴的 URL**
1. 直接粘贴一个 URL（如 `https://obsidian.md`）到笔记中。
2. 插件会自动抓取网页标题，并转换为：
   ```markdown
   [Obsidian - Sharpen your thinking](https://obsidian.md)
   ```

### **手动触发转换**
如果自动转换未生效：
1. 选中已粘贴的 URL。
2. 按 `Ctrl/Cmd+P`，输入 **"Auto Link Title: Fetch link title"** 并执行。

---

## **3. 插件设置**
进入 **设置 → Auto Link Title**，可配置以下选项：
| 选项 | 功能 |
|------|------|
| **API 服务** | 默认使用 `opengraph.io`，可切换为其他服务（如 `Embedly`）。 |
| **请求超时时间** | 调整抓取标题的超时时间（默认 5 秒）。 |
| **黑名单域名** | 禁止对某些域名自动转换（如 `localhost`）。 |
| **自定义格式** | 修改链接生成的 Markdown 样式（如移除括号）。 |

---

## **4. 常见问题**
### **1. 插件不工作？**
- **检查网络**：插件需要联网抓取标题，确保代理/VPN 未拦截请求。
- **查看错误日志**：按 `Ctrl/Cmd+Shift+I` 打开开发者工具，观察 Console 是否有报错（如 `CORS` 错误）。
- **更换 API 服务**：在设置中尝试切换 API（如从 `opengraph.io` 换成 `Embedly`）。

### **2. 如何禁用特定网站的自动转换？**
- 在设置中添加域名到 **黑名单**（如 `twitter.com`）。

### **3. 支持离线使用吗？**
- 不支持。插件依赖网络请求获取标题，但可禁用网络请求（设置中关闭 `Enable network requests`），仅处理本地链接。

---

## **5. 高级技巧**
### **自定义链接格式**
在设置中修改 **"Link Format"**，例如：
- 默认格式：`[%title%](%url%)`
- 移除标题括号：`%title% %url%`

### **批量转换现有 URL**
1. 选中多个未转换的 URL。
2. 执行命令 **"Auto Link Title: Fetch link titles in selection"**。

---

## **6. 替代方案**
如果插件失效，可临时手动操作：
1. 粘贴 URL 后，用快捷键 `Ctrl/Cmd+K` 手动添加链接。
2. 使用 **Templater** 插件编写自动化脚本。

---

## **总结**
- **核心功能**：一键将 URL 转为带标题的 Markdown 链接。
- **适用场景**：整理网页引用、快速生成笔记参考文献。
- **推荐设置**：保持默认 API + 超时 5 秒，黑名单屏蔽无效域名。

通过 Auto Link Title，你可以彻底告别手动复制网页标题的繁琐操作！