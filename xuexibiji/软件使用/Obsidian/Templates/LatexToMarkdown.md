
<%*
/* 本脚本用于，将MathJax格式渲染的公式，即
单行 \( a+b )\
多行 \[ a+b ]\
的格式，转为markdown中可使用的latex+格式，即
单行 $a+b$
多行 $$ a+b $$*/

// 读取本文件内容
const content = tp.file.content;

// 第一个replace,匹配单行公式
const converted1 = content.replace(/\\\(\s*(.*?)\s*\\\)/g, '$$$1$$');

// 第二个replace，匹配多行公式
const converted2 = converted1.replace(/\\\[\s*([\s\S]*?)\s*\\\]/g, '$$$$ $1 $$$$');

// 将修改好的内容，在光标处添加一份。这样会使得文章长度变为两份，
// 需手动删除掉之前的没替换过的那部分。
await tp.file.cursor_append(converted2);
%>