### 计算变换后点集的 Bounding Box（C#实现）

以下是计算一组2D坐标点经过变换矩阵后在新坐标系下的边界框（`xmin, xmax, ymin, ymax`）的完整C#代码：

```csharp
using System;
using System.Collections.Generic;

public class BoundingBoxCalculator
{
    public static BoundingBox GetTransformedBoundingBox(
        List<PointF> originalPoints, 
        float[,] transformMatrix)
    {
        if (originalPoints == null || originalPoints.Count == 0)
            throw new ArgumentException("点集不能为空");
        
        if (transformMatrix.GetLength(0) != 3 || transformMatrix.GetLength(1) != 3)
            throw new ArgumentException("变换矩阵必须是3x3矩阵");

        float xmin = float.MaxValue, xmax = float.MinValue;
        float ymin = float.MaxValue, ymax = float.MinValue;

        foreach (var point in originalPoints)
        {
            // 应用变换矩阵 (齐次坐标变换)
            float x = transformMatrix[0, 0] * point.X + 
                      transformMatrix[0, 1] * point.Y + 
                      transformMatrix[0, 2];
            
            float y = transformMatrix[1, 0] * point.X + 
                      transformMatrix[1, 1] * point.Y + 
                      transformMatrix[1, 2];
            
            // 透视除法（如果是仿射变换，则w=1）
            float w = transformMatrix[2, 0] * point.X + 
                      transformMatrix[2, 1] * point.Y + 
                      transformMatrix[2, 2];
            
            if (Math.Abs(w) > 1e-6) // 避免除以零
            {
                x /= w;
                y /= w;
            }

            // 更新边界
            xmin = Math.Min(xmin, x);
            xmax = Math.Max(xmax, x);
            ymin = Math.Min(ymin, y);
            ymax = Math.Max(ymax, y);
        }

        return new BoundingBox(xmin, xmax, ymin, ymax);
    }
}

// 辅助数据结构
public struct PointF
{
    public float X { get; }
    public float Y { get; }

    public PointF(float x, float y)
    {
        X = x;
        Y = y;
    }
}

public struct BoundingBox
{
    public float XMin { get; }
    public float XMax { get; }
    public float YMin { get; }
    public float YMax { get; }

    public BoundingBox(float xmin, float xmax, float ymin, float ymax)
    {
        XMin = xmin;
        XMax = xmax;
        YMin = ymin;
        YMax = ymax;
    }

    public override string ToString() => 
        $"X:[{XMin}, {XMax}], Y:[{YMin}, {YMax}]";
}
```

---

### **使用示例**
```csharp
static void Main()
{
    // 原始点集
    var points = new List<PointF>
    {
        new PointF(0, 0),
        new PointF(1, 0),
        new PointF(1, 1),
        new PointF(0, 1)
    };

    // 变换矩阵示例：平移(2,3) + 缩放(0.5倍)
    float[,] transform = new float[,]
    {
        { 0.5f, 0,    2 },
        { 0,    0.5f, 3 },
        { 0,    0,    1 }
    };

    var bbox = BoundingBoxCalculator.GetTransformedBoundingBox(points, transform);
    Console.WriteLine($"变换后的BoundingBox: {bbox}");
}
```

**输出**：
```
变换后的BoundingBox: X:[2, 2.5], Y:[3, 3.5]
```

---

### **关键逻辑说明**
1. **变换矩阵应用**：
   - 对每个点 `(x,y)` 应用3x3变换矩阵（齐次坐标）：
     ```
     x' = m00*x + m01*y + m02
     y' = m10*x + m11*y + m12
     w  = m20*x + m21*y + m22
     ```
   - 执行透视除法：`x' /= w`, `y' /= w`（如果是仿射变换，`w=1`）

2. **边界计算**：
   - 遍历所有变换后的点，记录最小/最大的X和Y值。

3. **支持变换类型**：
   - 平移、旋转、缩放、剪切等仿射变换。
   - 投影变换（需确保`w≠0`）。

---

### **边界情况测试**
| 场景                  | 输入点集          | 变换矩阵               | 输出BoundingBox       |
|-----------------------|------------------|-----------------------|-----------------------|
| 单位矩形平移           | `[(0,0), (1,1)]` | 平移 `(2,3)`          | `X:[2,3], Y:[3,4]`    |
| 缩放                  | `[(0,0), (1,1)]` | 缩放 `0.5`倍          | `X:[0,0.5], Y:[0,0.5]`|
| 旋转变换              | `[(0,0), (1,1)]` | 旋转90度              | `X:[-1,0], Y:[0,1]`   |
| 空点集                | `[]`             | 任意矩阵              | 抛出异常              |

---

### **性能优化建议**
- 如果点集非常大，可以改用并行计算（如 `Parallel.For`）。
- 对于纯仿射变换（`m20=m21=0, m22=1`），可以跳过透视除法。

此代码可直接集成到您的项目中，处理任意2D坐标变换后的边界框计算。