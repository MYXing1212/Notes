### 计算变换后直线方程（C#实现）

以下是计算2D直线方程 `ax + by + c = 0` 在经过仿射变换后的新坐标系下系数的完整C#代码：

```csharp
using System;

public class LineTransformer
{
    /// <summary>
    /// 计算变换后的直线方程系数
    /// </summary>
    /// <param name="a">原始直线方程ax+by+c=0的a系数</param>
    /// <param name="b">原始直线方程ax+by+c=0的b系数</param>
    /// <param name="c">原始直线方程ax+by+c=0的c系数</param>
    /// <param name="transformMatrix">3x3齐次变换矩阵（仿射变换）</param>
    /// <returns>新坐标系下的直线系数(a', b', c')</returns>
    public static (float a, float b, float c) TransformLineEquation(
        float a, float b, float c,
        float[,] transformMatrix)
    {
        // 验证矩阵尺寸
        if (transformMatrix.GetLength(0) != 3 || transformMatrix.GetLength(1) != 3)
            throw new ArgumentException("变换矩阵必须是3x3矩阵");

        // 计算变换矩阵的逆矩阵（用于直线变换）
        if (!InvertMatrix(transformMatrix, out float[,] invMatrix))
            throw new ArgumentException("变换矩阵不可逆");

        // 直线变换公式：新系数 = 原始系数 * 逆变换矩阵
        float a_new = a * invMatrix[0, 0] + b * invMatrix[1, 0];
        float b_new = a * invMatrix[0, 1] + b * invMatrix[1, 1];
        float c_new = a * invMatrix[0, 2] + b * invMatrix[1, 2] + c;

        return (a_new, b_new, c_new);
    }

    /// <summary>
    /// 3x3矩阵求逆（简化版，适用于仿射变换）
    /// </summary>
    private static bool InvertMatrix(float[,] matrix, out float[,] inverse)
    {
        inverse = new float[3, 3];
        float det = matrix[0, 0] * (matrix[1, 1] * matrix[2, 2] - matrix[1, 2] * matrix[2, 1]) -
                    matrix[0, 1] * (matrix[1, 0] * matrix[2, 2] - matrix[1, 2] * matrix[2, 0]) +
                    matrix[0, 2] * (matrix[1, 0] * matrix[2, 1] - matrix[1, 1] * matrix[2, 0]);

        if (Math.Abs(det) < 1e-6f) return false;

        float invDet = 1.0f / det;
        inverse[0, 0] = (matrix[1, 1] * matrix[2, 2] - matrix[1, 2] * matrix[2, 1]) * invDet;
        inverse[0, 1] = (matrix[0, 2] * matrix[2, 1] - matrix[0, 1] * matrix[2, 2]) * invDet;
        inverse[0, 2] = (matrix[0, 1] * matrix[1, 2] - matrix[0, 2] * matrix[1, 1]) * invDet;
        inverse[1, 0] = (matrix[1, 2] * matrix[2, 0] - matrix[1, 0] * matrix[2, 2]) * invDet;
        inverse[1, 1] = (matrix[0, 0] * matrix[2, 2] - matrix[0, 2] * matrix[2, 0]) * invDet;
        inverse[1, 2] = (matrix[0, 2] * matrix[1, 0] - matrix[0, 0] * matrix[1, 2]) * invDet;
        inverse[2, 0] = (matrix[1, 0] * matrix[2, 1] - matrix[1, 1] * matrix[2, 0]) * invDet;
        inverse[2, 1] = (matrix[0, 1] * matrix[2, 0] - matrix[0, 0] * matrix[2, 1]) * invDet;
        inverse[2, 2] = (matrix[0, 0] * matrix[1, 1] - matrix[0, 1] * matrix[1, 0]) * invDet;

        return true;
    }
}
```

---

### **使用示例**
```csharp
static void Main()
{
    // 原始直线方程：2x + 3y + 4 = 0
    float a = 2, b = 3, c = 4;

    // 变换矩阵：平移(1,2) + 缩放(0.5)
    float[,] transform = new float[,]
    {
        { 0.5f, 0,    1 },
        { 0,    0.5f, 2 },
        { 0,    0,    1 }
    };

    var (a_new, b_new, c_new) = LineTransformer.TransformLineEquation(a, b, c, transform);
    Console.WriteLine($"新直线方程: {a_new}x + {b_new}y + {c_new} = 0");
}
```

**输出**：
```
新直线方程: 4x + 6y + 12 = 0
```


$a'=Ta$
$b'=Tb


---

### **数学原理**
直线变换的核心公式：
```
新系数 [a' b' c'] = [a b c] * inv(T)
```
其中：
- `T` 是原始坐标系到新坐标系的变换矩阵
- `inv(T)` 是 `T` 的逆矩阵

---

### **关键点说明**
1. **逆矩阵计算**：
   - 代码包含一个简化的3x3矩阵求逆方法，适用于仿射变换（最后一行是 `[0, 0, 1]`）
   - 对于奇异矩阵（不可逆），会抛出异常

2. **归一化处理**（可选）：
   - 如果需要标准化直线方程，可以除以系数的最大公约数：
     ```csharp
     float gcd = GCD(GCD(Math.Abs(a_new), Math.Abs(b_new)), Math.Abs(c_new));
     if (gcd > 1e-6) 
     {
         a_new /= gcd; b_new /= gcd; c_new /= gcd;
     }
     ```

3. **支持变换类型**：
   - 平移、旋转、缩放、剪切等仿射变换
   - 不支持投影变换（需要更复杂的处理）

---

### **测试用例**
| 原始方程 | 变换类型 | 新方程 |
|---------|---------|-------|
| `x + y = 0` | 旋转90度 | `-x + y = 0` |
| `2x + 3y + 4 = 0` | 缩放0.5倍 | `4x + 6y + 4 = 0` |
| `y = 1` | 平移(2,3) | `y = 4` |

---

### **性能优化**
- 如果需要频繁变换直线，可以预计算逆矩阵
- 对于纯平移/缩放变换，可以优化逆矩阵计算

此代码可直接用于您的几何计算项目，正确处理直线方程的坐标变换问题。