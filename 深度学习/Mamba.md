好的！我来更详细地拆解第4步的推导过程，尤其是积分部分的变换。为了清晰，我们从头开始逐步展开。

---

### **详细推导：零阶保持（ZOH）离散化**

#### **1. 连续时间系统方程**
连续系统的状态方程：

$$
\frac{dx(t)}{dt} = A x(t) + B u(t)
$$

其解析解为（通过积分因子法）：

$$
x(t) = e^{A(t - t_0)} x(t_0) + \int_{t_0}^t e^{A(t - \tau)} B u(\tau) d\tau
$$


#### **2. 离散时间点的状态更新**
在离散时间点 $t = k\Delta$ 和 $t = (k+1)\Delta$ 之间（即 $t \in [k\Delta, (k+1)\Delta)$），利用零阶保持假设：

$$
u(\tau) = u_k \quad \text{（常量）}, \quad \tau \in [k\Delta, (k+1)\Delta)
$$

因此，状态更新公式变为：

$$
x((k+1)\Delta) = e^{A \Delta} x(k\Delta) + \left( \int_{k\Delta}^{(k+1)\Delta} e^{A((k+1)\Delta - \tau)} d\tau \right) B u_k
$$


#### **3. 积分变量替换**
令 $\tau' = (k+1)\Delta - \tau$，则：
- 当 $\tau = k\Delta$ 时，$\tau' = \Delta$；
- 当 $\tau = (k+1)\Delta$ 时，$\tau' = 0$；
- 微分关系：$d\tau = -d\tau'$。

积分限变换后：

$$
\int_{k\Delta}^{(k+1)\Delta} e^{A((k+1)\Delta - \tau)} d\tau = \int_{\Delta}^{0} e^{A \tau'} (-d\tau') = \int_{0}^{\Delta} e^{A \tau'} d\tau'
$$


#### **4. 离散化矩阵的表达式**
因此，离散化后的系统方程为：

$$
x_{k+1} = \underbrace{e^{A \Delta}}_{\bar{A}} x_k + \underbrace{\left( \int_{0}^{\Delta} e^{A \tau} d\tau \right) B}_{\bar{B}} u_k
$$


#### **5. 计算积分 $\int_{0}^{\Delta} e^{A \tau} d\tau$**
若 $A$ 可逆，可通过矩阵指数性质求解：

$$
\int_{0}^{\Delta} e^{A \tau} d\tau = A^{-1} \left( e^{A \Delta} - I \right)
$$

**推导过程**：
- 对矩阵指数 $e^{A \tau}$ 积分，逐项积分其泰勒级数：
  
$$
e^{A \tau} = \sum_{k=0}^{\infty} \frac{(A \tau)^k}{k!}
$$

  
$$
\int_{0}^{\Delta} e^{A \tau} d\tau = \sum_{k=0}^{\infty} \frac{A^k}{k!} \int_{0}^{\Delta} \tau^k d\tau = \sum_{k=0}^{\infty} \frac{A^k \Delta^{k+1}}{(k+1)!}
$$

- 提取 $A^{-1}$（假设 $A$ 可逆）：
  
$$
\sum_{k=0}^{\infty} \frac{A^k \Delta^{k+1}}{(k+1)!} = A^{-1} \sum_{k=0}^{\infty} \frac{(A \Delta)^{k+1}}{(k+1)!} = A^{-1} \left( e^{A \Delta} - I \right)
$$


若 $A$ 不可逆，需用其他方法（如特征分解或数值积分）。

#### **6. 最终离散化公式**

$$
\bar{A} = e^{A \Delta}, \quad \bar{B} = A^{-1} \left( e^{A \Delta} - I \right) B
$$


---

### **例子验证（具体数值）**
设：

$$
A = \begin{bmatrix} -1 & 0 \\ 0 & -2 \end{bmatrix}, \quad B = \begin{bmatrix} 1 \\ 1 \end{bmatrix}, \quad \Delta = 0.1
$$


#### **计算 $\bar{A} = e^{A \Delta}$**

$$
e^{A \Delta} = \exp \left( \begin{bmatrix} -0.1 & 0 \\ 0 & -0.2 \end{bmatrix} \right) = \begin{bmatrix} e^{-0.1} & 0 \\ 0 & e^{-0.2} \end{bmatrix} \approx \begin{bmatrix} 0.9048 & 0 \\ 0 & 0.8187 \end{bmatrix}
$$


#### **计算 $\bar{B}$**
1. 求 $A^{-1}$：
   
$$
A^{-1} = \begin{bmatrix} -1 & 0 \\ 0 & -0.5 \end{bmatrix}
$$

2. 计算 $e^{A \Delta} - I$：
   
$$
e^{A \Delta} - I = \begin{bmatrix} -0.0952 & 0 \\ 0 & -0.1813 \end{bmatrix}
$$

3. 组合结果：
   
$$
\bar{B} = A^{-1} (e^{A \Delta} - I) B = \begin{bmatrix} -1 & 0 \\ 0 & -0.5 \end{bmatrix} \begin{bmatrix} -0.0952 & 0 \\ 0 & -0.1813 \end{bmatrix} \begin{bmatrix} 1 \\ 1 \end{bmatrix} = \begin{bmatrix} 0.0952 \\ 0.0907 \end{bmatrix}
$$


---

### **为什么积分结果是 $A^{-1}(e^{A \Delta} - I)$？**
直观理解：
- 矩阵指数 $e^{A \tau}$ 的积分类似于标量指数函数的积分 $\int e^{a \tau} d\tau = \frac{1}{a} e^{a \tau} + C$。
- 对矩阵而言，$A^{-1}$ 相当于标量的 $\frac{1}{a}$，但需注意矩阵乘法的顺序（不可交换）。

---

### **关键点总结**
1. **ZOH假设**：输入 $u(t)$ 在离散间隔内恒定，使得积分项中的 $u(\tau)$ 可提到积分外。
2. **变量替换**：通过 $\tau' = (k+1)\Delta - \tau$ 简化积分限。
3. **矩阵指数积分**：利用泰勒级数展开或逆矩阵性质求解。
4. **离散化后的形式**：$\bar{A}$ 是矩阵指数，$\bar{B}$ 是积分与 $B$ 的乘积。

通过这种推导，SSMs 在训练时只需学习连续的 $A$ 和 $B$，推理时再按需离散化，兼顾了灵活性和效率。