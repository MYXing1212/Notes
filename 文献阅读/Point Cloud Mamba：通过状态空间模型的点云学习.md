原文
    最近，状态空间模型展现出了强大的全局建模能力和线性计算复杂度，与==Transformers==形成对
比。本研究着重于将此类架构应用于更高效且有效地以全局方式对点云数据进行建模，并实现线性计算复杂度。特别是，我们首次证明了基于 Mamba 的点云方法能够超越基于transform或多层感知机的先进方法。为了使 Mamba 能够更有效地处理 3D 点云数据，==我们提出了一种新颖的 Consistent Traverse Serialization（一致性遍历序列化）方法==，将点云转换为 1D 点序列，同时确保序列中相邻的点在空间上也是相邻的。通过置换 x、y 和 z 坐标的顺序，Consistent Traverse Serialization 产生了六种变体，这些变体的协同使用有助于 Mamba 全面地观察点云数据。此外，为了帮助 Mamba 更有效地处理不同顺序的点序列，我们引入了点提示，告知 Mamba 序列的排列规则。最后，我们提出了基于空间坐标映射的位置编码，以更有效地将位置信息注入点云序列中。点云 Mamba 超越了基于点的最先进方法 PointNeXt，并在 ScanObjectNN、ModelNet40、ShapeNetPart 和 S3DIS 数据集上取得了新的最先进性能。==值得强调的是，当使用更强大的局部特征提取模块时，我们的 PCM 在 S3DIS 上实现了 79.6 mIoU，分别以 5.5 mIoU 和 4.9 mIoU 的优势显著超越了先前的最先进模型 DeLA 和 PTv3==。

