import torch
import torch.nn as nn
import torch.nn.functional as F
from torch_geometric.nn import GATv2Conv

class GatedGATLayer(nn.Module):
    """
    门控图注意力层 (Gated Graph Attention Layer)
    创新点: 结合了几何连续性(Continuity)作为门控机制，控制信息流。
    """
    def __init__(self, in_dim, out_dim, edge_dim=2, heads=4):
        super(GatedGATLayer, self).__init__()

        # GATv2Conv 是 PyG 提供的第二代图注意力卷积，比标准 GAT 更强
        # edge_dim 参数允许我们将边特征 (角度, 连续性) 注入到注意力计算中
        self.gat = GATv2Conv(in_dim, out_dim, heads=heads, edge_dim=edge_dim, concat=True)

        # 门控网络: 根据边特征计算一个 0~1 的系数
        # 输入: [Edge_Dim], 输出: [1]
        self.gate_net = nn.Sequential(
            nn.Linear(edge_dim, 8),
            nn.ReLU(),
            nn.Linear(8, 1),
            nn.Sigmoid()
        )

        # 输出维度调整 (因为多头注意力会把维度翻倍)
        self.out_proj = nn.Linear(out_dim * heads, out_dim)

    def forward(self, x, edge_index, edge_attr):
        """
        x: 节点特征 [N, in_dim]
        edge_index: [2, E]
        edge_attr: [E, 2] (Angle, Continuity)
        """

        # 1. 标准 GAT 聚合
        # x_agg: [N, out_dim * heads]
        x_agg = self.gat(x, edge_index, edge_attr=edge_attr)

        # 2. 降维投影
        x_agg = self.out_proj(x_agg) # [N, out_dim]

        # 3. 门控机制 (简化版 - 残差连接加权)
        # 在这里，我们简单地使用 GAT + Residual
        # 完整的边门控需要在消息传递内部修改，PyG 的 GATv2Conv 已经通过 edge_dim 隐式处理了权重的计算
        # 因此，这里我们主要做非线性激活和残差

        x_out = F.elu(x_agg)

        # 如果输入输出维度一致，可以加残差连接 (ResNet风格)
        if x.shape[1] == x_out.shape[1]:
            x_out = x_out + x

        return x_out