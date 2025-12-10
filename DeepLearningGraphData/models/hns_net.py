import torch
import torch.nn as nn
import torch.nn.functional as F
import os
import sys

# ==============================================================================
# 导入路径兼容性修复
# ==============================================================================
try:
    # 场景1：作为包被外部导入 (from .models.hns_net import HNSNet)
    from .encoders import UVEncoder, SymbolicEncoder
    from .gnn import GatedGATLayer
except ImportError:
    # 场景2：作为脚本直接运行进行测试 (python hns_net.py)
    # 将当前目录加入路径，以便能直接导入同级模块
    current_dir = os.path.dirname(os.path.abspath(__file__))
    sys.path.append(current_dir)

    from encoders import UVEncoder, SymbolicEncoder
    from gnn import GatedGATLayer

class HNSNet(nn.Module):
    """
    [Phase I 最终模型] 神经-符号混合网络 (Hybrid Neuro-Symbolic Network)
    流程:
    1. UVEncoder 提取视觉特征 (V_vis)
    2. SymbolicEncoder 提取符号特征 (V_sym)
    3. 融合特征 V_fuse = Concat(V_vis, V_sym)
    4. GatedGAT 进行拓扑信息传播
    5. MLP 输出分类 logits
    """
    def __init__(self, num_classes):
        super(HNSNet, self).__init__()

        # 1. 编码器
        # 视觉流: [Batch, 6, 64, 64] -> [Batch, 128]
        self.uv_encoder = UVEncoder(in_channels=6, out_dim=128)

        # 符号流: [Batch, 3] -> [Batch, 64]
        # (Type, Area, Curvature)
        self.sym_encoder = SymbolicEncoder(in_dim=3, out_dim=64)

        # 融合后的特征维度
        fusion_dim = 128 + 64 # 192

        # 2. 图推理层 (堆叠 2 层 GAT)
        # GNN Layer 1: [192] -> [128]
        self.gnn1 = GatedGATLayer(fusion_dim, 128, edge_dim=2, heads=4)

        # GNN Layer 2: [128] -> [128]
        self.gnn2 = GatedGATLayer(128, 128, edge_dim=2, heads=4)

        # 3. 分类头 (Classifier)
        # [128] -> [num_classes]
        self.classifier = nn.Sequential(
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Dropout(0.5),
            nn.Linear(64, num_classes)
        )

    def forward(self, data):
        """
        data: PyG Data Batch 对象
        包含: x_vis, x_sym, edge_index, edge_attr
        """
        # --- A. 双流编码 ---
        v_vis = self.uv_encoder(data.x_vis)
        v_sym = self.sym_encoder(data.x_sym)

        # --- B. 特征融合 ---
        # [N, 128] + [N, 64] -> [N, 192]
        x = torch.cat([v_vis, v_sym], dim=1)

        # --- C. 图消息传递 ---
        # 第一层 GAT
        x = self.gnn1(x, data.edge_index, data.edge_attr)

        # 第二层 GAT
        x = self.gnn2(x, data.edge_index, data.edge_attr)

        # --- D. 分类预测 ---
        logits = self.classifier(x)

        return logits

# ==============================================================================
# 单元测试
# ==============================================================================
if __name__ == "__main__":
    print("-" * 60)
    print("正在测试 HNSNet 模型组装...")
    from torch_geometric.data import Data

    # 构造模拟数据
    num_nodes = 5
    num_classes = 4

    model = HNSNet(num_classes=num_classes)
    print(f"模型初始化成功: {model}")

    # 模拟输入数据
    # x_vis: [5节点, 6通道, 64宽, 64高]
    # x_sym: [5节点, 3特征]
    data = Data(
        x_vis = torch.randn(num_nodes, 6, 64, 64),
        x_sym = torch.randn(num_nodes, 3),
        edge_index = torch.tensor([[0, 1, 1, 2], [1, 0, 2, 1]], dtype=torch.long),
        edge_attr = torch.randn(4, 2) # 4条边，每条边2个特征
    )

    print("-" * 60)
    print("正在执行前向传播 (Forward Pass)...")
    try:
        out = model(data)
        print(f"[Pass] 模型输出形状: {out.shape} \t(预期: [{num_nodes}, {num_classes}])")
    except Exception as e:
        print(f"[Fail] 前向传播失败: {e}")
        import traceback
        traceback.print_exc()