import os
from datetime import datetime

import torch
import torch.nn as nn
import torch.optim as optim
from torch_geometric.loader import DataLoader

# 导入我们之前写的模块
from dataset import AircraftPartDataset
from models.hns_net import HNSNet

# ==============================================================================
# 配置参数 (Hyperparameters)
# ==============================================================================
CONFIG = {
    # 路径配置
    "train_files": [
        r"F:/2025-Now_NUAA_PhD/Storage/ProjectCodeStore/OCCTProjectWorkspace/TestModels/The Baseline_Ass.stp",
        # 后面Sprint 2生成数据后，这里可以是一个文件夹下的所有.stp文件
    ],
    "save_dir": "checkpoints",

    # 训练参数
    "num_epochs": 50,
    "batch_size": 4,       # 图数据的Batch Size (多少个零件一起训练)
    "learning_rate": 0.001,
    "num_classes": 5,      # 假设有5类语义 (如: 0:背景, 1:基准孔, 2:铆钉孔, 3:配合面, 4:倒角)
    "device": "cuda" if torch.cuda.is_available() else "cpu"
}

# ==============================================================================
# 损失函数: Focal Loss
#
# 解决航空装配中“关键特征少，普通面多”的类别不平衡问题
# ==============================================================================
class FocalLoss(nn.Module):
    def __init__(self, alpha=1, gamma=2, reduction='mean'):
        super(FocalLoss, self).__init__()
        self.alpha = alpha
        self.gamma = gamma
        self.reduction = reduction
        self.ce_loss = nn.CrossEntropyLoss(reduction='none')

    def forward(self, inputs, targets):
        log_pt = -self.ce_loss(inputs, targets)
        pt = torch.exp(log_pt)
        loss = self.alpha * (1 - pt) ** self.gamma * self.ce_loss(inputs, targets)

        if self.reduction == 'mean':
            return loss.mean()
        elif self.reduction == 'sum':
            return loss.sum()
        return loss

# ==============================================================================
# 训练主循环
# ==============================================================================
def train():
    print(f"[{datetime.now()}] 初始化训练环境...")
    print(f" -> Device: {CONFIG['device']}")

    # 1. 准备数据
    dataset = AircraftPartDataset(CONFIG['train_files'])

    # 这里的 DataLoader 是 PyG 专用的，它会自动把多个图拼接成一个大图 (Batch)
    loader = DataLoader(dataset, batch_size=CONFIG['batch_size'], shuffle=True)

    print(f" -> 数据集加载完成，共 {len(dataset)} 个样本。")

    # 2. 初始化模型
    model = HNSNet(num_classes=CONFIG['num_classes']).to(CONFIG['device'])
    optimizer = optim.Adam(model.parameters(), lr=CONFIG['learning_rate'])
    criterion = FocalLoss().to(CONFIG['device'])

    # 创建保存目录
    if not os.path.exists(CONFIG['save_dir']):
        os.makedirs(CONFIG['save_dir'])

    # 3. 开始循环
    print(f"[{datetime.now()}] 开始训练 (Epochs: {CONFIG['num_epochs']})...")
    model.train()

    for epoch in range(CONFIG['num_epochs']):
        total_loss = 0
        step_count = 0

        for batch in loader:
            batch = batch.to(CONFIG['device'])

            # -------------------------------------------------------
            # [临时逻辑] 生成伪标签 (Dummy Labels) 用于代码跑通验证
            # 真实训练时，ASGBuilder 会返回 batch.y
            # -------------------------------------------------------
            if not hasattr(batch, 'y') or batch.y is None:
                # 随机生成标签: [Total_Nodes_In_Batch]
                num_nodes = batch.x_sym.shape[0]
                batch.y = torch.randint(0, CONFIG['num_classes'], (num_nodes,)).to(CONFIG['device'])
            # -------------------------------------------------------

            optimizer.zero_grad()

            # 前向传播
            outputs = model(batch) # [Total_Nodes, Num_Classes]

            # 计算损失
            loss = criterion(outputs, batch.y)

            # 反向传播
            loss.backward()
            optimizer.step()

            total_loss += loss.item()
            step_count += 1

        avg_loss = total_loss / step_count if step_count > 0 else 0

        # 打印日志 (每10轮)
        if (epoch + 1) % 10 == 0:
            print(f"Epoch [{epoch+1}/{CONFIG['num_epochs']}] \t Loss: {avg_loss:.4f}")

            # 保存检查点
            checkpoint_path = os.path.join(CONFIG['save_dir'], f"hns_net_epoch_{epoch+1}.pth")
            torch.save(model.state_dict(), checkpoint_path)

    print(f"[{datetime.now()}] 训练完成！模型已保存至 {CONFIG['save_dir']}")

if __name__ == "__main__":
    # 简单的异常捕获，防止崩了看不到报错
    try:
        train()
    except Exception as e:
        print(f"[Fatal Error] 训练中断: {e}")
        import traceback
        traceback.print_exc()