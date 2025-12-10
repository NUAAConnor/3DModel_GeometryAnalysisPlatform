import torch
from dataset import AircraftPartDataset
from models.hns_net import HNSNet

# 1. 准备环境
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
num_classes = 5  # 必须和训练时一致

# 2. 实例化一个“空”模型（只有架构，没有脑子）
model = HNSNet(num_classes=num_classes).to(device)

# 3. 加载“大脑” (.pth 文件)
# 假设我们加载第 50 轮的模型
checkpoint_path = "checkpoints/hns_net_epoch_50.pth"
model.load_state_dict(torch.load(checkpoint_path))
model.eval()  # 切换到评估模式 (关闭 Dropout 等)

print("模型加载成功！准备推理...")

# 4. 加载数据并预测
test_file = r"F:/2025-Now_NUAA_PhD/Storage/ProjectCodeStore/OCCTProjectWorkspace/TestModels/The Baseline_Ass.stp"
dataset = AircraftPartDataset([test_file])
data = dataset.get(0).to(device)

with torch.no_grad():
    # 前向传播
    logits = model(data)
    # 获取概率最大的类别
    predictions = torch.argmax(logits, dim=1)

print(f"预测结果 (每个面的类别): {predictions}")