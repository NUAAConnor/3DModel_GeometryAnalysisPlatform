import torch
import torch.nn as nn
import torch.nn.functional as F

class UVEncoder(nn.Module):
    """
    [视觉流] UV-Grid 特征提取器
    输入: [Batch, 6, 64, 64] (6 Channels: Nx, Ny, Nz, K, H, Mask)
    输出: [Batch, 128] (视觉特征向量 V_vis)
    架构: 定制化 CNN，保留高分辨率特征以识别微小倒角和孔洞。
    """
    def __init__(self, in_channels=6, out_dim=128):
        super(UVEncoder, self).__init__()

        # Layer 1: 捕捉微观几何纹理 (64x64 -> 32x32)
        self.conv1 = nn.Conv2d(in_channels, 32, kernel_size=3, stride=1, padding=1)
        self.bn1 = nn.BatchNorm2d(32)
        # 注意：这里我们不立即做 MaxPool，而是用 Stride=2 的卷积在下一层降维，
        # 或者仅仅依靠后续层，为了保留像“倒角”这样的单像素级特征。

        # Layer 2: (64x64 -> 32x32)
        self.conv2 = nn.Conv2d(32, 64, kernel_size=3, stride=2, padding=1)
        self.bn2 = nn.BatchNorm2d(64)

        # Layer 3: (32x32 -> 16x16)
        self.conv3 = nn.Conv2d(64, 128, kernel_size=3, stride=2, padding=1)
        self.bn3 = nn.BatchNorm2d(128)

        # Layer 4: (16x16 -> 8x8)
        self.conv4 = nn.Conv2d(128, 256, kernel_size=3, stride=2, padding=1)
        self.bn4 = nn.BatchNorm2d(256)

        # Global Average Pooling (GAP) -> [Batch, 256, 1, 1]
        self.gap = nn.AdaptiveAvgPool2d(1)

        # Projection Head -> [Batch, 128]
        self.fc = nn.Linear(256, out_dim)

    def forward(self, x):
        # x: [Batch, 6, 64, 64]

        x = F.relu(self.bn1(self.conv1(x)))
        x = F.relu(self.bn2(self.conv2(x)))
        x = F.relu(self.bn3(self.conv3(x)))
        x = F.relu(self.bn4(self.conv4(x)))

        x = self.gap(x)
        x = x.view(x.size(0), -1) # Flatten
        x = self.fc(x)

        return x # V_vis

class SymbolicEncoder(nn.Module):
    """
    [符号流] 显式工程特征编码器
    输入: [Batch, 3] (Type, Area, Curvature)
    输出: [Batch, 64] (符号特征向量 V_sym)
    """
    def __init__(self, in_dim=3, out_dim=64):
        super(SymbolicEncoder, self).__init__()

        self.net = nn.Sequential(
            nn.Linear(in_dim, 64),
            nn.BatchNorm1d(64),
            nn.ReLU(),

            nn.Linear(64, 128),
            nn.BatchNorm1d(128),
            nn.ReLU(),

            nn.Linear(128, out_dim),
            nn.BatchNorm1d(out_dim),
            nn.ReLU()
        )

    def forward(self, x):
        # x: [Batch, 3]
        return self.net(x)

if __name__ == "__main__":
    # 简单的维度测试
    print("Testing Encoders...")
    batch_size = 10

    # Test UVEncoder
    uv_input = torch.randn(batch_size, 6, 64, 64)
    uv_model = UVEncoder()
    uv_out = uv_model(uv_input)
    print(f"UVEncoder Output: {uv_out.shape} (Expected: [{batch_size}, 128])")

    # Test SymbolicEncoder
    sym_input = torch.randn(batch_size, 3)
    sym_model = SymbolicEncoder()
    sym_out = sym_model(sym_input)
    print(f"SymbolicEncoder Output: {sym_out.shape} (Expected: [{batch_size}, 64])")