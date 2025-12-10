import os
import torch
import numpy as np
from torch_geometric.data import Data, Dataset
from typing import List

# ==============================================================================
# 模块导入逻辑修正
# ==============================================================================
try:
    # 场景1：作为包的一部分被导入 (from DeepLearningGraphData import ...)
    from . import ASGBuilder
except ImportError:
    # 场景2：作为脚本直接运行 (python dataset.py)
    # 需要手动把父目录加入路径，才能识别 DeepLearningGraphData 包
    import sys
    current_dir = os.path.dirname(os.path.abspath(__file__))
    parent_dir = os.path.dirname(current_dir)
    if parent_dir not in sys.path:
        sys.path.append(parent_dir)

    try:
        from DeepLearningGraphData import ASGBuilder
    except ImportError as e:
        raise ImportError(f"无法加载 ASGBuilder。请确保编译好的 .pyd 文件在正确位置。\n错误详情: {e}")

class AircraftPartDataset(Dataset):
    """
    飞机装配零件数据集 (Aircraft Assembly Part Dataset)
    """
    def __init__(self, step_files: List[str], transform=None, pre_transform=None):
        super().__init__(None, transform, pre_transform)
        # 过滤不存在的文件
        self.step_files = [f for f in step_files if os.path.exists(f)]

        if not self.step_files:
            print("[Warning] 初始化数据集失败：没有找到有效的 STEP 文件。")

        # [Fix] ASGBuilder 导入进来就是类，直接实例化
        self.builder = ASGBuilder()

    def len(self):
        return len(self.step_files)

    def get(self, idx):
        step_path = self.step_files[idx]

        # 1. C++ 加载与解析
        # 注意：load_step_file 返回 bool
        if not self.builder.load_step_file(step_path):
            print(f"[Error] 无法加载文件: {step_path}")
            return Data()

        self.builder.parse_assembly()
        self.builder.classify_features()

        # 获取第一个零件
        all_parts = self.builder.get_all_part_ids()
        if not all_parts:
            return Data()

        target_part_id = all_parts[0]

        # 2. 获取符号图数据
        graph_data = self.builder.get_graph_data(target_part_id)

        if not graph_data or not graph_data["node_types"]:
            return Data()

        # --- 节点特征 (Symbolic) ---
        # [N, 3]: Type, Area, Curvature
        x_sym = torch.stack([
            torch.tensor(graph_data["node_types"], dtype=torch.float),
            torch.tensor(graph_data["node_areas"], dtype=torch.float),
            torch.tensor(graph_data["node_curvatures"], dtype=torch.float)
        ], dim=1)

        # --- 边索引 (Adjacency) ---
        edge_index = torch.tensor([
            graph_data["edge_source"],
            graph_data["edge_target"]
        ], dtype=torch.long)

        # --- 边属性 (Edge Attributes) ---
        # [E, 2]: Angle, Continuity
        edge_attr = torch.stack([
            torch.tensor(graph_data["edge_angles"], dtype=torch.float),
            torch.tensor(graph_data["edge_continuity"], dtype=torch.float)
        ], dim=1)

        # 3. 获取视觉特征 (UV Grids)
        uv_grids = self.builder.get_part_uv_grids(target_part_id)

        # --- 图像张量 (Visual) ---
        tensor_list = []
        for grid in uv_grids:
            flat_data = np.array(grid.data, dtype=np.float32)
            resolution = grid.resolution
            channels = grid.channels

            # Reshape: [H, W, C] -> Transpose -> [C, H, W]
            img_hwc = flat_data.reshape(resolution, resolution, channels)
            img_chw = img_hwc.transpose(2, 0, 1)

            tensor_list.append(torch.from_numpy(img_chw))

        if tensor_list:
            x_vis = torch.stack(tensor_list) # [N, 6, 64, 64]
        else:
            x_vis = torch.empty((0, 6, 64, 64))

        # 4. 封装 Data 对象
        data = Data(
            x_sym=x_sym,
            x_vis=x_vis,
            edge_index=edge_index,
            edge_attr=edge_attr,
            part_id=target_part_id
        )

        return data

# ==============================================================================
# 单元测试
# ==============================================================================
if __name__ == "__main__":
    print("-" * 60)
    print("正在测试 AircraftPartDataset...")

    # 请确保路径正确
    TEST_FILE = r"F:/2025-Now_NUAA_PhD/Storage/ProjectCodeStore/OCCTProjectWorkspace/TestModels/The Baseline_Ass.stp"

    if not os.path.exists(TEST_FILE):
        print(f"[Error] 测试文件不存在: {TEST_FILE}")
        exit(1)

    dataset = AircraftPartDataset([TEST_FILE])
    print(f"数据集长度: {len(dataset)}")

    try:
        data = dataset.get(0)
        print("-" * 60)
        print(f"样本 Part ID: {data.part_id}")
        print(f"[Check 1] 符号特征 x_sym:  {data.x_sym.shape} \t(预期: [N, 3])")
        print(f"[Check 2] 视觉特征 x_vis:  {data.x_vis.shape} \t(预期: [N, 6, 64, 64])")
        print(f"[Check 3] 边索引 edge_index: {data.edge_index.shape} \t(预期: [2, E])")
        print("-" * 60)
        print("[Success] 数据管道验证通过！")
    except Exception as e:
        print(f"[Fail] 数据获取失败: {e}")
        import traceback
        traceback.print_exc()