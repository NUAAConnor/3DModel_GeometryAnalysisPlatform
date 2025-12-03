import sys
import os

# ==============================================================================
# 1. 配置 DLL/模块 搜索路径
# ==============================================================================

# 您的 .pyd 文件所在的【文件夹】路径
pyd_dir = r"F:\2025-Now_NUAA_PhD\Storage\ProjectCodeStore\OCCTProjectWorkspace\3DModel_GeometryAnalysisPlatform\cmake-build-release\modules\ASGBuilder"

if not os.path.exists(pyd_dir):
    print(f"[Error] 找不到编译输出目录: {pyd_dir}")
    sys.exit(1)
sys.path.append(pyd_dir)

# ------------------------------------------------------------------------------
# 关键修复：添加 OpenCASCADE 及其依赖的第三方库 (3rdparty) 的路径
# ------------------------------------------------------------------------------

# 1. 定义依赖根目录 (根据你的项目结构推断)
# 你的 OCCT 路径是: .../dependencies/occt-install-release/...
# 所以 dependencies 根目录在两层之上
dependencies_root = r"F:\2025-Now_NUAA_PhD\Storage\ProjectCodeStore\OCCTProjectWorkspace\dependencies"

# 2. 定义所有需要添加的 DLL 目录列表
dll_paths = [
    # OpenCASCADE 核心 DLL
    os.path.join(dependencies_root, r"occt-install-release\win64\vc14\bin"),

    # 第三方库 DLL (这些是 OCCT 运行必须的，参考你的 CMakeLists.txt)
    os.path.join(dependencies_root, r"3rdparty\freetype-2.13.3-x64\bin"),
    os.path.join(dependencies_root, r"3rdparty\freeimage-3.18.0-x64\bin"),
    os.path.join(dependencies_root, r"3rdparty\tcltk-8.6.15-x64\bin")
    # 如果有 zlib 或其他库，也需要加在这里
]

# 3. 循环添加路径
print("[Info] 正在添加 DLL 搜索路径:")
for path in dll_paths:
    if os.path.exists(path):
        try:
            # Python 3.8+ Windows 必须使用 add_dll_directory
            os.add_dll_directory(path)
            print(f"  [OK] {path}")
        except Exception as e:
            print(f"  [Failed] add_dll_directory 失败: {path}, 错误: {e}")

        # 为了兼容性，同时也加到环境变量 PATH 中
        os.environ['PATH'] = path + os.pathsep + os.environ['PATH']
    else:
        print(f"  [Warning] 路径不存在，跳过: {path}")
# ==============================================================================
# 2. 导入模块
# ==============================================================================
try:
    # 模块名必须与 PYBIND11_MODULE(ASGCore, ...) 中的名字一致
    import ASGCore

    print(f"[Success] 成功导入模块: ASGCore")
    print(f"模块文档: {ASGCore.__doc__}")
except ImportError as e:
    print(f"[Error] 无法导入 ASGCore: {e}")
    print(f"请确认文件夹 {pyd_dir} 下确实存在名为 ASGCore.cp311-win_amd64.pyd 的文件")
    sys.exit(1)


# ==============================================================================
# 3. 测试功能流程
# ==============================================================================
def run_test_pipeline():
    print("\n--- 1. 初始化构建器 ---")
    builder = ASGCore.ASGBuilder()

    # 替换为您的真实 STEP 文件路径
    step_file_path = r"F:\2025-Now_NUAA_PhD\Storage\ProjectCodeStore\OCCTProjectWorkspace\TestModels\The Baseline_Ass.stp"

    if not os.path.exists(step_file_path):
        print(f"[Error] STEP 文件不存在: {step_file_path}")
        return

    print(f"正在加载文件: {step_file_path}")
    if builder.load_step_file(step_file_path):
        print("[Success] STEP 文件加载成功。")

        # 执行几何分析管线
        print("\n--- 2. 执行几何分析 ---")
        print("正在解析装配树...")
        builder.parse_assembly()
        print("正在分类原子特征...")
        builder.classify_features()
        print("正在识别复合特征...")
        builder.recognize_composites()
        print("正在构建约束图...")
        builder.build_constraint_graph()

        # 导出 JSON (可选)
        json_output = r"output_test.json"
        builder.export_json(json_output)
        print(f"结果已导出至: {json_output}")

        # 测试 GNN 数据导出
        print("\n--- 3. 测试 GNN 数据提取 (GetGraphData) ---")
        # 注意：这里的 ID 必须是 C++ 解析出来的真实 ID (例如 "HolePlane" 或 "Part_1")
        # 如果不确定，看上面 run_test() 运行时控制台打印的 "|- Part Found: XXXXX"
        target_part_id = "HolePlane"

        print(f"尝试提取零件 '{target_part_id}' 的图数据...")
        graph_data = builder.get_graph_data(target_part_id)

        if not graph_data["node_types"]:
            print(f"[Warning] 数据为空。请确认 PartID '{target_part_id}' 是否正确（区分大小写）。")
        else:
            print("[Success] 成功获取图数据！")
            num_nodes = len(graph_data['node_types'])
            num_edges = len(graph_data['edge_source'])
            print(f"  > 节点数量: {num_nodes}")
            print(f"  > 边数量:   {num_edges}")

            # 打印前 3 个节点信息作为采样
            print("  > 节点数据采样 (Top 3):")
            for i in range(min(3, num_nodes)):
                n_type = graph_data['node_types'][i]
                n_area = graph_data['node_areas'][i]
                n_curv = graph_data['node_curvatures'][i]
                lbl_type = graph_data['node_label_types'][i]
                print(f"    [{i}] Type={n_type}, Area={n_area:.2f}, Curv={n_curv:.2f}, LabelType={lbl_type}")

        # 测试约束导出
        print("\n--- 4. 测试装配约束导出 (Knowledge Graph) ---")
        constraints = builder.get_constraints()
        print(f"检测到装配约束数量: {len(constraints)}")
        for i, c in enumerate(constraints):
            # c 是 AssemblyConstraint 对象，会自动调用 C++ 的 ToString()
            print(f"  [{i}] {c}")


if __name__ == "__main__":
    run_test_pipeline()
