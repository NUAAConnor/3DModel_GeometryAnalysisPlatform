import sys
from pathlib import Path

# 将项目根目录加入 sys.path，以便能找到 'ASG' 包
# (如果以后将 ASG 作为一个标准 pip 包安装，连这一步都不需要)
current_dir = Path(__file__).parent
project_root = current_dir.parent
sys.path.append(str(project_root))

# ==========================================
# 现在的导入变得非常简洁
# ==========================================
try:
    # 直接导入包，__init__.py 会自动处理 DLL 和路径
    from DeepLearningGraphData import ASGBuilder
except ImportError as e:
    print(f"初始化失败: {e}")
    sys.exit(1)

def run_test_pipeline():
    print("--- 初始化构建器 ---")
    builder = ASGBuilder() # 直接使用

    # ... 后续逻辑不变 ...
    # 这里的路径也可以改为相对路径，进一步增强可移植性
    step_file_relative = project_root / ".." / "TestModels" / "The Baseline_Ass.stp"

    if builder.load_step_file(str(step_file_relative.resolve())):
        print("加载成功")
        # ...

if __name__ == "__main__":
    run_test_pipeline()