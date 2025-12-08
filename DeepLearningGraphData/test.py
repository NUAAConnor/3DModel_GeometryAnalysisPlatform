import sys
from pathlib import Path

# Add the project root to sys.path so the ASG package can be imported
# (Once ASG ships as an installed package this hack becomes unnecessary)
current_dir = Path(__file__).parent
project_root = current_dir.parent
sys.path.append(str(project_root))

# ==========================================
# Import stays minimal thanks to package initialization
# ==========================================
# ASGBuilder = None  # Ensure symbol exists even if the guarded import fails
try:
    # Directly import the package; __init__.py configures DLLs and paths
    from DeepLearningGraphData import ASGBuilder
except ImportError as e:
    print(f"Initialization failed: {e}")
    sys.exit(1)


def run_test_pipeline():
    print("--- Initializing builder ---")
    builder = ASGBuilder  # Invoke directly

    # ... Remaining logic is unchanged ...
    # Relative paths keep the workflow portable across machines
    step_file_relative = project_root / ".." / "TestModels" / "The Baseline_Ass.stp"

    if builder.load_step_file(str(step_file_relative.resolve())):
        print("STEP load succeeded")
        # ...


if __name__ == "__main__":
    run_test_pipeline()