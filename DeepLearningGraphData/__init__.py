import sys
import os
import platform
from pathlib import Path

# ==============================================================================
# 1. 动态计算相对路径 (Dynamic Path Resolution)
# ==============================================================================

# 获取当前 __init__.py 文件的绝对路径
# e.g., .../3DModel_GeometryAnalysisPlatform/ASG/__init__.py
current_package_dir = Path(__file__).parent.resolve()

# 推导项目根目录 (假设 ASG 在项目根目录下)
# e.g., .../3DModel_GeometryAnalysisPlatform
project_root = current_package_dir.parent

# 推导依赖库根目录 (假设 dependencies 与项目根目录是兄弟关系)
# 逻辑：ProjectRoot -> 上一级 -> dependencies
# e.g., .../OCCTProjectWorkspace/dependencies
dependencies_root = project_root.parent / "dependencies"

# ==============================================================================
# 2. 配置 .pyd (C++ 扩展) 的搜索路径
# ==============================================================================

# 定义编译输出目录 (根据你的 CMake 设置，Release 模式)
# 如果多人开发中有人用 Debug，可以加一个判断逻辑
pyd_dir_release = project_root / "cmake-build-release" / "modules" / "ASGBuilder"
pyd_dir_debug = project_root / "cmake-build-debug" / "modules" / "ASGBuilder"

# 优先查找 Release，如果没有则查找 Debug
if pyd_dir_release.exists():
    pyd_path = pyd_dir_release
elif pyd_dir_debug.exists():
    pyd_path = pyd_dir_debug
    print(f"[ASG Warning] Loaded Debug build from: {pyd_path}")
else:
    # 也可以选择不报错，假设用户已经把它安装到了 site-packages
    raise FileNotFoundError(f"Cannot find compiled extension (ASGCore.pyd) in build directories.\n"
                            f"Checked: \n  {pyd_dir_release}\n  {pyd_dir_debug}")

# 将 .pyd 所在目录加入 Python 模块搜索路径
if str(pyd_path) not in sys.path:
    sys.path.append(str(pyd_path))

# ==============================================================================
# 3. 配置 DLL 运行时依赖 (Windows Specific)
# ==============================================================================

if platform.system() == "Windows":
    # 定义所有需要添加的 DLL 目录 (使用相对路径)
    # 注意：这里的文件夹名 (如 freetype-2.13.3) 需要每个人保持一致
    # 如果版本经常变动，可以使用 glob 模糊匹配

    occt_bin = dependencies_root / "occt-install-release" / "win64" / "vc14" / "bin"

    # 3rdparty 路径
    thirdparty_root = dependencies_root / "3rdparty"

    dll_paths = [
        occt_bin,
        thirdparty_root / "freetype-2.13.3-x64" / "bin",
        thirdparty_root / "freeimage-3.18.0-x64" / "bin",
        thirdparty_root / "tcltk-8.6.15-x64" / "bin"
    ]

    for path in dll_paths:
        if path.exists():
            try:
                # Python 3.8+ 必须显式添加 DLL 目录
                os.add_dll_directory(str(path))
            except Exception as e:
                print(f"[ASG Error] Failed to add DLL directory: {path}\n{e}")
        else:
            print(f"[ASG Warning] DLL path not found: {path}")

# ==============================================================================
# 4. 导入核心模块并暴露接口
# ==============================================================================

try:
    # 这里的 ASGCore 就是 cmake 生成的 .pyd 文件名
    import ASGCore

    # 将 ASGCore 中的类暴露到 ASG 包的顶层
    # 这样用户可以 `from ASG import ASGBuilder` 而不是 `from ASG.ASGCore import ASGBuilder`
    from ASGCore import ASGBuilder, AtomType, ConstraintType

    # 可选：打印加载成功信息
    # print(f"[ASG] Successfully initialized from {pyd_path}")

except ImportError as e:
    raise ImportError(f"Failed to import ASGCore extension. Ensure dependencies are correct. Error: {e}")

__all__ = ['ASGBuilder', 'AtomType', 'ConstraintType']