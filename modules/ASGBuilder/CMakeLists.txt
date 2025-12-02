# modules/ASGBuilder/CMakeLists.txt
# Refactored for Modern C++ and Project Consistency

# ===================================================================
#   ASGBuilder Module Configuration
# ===================================================================

# Define the library target
# Includes all source files identified in the directory structure
add_library(ASGBuilder
        ASGBuilder.h
        ASGBuilder.cpp
        ASGFunction.cpp
)


#D:\anaconda3\envs\Python_Projects\CodeReproduction\python.exe
set(Python_EXECUTABLE "D:/anaconda3/envs/Python_Projects/CodeReproduction/python.exe")

# Anaconda 用户注意：确保在运行 CMake 前激活了你的 conda 环境
find_package(Python 3.14.0 COMPONENTS Interpreter Development.Module REQUIRED)


# 2. [新增] 自动探测 pybind11 路径
# 利用 Python 解释器执行 "python -m pybind11 --cmakedir" 获取准确路径
execute_process(
        COMMAND "${Python_EXECUTABLE}" -m pybind11 --cmakedir
        OUTPUT_VARIABLE PYBIND11_CMAKE_DIR
        OUTPUT_STRIP_TRAILING_WHITESPACE
        RESULT_VARIABLE PYBIND11_FIND_RESULT
)

if (PYBIND11_FIND_RESULT EQUAL 0)
    # 将找到的路径加入 CMake 的搜索列表
    list(APPEND CMAKE_PREFIX_PATH "${PYBIND11_CMAKE_DIR}")
    message(STATUS "Auto-detected pybind11 config at: ${PYBIND11_CMAKE_DIR}")
else()
    message(WARNING "Could not auto-detect pybind11 path via python. Ensure 'pip install pybind11' runs successfully.")
endif()

# 3. 查找 Pybind11 (现在应该能找到了)
find_package(pybind11 CONFIG REQUIRED)

# 创建 Python 扩展模块
# 注意：源文件必须包含我们马上要写的 pybind_module.cpp
pybind11_add_module(ASGBuilder_Py pybind_module.cpp)


# -------------------------------------------------------------------
# [新增/修复] 强制获取并添加 pybind11 头文件路径
# -------------------------------------------------------------------
# 直接询问 Python：你的 pybind11 头文件在哪里？
execute_process(
        COMMAND "${Python_EXECUTABLE}" -c "import pybind11; print(pybind11.get_include())"
        OUTPUT_VARIABLE PYBIND11_INCLUDE_DIR_USER
        OUTPUT_STRIP_TRAILING_WHITESPACE
)

# 打印出来看看（调试用）
message(STATUS "Force-adding pybind11 headers from: ${PYBIND11_INCLUDE_DIR_USER}")

# 将这个路径强制加入到我们的目标中
target_include_directories(ASGBuilder_Py PRIVATE "${PYBIND11_INCLUDE_DIR_USER}")

# 还要加上 Python 自己的头文件 (Python.h)，防止报找不到 Python.h
target_include_directories(ASGBuilder_Py PRIVATE "${Python_INCLUDE_DIRS}")
# -------------------------------------------------------------------




# 链接依赖
# 将你的 C++ 核心库 (ASGBuilder) 链接到这个 Python 包装器上
target_link_libraries(ASGBuilder_Py PRIVATE ASGBuilder)

# [关键] 设置输出名称
# 这会将生成的 .pyd (Windows) 或 .so (Linux) 文件重命名为 "ASGBuilder"
# 这样你在 Python 里才能直接写: import ASGBuilder
set_target_properties(ASGBuilder_Py PROPERTIES OUTPUT_NAME "ASGBuilder")

# Define Include Directories
# PUBLIC: Appends this directory to the include path of this target
# AND any target that links against it (e.g., the main app).
target_include_directories(ASGBuilder PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

# Link Dependencies
# Link against OpenCASCADE libraries defined in the root CMakeLists.txt
target_link_libraries(ASGBuilder PUBLIC
        ${OpenCASCADE_LIBRARIES}
)

# Enforce C++20 Standard specifically for this target
target_compile_features(ASGBuilder PUBLIC cxx_std_20)