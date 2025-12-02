# ===================================================================
# modules/ASGBuilder/CMakeLists.txt
# Refactored for Modern C++ and Project Consistency
# ===================================================================

# ===================================================================
# ASGBuilder Module Configuration
# ===================================================================

# Define the library target
# Includes all source files identified in the directory structure
add_library(ASGBuilder
        ASGBuilder.h
        ASGBuilder.cpp
        ASGFunction.cpp
)

# ===================================================================
# Python Binding Configuration
# ===================================================================

# Set Python executable path
# Note: Adjust this path to match your Python environment
set(Python_EXECUTABLE "D:/anaconda3/envs/Python_Projects/ModelsDeepLearning/python.exe")

# Find Python package
# Note: Ensure conda environment is activated before running CMake
find_package(Python 3.11.14 COMPONENTS Interpreter Development.Module REQUIRED)

# Auto-detect pybind11 path using Python interpreter
# Executes "python -m pybind11 --cmakedir" to obtain the accurate path
execute_process(
        COMMAND "${Python_EXECUTABLE}" -m pybind11 --cmakedir
        OUTPUT_VARIABLE PYBIND11_CMAKE_DIR
        OUTPUT_STRIP_TRAILING_WHITESPACE
        RESULT_VARIABLE PYBIND11_FIND_RESULT
)

# Add pybind11 cmake directory to search path if found
if (PYBIND11_FIND_RESULT EQUAL 0)
    list(APPEND CMAKE_PREFIX_PATH "${PYBIND11_CMAKE_DIR}")
    message(STATUS "Auto-detected pybind11 config at: ${PYBIND11_CMAKE_DIR}")
else()
    message(WARNING "Could not auto-detect pybind11 path via python. Ensure 'pip install pybind11' runs successfully.")
endif()

# Find pybind11 package
find_package(pybind11 CONFIG REQUIRED)

# Create Python extension module
# Note: Source file must include pybind_module.cpp
pybind11_add_module(ASGBuilder_Py pybind_module.cpp)

# -------------------------------------------------------------------
# Force add pybind11 header file paths
# -------------------------------------------------------------------

# Query Python for pybind11 include directory
execute_process(
        COMMAND "${Python_EXECUTABLE}" -c "import pybind11; print(pybind11.get_include())"
        OUTPUT_VARIABLE PYBIND11_INCLUDE_DIR_USER
        OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Display detected path for debugging
message(STATUS "Force-adding pybind11 headers from: ${PYBIND11_INCLUDE_DIR_USER}")

# Add pybind11 include directory to target
target_include_directories(ASGBuilder_Py PRIVATE "${PYBIND11_INCLUDE_DIR_USER}")

# Add Python include directory to prevent Python.h not found errors
target_include_directories(ASGBuilder_Py PRIVATE "${Python_INCLUDE_DIRS}")

# -------------------------------------------------------------------
# Python Module Linking and Properties
# -------------------------------------------------------------------

# Link the C++ core library (ASGBuilder) to the Python wrapper
target_link_libraries(ASGBuilder_Py PRIVATE ASGBuilder)

# Set output name for the Python module
# This renames the generated .pyd (Windows) or .so (Linux) file to "ASGBuilder"
# Allows direct import in Python: import ASGBuilder
set_target_properties(ASGBuilder_Py PROPERTIES OUTPUT_NAME "ASGCore")

# ===================================================================
# ASGBuilder Library Configuration
# ===================================================================

# Define include directories
# PUBLIC: Appends this directory to the include path of this target
# AND any target that links against it (e.g., the main app)
target_include_directories(ASGBuilder PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

# Link dependencies
# Link against OpenCASCADE libraries defined in the root CMakeLists.txt
target_link_libraries(ASGBuilder PUBLIC
        ${OpenCASCADE_LIBRARIES}
)

# Enforce C++20 standard specifically for this target
target_compile_features(ASGBuilder PUBLIC cxx_std_20)