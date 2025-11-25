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