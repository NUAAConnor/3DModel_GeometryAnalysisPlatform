# CMake Project Configuration for 3DModel_GeometryAnalysisPlatform
# Refactored for Modern C++ and CLion/MSVC environment

cmake_minimum_required(VERSION 3.20)

# Enable Hot Reload for MSVC if supported (CMP0141)
if(POLICY CMP0141)
    cmake_policy(SET CMP0141 NEW)
endif()

project("3DModel_GeometryAnalysisPlatform" LANGUAGES CXX)

# ===================================================================
#   Resolving Chinese character encoding issues
# ===================================================================
if(MSVC)
    add_compile_options("/utf-8")
endif()


# ===================================================================
#   C++ Standard Configuration
# ===================================================================

# Enforce C++20 standard directly.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF) # Use standard C++, not MSVC specific extensions

# ===================================================================
#   Dependencies Path Configuration
#   依赖路径配置
# ===================================================================

# Define Dependencies Root
get_filename_component(DEPS_ROOT "${CMAKE_SOURCE_DIR}/../dependencies" ABSOLUTE)
message(STATUS "Dependencies Root: ${DEPS_ROOT}")

# Set OCCT and 3rdparty paths
set(OpenCASCADE_DIR "${DEPS_ROOT}/occt-install-d/cmake")
set(THIRDPARTY_DIR  "${DEPS_ROOT}/3rdparty")

# Validate OCCT path existence
if(NOT EXISTS "${OpenCASCADE_DIR}")
    message(FATAL_ERROR "OCCT 'cmake' directory not found at: ${OpenCASCADE_DIR}\n"
            "Please verify your directory structure matches the project requirements.")
endif()

# ===================================================================
#   Find OpenCASCADE
# ===================================================================

# Find OCCT package - This makes OCCT variables available to submodules
find_package(OpenCASCADE 7.9.2 REQUIRED)

if(OpenCASCADE_FOUND)
    message(STATUS "OpenCASCADE ${OpenCASCADE_VERSION} found at: ${OpenCASCADE_DIR}")
else()
    message(FATAL_ERROR "OpenCASCADE not found!")
endif()

# ===================================================================
#   Subdirectories (Modules)
# ===================================================================

add_subdirectory(modules/ASGBuilder)

# ===================================================================
#   Main Application Target
# ===================================================================

# Define source files for the main application
set(APP_SOURCES
        "${CMAKE_CURRENT_SOURCE_DIR}/app/main.cpp"
)

# Create the executable target
add_executable(${PROJECT_NAME} ${APP_SOURCES})

# Configure include directories
target_include_directories(${PROJECT_NAME} PRIVATE
        ${OpenCASCADE_INCLUDE_DIR}
        "${CMAKE_CURRENT_SOURCE_DIR}/modules/ASGBuilder"
)

# Link libraries
target_link_libraries(${PROJECT_NAME} PRIVATE
        ASGBuilder
)


# ===================================================================
#   Post-Build: DLL Deployment (Windows/MSVC)
# ===================================================================

if(MSVC)
    set(APP_DIR $<TARGET_FILE_DIR:${PROJECT_NAME}>)

    # Define OCCT platform subdirectory (Verify this matches your installation)
    set(OCCT_PLATFORM_SUBDIR "win64/vc14")

    # 1. Copy OCCT DLLs
    # Resolve the root from the cmake dir: .../occt-install/cmake/.. -> .../occt-install
    get_filename_component(OCCT_INSTALL_ROOT "${OpenCASCADE_DIR}/.." REALPATH)

    # Note: Using 'bin' for Release/RelWithDebInfo.
    # If you strictly need Debug DLLs (often in 'bind'), you might need a configuration check here.
    set(OCCT_BIN_DIR "${OCCT_INSTALL_ROOT}/${OCCT_PLATFORM_SUBDIR}/bind")

    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${OCCT_BIN_DIR}"
            "${APP_DIR}"
            COMMENT "Copying OCCT DLLs to output directory..."
    )

    # 2. Copy 3rdparty DLLs
    set(THIRDPARTY_LIBS_TO_COPY
            "tcltk-8.6.15-x64"
            "freetype-2.13.3-x64"
            "freeimage-3.18.0-x64"
            # Add additional 3rdparty folders here if needed
    )

    foreach(LIB_NAME ${THIRDPARTY_LIBS_TO_COPY})
        set(LIB_BIN_DIR "${THIRDPARTY_DIR}/${LIB_NAME}/bin")
        if(EXISTS "${LIB_BIN_DIR}")
            add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
                    COMMAND ${CMAKE_COMMAND} -E copy_directory
                    "${LIB_BIN_DIR}"
                    "${APP_DIR}"
                    COMMENT "Copying ${LIB_NAME} DLLs..."
            )
        endif()
    endforeach()
endif()