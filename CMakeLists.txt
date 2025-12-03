# CMake Project Configuration for 3DModel_GeometryAnalysisPlatform
# Refactored for Modern C++ and CLion/MSVC environment

cmake_minimum_required(VERSION 3.20)

# Enable Hot Reload for MSVC if supported (CMP0141)
if(POLICY CMP0141)
    cmake_policy(SET CMP0141 NEW)
endif()

project("3DModel_GeometryAnalysisPlatform" LANGUAGES CXX)

# ===================================================================
#   Build Type Configuration (Added)
# ===================================================================

# Set a default build type if none was specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'Release' as none was specified.")
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

message(STATUS "Current Build Type: ${CMAKE_BUILD_TYPE}")

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
# ===================================================================

# Define Dependencies Root
get_filename_component(DEPS_ROOT "${CMAKE_SOURCE_DIR}/../dependencies" ABSOLUTE)
message(STATUS "Dependencies Root: ${DEPS_ROOT}")

# -------------------------------------------------------------------
#   Configurable Paths based on Build Type
# -------------------------------------------------------------------

# Determine folder names based on build type
if(CMAKE_BUILD_TYPE MATCHES "^[Dd][Ee][Bb][Uu][Gg]$")
    # Debug Mode
    set(OCCT_INSTALL_FOLDER_NAME "occt-install-debug")
    set(OCCT_BIN_FOLDER_NAME "bind") # Debug binaries often in 'bind'
    message(STATUS "Configuring for DEBUG mode (using ${OCCT_INSTALL_FOLDER_NAME})")
else()
    # Release Mode (Release, RelWithDebInfo, MinSizeRel)
    set(OCCT_INSTALL_FOLDER_NAME "occt-install-release")
    set(OCCT_BIN_FOLDER_NAME "bin")  # Release binaries usually in 'bin'
    message(STATUS "Configuring for RELEASE mode (using ${OCCT_INSTALL_FOLDER_NAME})")
endif()

# Set OCCT and 3rdparty paths
set(OpenCASCADE_DIR "${DEPS_ROOT}/${OCCT_INSTALL_FOLDER_NAME}/cmake")
set(THIRDPARTY_DIR  "${DEPS_ROOT}/3rdparty")

# Validate OCCT path existence
if(NOT EXISTS "${OpenCASCADE_DIR}")
    message(FATAL_ERROR "OCCT 'cmake' directory not found at: ${OpenCASCADE_DIR}\n"
            "Current Build Type: ${CMAKE_BUILD_TYPE}\n"
            "Please check if the directory '${OCCT_INSTALL_FOLDER_NAME}' exists in dependencies.")
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

    # Resolve the root from the cmake dir: .../occt-install-X/cmake/.. -> .../occt-install-X
    get_filename_component(OCCT_INSTALL_ROOT "${OpenCASCADE_DIR}/.." REALPATH)

    # Use the variable defined at the top (bind or bin)
    set(OCCT_BIN_DIR "${OCCT_INSTALL_ROOT}/${OCCT_PLATFORM_SUBDIR}/${OCCT_BIN_FOLDER_NAME}")

    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${OCCT_BIN_DIR}"
            "${APP_DIR}"
            COMMENT "Copying OCCT DLLs (${OCCT_BIN_FOLDER_NAME}) to output directory..."
    )

    # 2. Copy 3rdparty DLLs
    set(THIRDPARTY_LIBS_TO_COPY
            "tcltk-8.6.15-x64"
            "freetype-2.13.3-x64"
            "freeimage-3.18.0-x64"
            # Add additional 3rdparty folders here if needed
    )

    foreach(LIB_NAME ${THIRDPARTY_LIBS_TO_COPY})
        # Check if 'bind' exists for 3rdparty in Debug, otherwise fallback to 'bin'
        set(LIB_BIN_DIR_DEBUG "${THIRDPARTY_DIR}/${LIB_NAME}/bind")
        set(LIB_BIN_DIR_RELEASE "${THIRDPARTY_DIR}/${LIB_NAME}/bin")

        set(TARGET_LIB_BIN_DIR "")

        if(CMAKE_BUILD_TYPE MATCHES "^[Dd][Ee][Bb][Uu][Gg]$" AND EXISTS "${LIB_BIN_DIR_DEBUG}")
            set(TARGET_LIB_BIN_DIR "${LIB_BIN_DIR_DEBUG}")
        elseif(EXISTS "${LIB_BIN_DIR_RELEASE}")
            set(TARGET_LIB_BIN_DIR "${LIB_BIN_DIR_RELEASE}")
        endif()

        if(EXISTS "${TARGET_LIB_BIN_DIR}")
            add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
                    COMMAND ${CMAKE_COMMAND} -E copy_directory
                    "${TARGET_LIB_BIN_DIR}"
                    "${APP_DIR}"
                    COMMENT "Copying ${LIB_NAME} DLLs from ${TARGET_LIB_BIN_DIR}..."
            )
        else()
            message(WARNING "Could not find bin directory for ${LIB_NAME}. Expected at ${LIB_BIN_DIR_RELEASE} (or bind)")
        endif()
    endforeach()
endif()