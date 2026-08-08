# -------------------------------------------------------------------
# LibTorch integration for CMake
#
# Locates a LibTorch installation and links it to the given target.
# The project requires PyTorch 2.0+ with CUDA support.
#
# Usage (from CMakeLists.txt):
#     include(cmake/libtorch.cmake)
#     ADD_LIBTORCH(SplatEditor)
#
# Environment variables / CMake cache variables:
#     -DTorch_DIR=<path>/share/cmake/Torch  (CMake standard)
#     LIBTORCH_PATH (env, optional fallback)
#
# Download LibTorch (C++ distribution):
#     https://pytorch.org/get-started/locally/
#     Choose: LibTorch, Stable, Linux/Windows, C++, CUDA 12.4
# -------------------------------------------------------------------

function(ADD_LIBTORCH TARGET_NAME)

    # ------------------------------------------------------------------
    # 1. Try the standard CMake approach first (Torch_DIR)
    # ------------------------------------------------------------------
    find_package(Torch QUIET)
    if(Torch_FOUND)
        message(STATUS "LibTorch found via find_package (Torch_DIR=${Torch_DIR})")
        message(STATUS "  CXX flags:   ${TORCH_CXX_FLAGS}")
        message(STATUS "  Libraries:   ${TORCH_LIBRARIES}")
        message(STATUS "  Include:     ${TORCH_INCLUDE_DIRS}")

        target_compile_definitions(${TARGET_NAME} PRIVATE SPLATSHOP_HAS_LIBTORCH=1)
        target_include_directories(${TARGET_NAME} PRIVATE ${TORCH_INCLUDE_DIRS})
        target_link_libraries(${TARGET_NAME} ${TORCH_LIBRARIES})

        # On Windows, LibTorch ships with its own DLLs; ensure they are
        # available at runtime by copying them to the output directory.
        if(WIN32 AND TORCH_LIBRARIES)
            # Copy the LibTorch DLLs to the build output directory
            add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "$<TARGET_RUNTIME_DLLS:${TARGET_NAME}>"
                    "$<TARGET_FILE_DIR:${TARGET_NAME}>"
                COMMAND_EXPAND_LISTS
                COMMENT "POST BUILD: copying LibTorch runtime DLLs")
        endif()

        return()
    endif()

    # ------------------------------------------------------------------
    # 2. Fallback: check in-project location libs/libtorch
    # ------------------------------------------------------------------
    set(_project_libtorch "${CMAKE_SOURCE_DIR}/libs/libtorch")
    if(NOT DEFINED LIBTORCH_PATH AND EXISTS "${_project_libtorch}/include")
        set(LIBTORCH_PATH "${_project_libtorch}" CACHE PATH "LibTorch install directory")
        message(STATUS "LibTorch auto-detected in project: ${LIBTORCH_PATH}")
    endif()

    # ------------------------------------------------------------------
    # 3. Fallback: try LIBTORCH_PATH environment variable
    # ------------------------------------------------------------------
    if(NOT DEFINED LIBTORCH_PATH AND DEFINED ENV{LIBTORCH_PATH})
        set(LIBTORCH_PATH "$ENV{LIBTORCH_PATH}" CACHE PATH "LibTorch install directory")
    endif()

    if(NOT LIBTORCH_PATH)
        message(WARNING "LibTorch not found. 4DGS support will be disabled.")
        message(WARNING "To enable: install LibTorch and set -DTorch_DIR=<path>/share/cmake/Torch")
        message(WARNING "Or: set LIBTORCH_PATH environment variable to LibTorch root directory.")
        return()
    endif()

    # ------------------------------------------------------------------
    # 3. Manual resolution from LIBTORCH_PATH
    # ------------------------------------------------------------------
    set(TORCH_ROOT "${LIBTORCH_PATH}")

    # Detect include directory
    if(EXISTS "${TORCH_ROOT}/include/torch/csrc/api/include/torch/torch.h")
        set(TORCH_INCLUDE_DIRS
            "${TORCH_ROOT}/include/torch/csrc/api/include"
            "${TORCH_ROOT}/include"
        )
    elseif(EXISTS "${TORCH_ROOT}/include/torch/torch.h")
        set(TORCH_INCLUDE_DIRS "${TORCH_ROOT}/include")
    else()
        message(WARNING "Cannot find torch.h under ${TORCH_ROOT}/include")
        return()
    endif()

    # Detect libraries
    if(WIN32)
        set(TORCH_LIB_DIR "${TORCH_ROOT}/lib")
    else()
        # Linux: the .so files are typically under lib/ or lib64/
        if(EXISTS "${TORCH_ROOT}/lib64")
            set(TORCH_LIB_DIR "${TORCH_ROOT}/lib64")
        else()
            set(TORCH_LIB_DIR "${TORCH_ROOT}/lib")
        endif()
    endif()

    # Build the library list.
    # The set of required libraries depends on the LibTorch distribution.
    # We try to find the core ones.
    set(TORCH_LIBS_LIST)
    macro(try_lib NAME)
        if(WIN32)
            find_library(FOUND_LIB_${NAME}
                NAMES ${NAME}
                PATHS ${TORCH_LIB_DIR}
                NO_DEFAULT_PATH)
            if(FOUND_LIB_${NAME})
                list(APPEND TORCH_LIBS_LIST ${FOUND_LIB_${NAME}})
            endif()
        else()
            find_library(FOUND_LIB_${NAME}
                NAMES lib${NAME}.so ${NAME}
                PATHS ${TORCH_LIB_DIR}
                NO_DEFAULT_PATH)
            if(FOUND_LIB_${NAME})
                list(APPEND TORCH_LIBS_LIST ${FOUND_LIB_${NAME}})
            endif()
        endif()
    endmacro()

    try_lib(torch)
    try_lib(torch_cpu)
    try_lib(torch_cuda)
    try_lib(torch_cuda_cpp)
    try_lib(c10)
    try_lib(c10_cuda)

    if(NOT TORCH_LIBS_LIST)
        message(WARNING "No LibTorch libraries found under ${TORCH_LIB_DIR}")
        return()
    endif()

    message(STATUS "LibTorch resolved manually from LIBTORCH_PATH=${LIBTORCH_PATH}")
    message(STATUS "  Include: ${TORCH_INCLUDE_DIRS}")
    message(STATUS "  Libraries: ${TORCH_LIBS_LIST}")

    # C++ standard: LibTorch uses C++17 on some distributions; we must
    # align the ABI flags.
    target_compile_definitions(${TARGET_NAME} PRIVATE
        SPLATSHOP_HAS_LIBTORCH=1
        _GLIBCXX_USE_CXX11_ABI=1
    )
    target_include_directories(${TARGET_NAME} PRIVATE ${TORCH_INCLUDE_DIRS})
    target_link_libraries(${TARGET_NAME} ${TORCH_LIBS_LIST})

    # On Windows, copy DLLs to output directory
    if(WIN32)
        file(GLOB TORCH_DLLS "${TORCH_LIB_DIR}/*.dll")
        foreach(DLL ${TORCH_DLLS})
            add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${DLL}" "$<TARGET_FILE_DIR:${TARGET_NAME}>"
                COMMENT "POST BUILD: copying LibTorch DLL: ${DLL}")
        endforeach()
    endif()

endfunction()
