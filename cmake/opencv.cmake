# -------------------------------------------------------------------
# OpenCV integration for CMake
#
# Locates OpenCV and links it to the given target. Enables the optional
# RGBD camera calibration module (chessboard detection, lens
# undistortion, intrinsics optimisation — see src/calibration/).
#
# Usage (from CMakeLists.txt):
#     include(cmake/opencv.cmake)
#     ADD_OPENCV(SplatEditor)
#
# Lookup order:
#   1. The vendored OpenCV 4.x prebuilt package under libs/opencv/build
#      (preferred — no environment setup needed, mirrors the OrbbecSDK
#      vendoring approach). OpenCVConfig.cmake lives at
#      libs/opencv/build/OpenCVConfig.cmake so the lookup variable is
#      OpenCV_DIR = libs/opencv/build.
#   2. A system OpenCV found via find_package(OpenCV) default search
#      (honours the OpenCV_DIR / OpenCV_ROOT environment variables).
#
# Only the core / imgproc / calib3d components are required (highgui is
# not used — all preview is done through ImGui + GL). If OpenCV is not
# found the function silently returns so the rest of the build still
# succeeds; the calibration sources compile to empty stubs gated by
# SPLATSHOP_HAS_OPENCV.
# -------------------------------------------------------------------

function(ADD_OPENCV TARGET_NAME)

    set(_OPENCV_VENDORED_DIR "${CMAKE_CURRENT_SOURCE_DIR}/libs/opencv/build")

    # 1) Try the vendored prebuilt package first.
    if(EXISTS "${_OPENCV_VENDORED_DIR}/OpenCVConfig.cmake")
        # The vendored OpenCV 4.11 Windows pack ships vc16 (VS2019) binaries
        # which are ABI-compatible with the v143 toolset (VC14.x ABI) this
        # project builds with. However OpenCVConfig.cmake's MSVC_VERSION
        # table only covers up to ^19[34][0-9]$ (VS2022, MSVC 1930-1949);
        # newer toolsets (e.g. MSVC 1951 / VS2026) leave OpenCV_RUNTIME
        # empty and the package is rejected. Pre-set OpenCV_ARCH and
        # OpenCV_RUNTIME so the config script's auto-detection is bypassed
        # (it honours pre-defined values — see OpenCVConfig.cmake:96-97).
        if(WIN32 AND MSVC AND NOT DEFINED OpenCV_ARCH)
            set(OpenCV_ARCH "x64" CACHE STRING "OpenCV target arch (vendored override)" FORCE)
        endif()
        if(WIN32 AND MSVC AND NOT DEFINED OpenCV_RUNTIME)
            set(OpenCV_RUNTIME "vc16" CACHE STRING "OpenVC runtime (vendored override)" FORCE)
        endif()
        set(OpenCV_DIR "${_OPENCV_VENDORED_DIR}" CACHE PATH
            "Path to OpenCVConfig.cmake (vendored)" FORCE)
        # HINTS picks up OpenCV_DIR; NO_CMAKE_SYSTEM_PATH avoids pulling in a
        # stray system OpenCV. Do NOT use NO_DEFAULT_PATH — that also drops
        # the HINTS-derived directory.
        find_package(OpenCV QUIET HINTS "${_OPENCV_VENDORED_DIR}"
                     NO_CMAKE_SYSTEM_PATH COMPONENTS core imgproc calib3d)
    endif()

    # 2) Fall back to a system OpenCV install.
    if(NOT OpenCV_FOUND)
        find_package(OpenCV QUIET COMPONENTS core imgproc calib3d)
    endif()

    if(NOT OpenCV_FOUND)
        message(WARNING "OpenCV not found. RGBD camera calibration module disabled. "
                        "Vendored package expected at libs/opencv/build/OpenCVConfig.cmake.")
        return()
    endif()

    message(STATUS "OpenCV ${OpenCV_VERSION} found. RGBD camera calibration module enabled.")

    target_compile_definitions(${TARGET_NAME} PRIVATE SPLATSHOP_HAS_OPENCV=1)
    # Use the plain target_link_libraries signature to match the other
    # ADD_* helpers in this project (common.cmake uses plain calls on
    # this target; mixing keyword and plain signatures is a CMake error).
    target_link_libraries(${TARGET_NAME} ${OpenCV_LIBS})
    target_include_directories(${TARGET_NAME} PRIVATE ${OpenCV_INCLUDE_DIRS})

    # Stage the runtime DLLs next to the executable so the calibration
    # module loads at runtime. The vendored package ships the opencv_world
    # DLL under build/x64/vc16/bin (vc16 == VS2019, ABI-compatible with the
    # vc14.x toolset this project builds with).
    if(WIN32)
        set(_OPENCV_BIN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/libs/opencv/build/x64/vc16/bin")
        if(EXISTS "${_OPENCV_BIN_DIR}/opencv_world4110.dll")
            add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${_OPENCV_BIN_DIR}/opencv_world4110.dll"
                    "$<TARGET_FILE_DIR:${TARGET_NAME}>"
                COMMENT "POST BUILD: copying OpenCV runtime DLL")
        endif()
    endif()

endfunction()
