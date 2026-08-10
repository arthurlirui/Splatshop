# -------------------------------------------------------------------
# OrbbecSDK-K4A-Wrapper integration for CMake
#
# Locates the vendored OrbbecSDK-K4A-Wrapper installation under
# libs/OrbbecSDK_K4A_Wrapper and links it to the given target. Enables
# the optional K4A-based RGBD camera capture / recording / playback /
# transformation module (parallel to the existing OrbbecSDK integration).
#
# The wrapper tree is NOT committed to git (per policy no SDK/lib binaries
# are pushed to GitHub). Users who want the K4A module download the wrapper
# release and unpack it to libs/OrbbecSDK_K4A_Wrapper/ locally.
#
# Usage (from CMakeLists.txt):
#     include(cmake/k4a.cmake)
#     ADD_K4A(SplatEditor)
#
# The wrapper ships CMake package configs at
#     libs/OrbbecSDK_K4A_Wrapper/lib/cmake/k4a/k4aConfig.cmake
#     libs/OrbbecSDK_K4A_Wrapper/lib/cmake/k4arecord/k4arecordConfig.cmake
# which export the imported targets `k4a::k4a` and `k4a::k4arecord`.
#
# The k4arecord target has a hard INTERFACE_LINK_LIBRARIES dependency on
# `OrbbecSDK::OrbbecSDK`, but the vendored OrbbecSDK exports its target as
# `ob::OrbbecSDK`. We bridge the naming gap by creating an ALIAS target
# `OrbbecSDK::OrbbecSDK` -> `ob::OrbbecSDK` (requires CMake 3.11+; the
# project requires 3.22 so this is fine). If the OrbbecSDK is not present
# (ob::OrbbecSDK undefined) we skip k4arecord and only enable the core k4a
# API, since recording/playback cannot link without the alias.
#
# ADD_K4A must be called AFTER ADD_ORBBEC so that ob::OrbbecSDK is already
# defined when the alias is created.
#
# If the wrapper is not found the function silently returns so the rest of
# the build still succeeds; the K4A module sources compile to empty stubs
# (gated by SPLATSHOP_HAS_K4A).
# -------------------------------------------------------------------

function(ADD_K4A TARGET_NAME)

    set(_K4A_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/libs/OrbbecSDK_K4A_Wrapper")

    if(NOT EXISTS "${_K4A_SDK_DIR}/lib/cmake/k4a/k4aConfig.cmake")
        message(WARNING "OrbbecSDK-K4A-Wrapper not found under ${_K4A_SDK_DIR}. K4A camera module disabled.")
        return()
    endif()

    # --- Core k4a API (device / capture / streaming / transformation) ---
    set(k4a_DIR "${_K4A_SDK_DIR}/lib/cmake/k4a" CACHE PATH "Path to k4aConfig.cmake" FORCE)
    find_package(k4a QUIET NO_DEFAULT_PATH)

    if(NOT k4a_FOUND)
        message(WARNING "k4a package config found but failed to load. K4A camera module disabled.")
        return()
    endif()

    message(STATUS "OrbbecSDK-K4A-Wrapper found at ${_K4A_SDK_DIR}. K4A camera module enabled.")

    target_compile_definitions(${TARGET_NAME} PRIVATE SPLATSHOP_HAS_K4A=1)

    # Use the plain target_link_libraries signature because common.cmake
    # already uses plain calls on this target; mixing keyword and plain
    # signatures on the same target is a CMake error.
    target_link_libraries(${TARGET_NAME} k4a::k4a)

    # --- k4arecord API (MKV recording / playback) ---
    # k4a::k4arecord has INTERFACE_LINK_LIBRARIES "k4a::k4a;OrbbecSDK::OrbbecSDK".
    # The vendored OrbbecSDK exports `ob::OrbbecSDK`, not `OrbbecSDK::OrbbecSDK`.
    # Bridge the naming gap with an ALIAS so the propagated link dependency
    # resolves. Without the alias, linking k4a::k4arecord would fail.
    set(_K4A_RECORD_ENABLED FALSE)
    if(TARGET ob::OrbbecSDK AND NOT TARGET OrbbecSDK::OrbbecSDK)
        add_library(OrbbecSDK::OrbbecSDK ALIAS ob::OrbbecSDK)
    endif()

    if(TARGET OrbbecSDK::OrbbecSDK)
        set(k4arecord_DIR "${_K4A_SDK_DIR}/lib/cmake/k4arecord" CACHE PATH "Path to k4arecordConfig.cmake" FORCE)
        find_package(k4arecord QUIET NO_DEFAULT_PATH)
        if(k4arecord_FOUND)
            message(STATUS "k4arecord package found. K4A recording/playback module enabled.")
            target_compile_definitions(${TARGET_NAME} PRIVATE SPLATSHOP_HAS_K4ARECORD=1)
            target_link_libraries(${TARGET_NAME} k4a::k4arecord)
            set(_K4A_RECORD_ENABLED TRUE)
        else()
            message(WARNING "k4arecord package config found but failed to load. Recording/playback disabled (core K4A still enabled).")
        endif()
    else()
        message(WARNING "OrbbecSDK::OrbbecSDK target not available (OrbbecSDK not found). k4arecord (recording/playback) disabled. Core K4A still enabled.")
    endif()

    # Stage the runtime DLLs next to the executable. Only k4a.dll and
    # k4arecord.dll are the wrapper's own libraries; OrbbecSDK.dll and
    # the extensions directory are owned by ADD_ORBBEC and copied there
    # to avoid a stale/older OrbbecSDK.dll from this wrapper overwriting
    # the canonical one staged by orbbec.cmake.
    if(WIN32)
        set(_K4A_BIN_DIR "${_K4A_SDK_DIR}/bin")
        add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${_K4A_BIN_DIR}/k4a.dll"
                "$<TARGET_FILE_DIR:${TARGET_NAME}>"
            COMMENT "POST BUILD: copying K4A Wrapper runtime files")
        if(_K4A_RECORD_ENABLED)
            add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${_K4A_BIN_DIR}/k4arecord.dll"
                    "$<TARGET_FILE_DIR:${TARGET_NAME}>"
                COMMENT "POST BUILD: copying k4arecord.dll")
        endif()
    endif()

endfunction()
