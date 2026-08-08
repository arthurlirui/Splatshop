# -------------------------------------------------------------------
# OrbbecSDK integration for CMake
#
# Locates the vendored OrbbecSDK (libobsensor) installation under
# libs/OrbbecSDK and links it to the given target. Enables the optional
# RGBD camera capture module.
#
# The OrbbecSDK tree is NOT committed to git (per policy no SDK/lib
# binaries are pushed to GitHub). Users who want the RGBD camera module
# download the OrbbecSDK and unpack it to libs/OrbbecSDK/ locally.
#
# Usage (from CMakeLists.txt):
#     include(cmake/orbbec.cmake)
#     ADD_ORBBEC(SplatEditor)
#
# The SDK ships its own CMake package config at
#     libs/OrbbecSDK/lib/OrbbecSDKConfig.cmake
# which exports the imported target `ob::OrbbecSDK`.
#
# If the SDK is not found the function silently returns so the rest of
# the build still succeeds; the camera module sources compile to empty
# stubs (gated by SPLATSHOP_HAS_ORBBEC).
# -------------------------------------------------------------------

function(ADD_ORBBEC TARGET_NAME)

    set(_ORBBEC_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/libs/OrbbecSDK")

    if(NOT EXISTS "${_ORBBEC_SDK_DIR}/lib/OrbbecSDKConfig.cmake")
        message(WARNING "OrbbecSDK not found under ${_ORBBEC_SDK_DIR}. RGBD camera module disabled.")
        return()
    endif()

    # Tell find_package where the package config lives. The package name is
    # "OrbbecSDK" so the lookup variable is OrbbecSDK_DIR (CMake convention).
    set(OrbbecSDK_DIR "${_ORBBEC_SDK_DIR}/lib" CACHE PATH "Path to OrbbecSDKConfig.cmake" FORCE)
    find_package(OrbbecSDK QUIET NO_DEFAULT_PATH)

    if(NOT OrbbecSDK_FOUND)
        message(WARNING "OrbbecSDK package config found but failed to load. RGBD camera module disabled.")
        return()
    endif()

    message(STATUS "OrbbecSDK found at ${_ORBBEC_SDK_DIR}. RGBD camera module enabled.")

    target_compile_definitions(${TARGET_NAME} PRIVATE SPLATSHOP_HAS_ORBBEC=1)
    # Use the plain target_link_libraries signature because common.cmake
    # already uses plain calls on this target; mixing keyword and plain
    # signatures on the same target is a CMake error.
    target_link_libraries(${TARGET_NAME} ob::OrbbecSDK)

    # Stage the runtime DLLs next to the executable. OrbbecSDK.dll plus
    # the extensions directory (depth engine / filters / frame processor)
    # must be available at runtime.
    if(WIN32)
        set(_ORBBEC_BIN_DIR "${_ORBBEC_SDK_DIR}/bin")
        add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${_ORBBEC_BIN_DIR}/OrbbecSDK.dll"
                "$<TARGET_FILE_DIR:${TARGET_NAME}>"
            COMMAND ${CMAKE_COMMAND} -E copy_directory
                "${_ORBBEC_BIN_DIR}/extensions"
                "$<TARGET_FILE_DIR:${TARGET_NAME}>/extensions"
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${_ORBBEC_BIN_DIR}/OrbbecSDKConfig.xml"
                "$<TARGET_FILE_DIR:${TARGET_NAME}>"
            COMMENT "POST BUILD: copying OrbbecSDK runtime files")
    endif()

endfunction()
