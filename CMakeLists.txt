cmake_minimum_required(VERSION 3.22)

project(SplatEditor)
set(CMAKE_CXX_STANDARD 23)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)


# add_subdirectory(./libs/openvr-master)

include(cmake/common.cmake)
include(cmake/libtorch.cmake)   # adds ADD_LIBTORCH(target) function
include(cmake/orbbec.cmake)     # adds ADD_ORBBEC(target) function
include(cmake/opencv.cmake)     # adds ADD_OPENCV(target) function (calibration module)

##############################################
## SUBPROJECT: Splat Editor
##############################################

set(TARGET_NAME SplatEditor) 

file(GLOB SplatEditor_SOURCES CONFIGURE_DEPENDS
	src/*.*
	src/actions/*.*
	src/GPUPrefixSums/*.*
	src/GPUSorting/*.*
	src/gui/*.*
	src/motion/*.*
	src/optim/*.*
	src/render/*.*
	src/remesh/*.*
	src/remote/*.*
	src/scene/*.*
	src/update/*.*
	src/loader/*.*
	src/writer/*.*
	src/camera/*.*
	src/calibration/*.*
	include/*.*
)

if (WIN32)
	list(APPEND SplatEditor_SOURCES
		src/win32/GLRenderer_win32.cpp
		src/win32/OpenVRHelper.cpp
	)
elseif (UNIX)
	list(APPEND SplatEditor_SOURCES
		src/unix/GLRenderer_linux.cpp
	)
endif (WIN32)

add_executable(${TARGET_NAME} ${SplatEditor_SOURCES})

if (WIN32)
	# set workdir to ../$(SolutionDir), so that .cu files can be monitored/reloaded/compiled
	set_property(TARGET ${TARGET_NAME} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$(SolutionDir)..")
endif (WIN32)

target_include_directories(${TARGET_NAME} PRIVATE 
	./include
	./src
)

#enable_language(CUDA)

# add some third-party libraries
	ADD_CUDA(${TARGET_NAME})
	ADD_LIBTORCH(${TARGET_NAME})   # 4DGS support (optional; silently skips if not found)
	ADD_ORBBEC(${TARGET_NAME})     # Orbbec RGBD camera support (optional; silently skips if not found)
	ADD_OPENCV(${TARGET_NAME})     # OpenCV calibration support (optional; silently skips if not found)
	ADD_OPENGL(${TARGET_NAME})
ADD_GLM(${TARGET_NAME})
ADD_IMGUI(${TARGET_NAME})
ADD_IMPLOT(${TARGET_NAME})
ADD_IMGUIZMO(${TARGET_NAME})
target_include_directories(${TARGET_NAME} PRIVATE ./include)
target_include_directories(${TARGET_NAME} PRIVATE ./libs)

# OpenVR (SteamVR) integration. The headers under libs/openvr/headers are
# tracked in git (they are source, not binaries), but the import library
# (openvr_api.lib) and runtime DLL are NOT committed - per user policy no
# lib/sdk binaries go to GitHub. Users who want VR install the OpenVR SDK
# locally; if the .lib is absent the build gracefully degrades to the
# NO_OPENVR stubs so everything else still compiles.
target_include_directories(${TARGET_NAME} PRIVATE ./libs/openvr/headers)
if (WIN32 AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/libs/openvr/lib/win64/openvr_api.lib")
	target_link_libraries(${TARGET_NAME} ../libs/openvr/lib/win64/openvr_api)
	add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy "$(SolutionDir)../libs/openvr/bin/win64/openvr_api.dll" "$(OutDir)"
		COMMENT "POST BUILD: copying openvr folder")
else()
	# No openvr_api.lib present (or non-Windows): disable VR support.
	add_compile_definitions(NO_OPENVR)
endif ()

if (UNIX)
	find_package(TBB REQUIRED)
	target_link_libraries(${TARGET_NAME} TBB::tbb)
endif (UNIX)

##############################################
## SOME FINAL STUFF
##############################################

# set startup project
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT SplatEditor)