cmake_minimum_required(VERSION 3.14)

include(FetchContent)

set(GLFW_BUILD_EXAMPLES OFF)
set(GLFW_BUILD_TESTS OFF)
set(GLFW_BUILD_DOCS OFF)
set(GLFW_INSTALL OFF)

FetchContent_Declare(
		glfw
		GIT_REPOSITORY      https://github.com/glfw/glfw.git
		GIT_TAG             3.3.2
)

# GLFW 3.3.2 declares cmake_minimum_required(VERSION 2.x), which CMake 4.0+
# no longer accepts. Allow the old policy version for the GLFW subproject only
# (scope limited to this directory before FetchContent_MakeAvailable).
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

FetchContent_MakeAvailable(glfw)