cmake_minimum_required(VERSION 3.1.0)

set(CMAKE_SUPPRESS_REGENERATION true)

project(multicopter_design)

# Enable C++ 11
set(CMAKE_CXX_STANDARD 11)

# Suppress warnings.
add_definitions(
  -D_CRT_SECURE_NO_WARNINGS
)
if(UNIX)
  set(GCC_COVERAGE_COMPILE_FLAGS "-Wno-format")
  add_definitions(${GCC_COVERAGE_COMPILE_FLAGS})
endif()

# Expose PROJECT_SOURCE_DIR to the source code.
add_definitions(-DPROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

# Commands relevant to graphics_codebase:
set(OPENGL_VIEWER_ROOT ${PROJECT_SOURCE_DIR}/externals/opengl_viewer)
add_subdirectory(${OPENGL_VIEWER_ROOT})
get_directory_property(OEPNGL_VIEWER_HEADER
  DIRECTORY ${OPENGL_VIEWER_ROOT}
  DEFINITION OPENGL_VIEWER_HEADER)

# Other external libraries.
add_subdirectory(externals/)

# Include separate projects.
add_subdirectory(projects/copter_simulation/)
add_subdirectory(projects/copter_viewer/)
