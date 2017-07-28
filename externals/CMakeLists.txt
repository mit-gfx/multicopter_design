# Command relevant to tinyxml2.
# Integrating tinyxml2's default CMakeLists.txt is surprisingly hard. So I will
# simply rewrite a shorter version.
add_library(tinyxml2 tinyxml2/tinyxml2.h tinyxml2/tinyxml2.cpp)

# Command relevant to nlopt.
# Disable unnecessary build.
set(BUILD_PYTHON OFF CACHE BOOL "build python bindings")
set(BUILD_OCTAVE OFF CACHE BOOL "build octave bindings")
set(BUILD_MATLAB OFF CACHE BOOL "build matlab bindings")
set(BUILD_GUILE OFF CACHE BOOL "build guile bindings")
set(USE_SWIG OFF CACHE BOOL "use SWIG to build bindings")
add_subdirectory(nlopt/)