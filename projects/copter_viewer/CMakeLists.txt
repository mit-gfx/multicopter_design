set(VIEWER_SOURCE_DIR ${PROJECT_SOURCE_DIR}/projects/copter_viewer)

file(GLOB_RECURSE VIEWER_HEADER ${VIEWER_SOURCE_DIR}/*.h)
file(GLOB_RECURSE VIEWER_SOURCE ${VIEWER_SOURCE_DIR}/*.cpp)

add_executable(copter_viewer ${VIEWER_HEADER} ${VIEWER_SOURCE})

# Includes.
target_include_directories(copter_viewer PRIVATE ${OPENGL_VIEWER_HEADER})
target_include_directories(copter_viewer PRIVATE ${VIEWER_SOURCE_DIR}/include/)
target_include_directories(copter_viewer PRIVATE
  ${PROJECT_SOURCE_DIR}/projects/copter_simulation/include/)

target_link_libraries(copter_viewer
  copter_simulation
  tinyxml2
)
