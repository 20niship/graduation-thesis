cmake_minimum_required(VERSION 3.1)

cmake_policy(SET CMP0072 NEW)
set(OpenGL_GL_PREFERENCE LEGACY)

find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)

message(" ${PROJECT_SOURCE_DIR}   ${CMAKE_CURRENT_SOURCE_DIR}")

set(HR4C_INCLUDES  PARENT_SCOPE
  ${PROJECT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/cpptoml/include
  ${CMAKE_CURRENT_SOURCE_DIR}/spdlog/include
  ${CMAKE_CURRENT_SOURCE_DIR}/imgui
  ${CMAKE_CURRENT_SOURCE_DIR}/imgui/backends
  ${CMAKE_CURRENT_SOURCE_DIR}/implot
  ${GLEW_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OPENGL_INCLUDE_DIRS}
  ${GLFW_INCLUDE_DIRS}
)

include_directories(
  ${HR4C_INCLUDES}
)

set(ALL_LINKS 
  Threads::Threads
  modbus
  ${GLEW_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${GLFW_LIBRARIES}
  glfw
  ${GLFW_LIBRARY_DIRS}
)

file(GLOB hr4c_CORE_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/core/*.cpp)
add_library(hr4c-core ${hr4c_CORE_SOURCE})
target_link_libraries(hr4c-core ${ALL_LINKS})
set(ALL_LINKS ${ALL_LINKS} hr4c-core)

file(GLOB hr4c_Graphics_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/graphics/*.cpp)
set(DEAR_IMGUI_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/imgui)
set(IMPLOT_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/implot)
add_library(hr4c-graphics 
  ${hr4c_Graphics_SOURCE}

  ${DEAR_IMGUI_PREFIX}/imgui.cpp
  ${DEAR_IMGUI_PREFIX}/imgui_draw.cpp
  ${DEAR_IMGUI_PREFIX}/imgui_tables.cpp
  ${DEAR_IMGUI_PREFIX}/imgui_widgets.cpp
  ${DEAR_IMGUI_PREFIX}/imgui_demo.cpp
  ${DEAR_IMGUI_PREFIX}/backends/imgui_impl_glfw.cpp
  ${DEAR_IMGUI_PREFIX}/backends/imgui_impl_opengl3.cpp

  ${IMPLOT_PREFIX}/implot.cpp
  ${IMPLOT_PREFIX}/implot_demo.cpp
  ${IMPLOT_PREFIX}/implot_items.cpp
)
target_link_libraries(hr4c-graphics ${ALL_LINKS})
set(ALL_LINKS ${ALL_LINKS} hr4c-graphics)

set(ALL_LINKS hr4c-core hr4c-graphics PARENT_SCOPE)

