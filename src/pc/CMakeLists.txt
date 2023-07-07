cmake_minimum_required(VERSION 3.1)
project(hr4c-main)

set(CMAKE_EXPORT_COMPILE_COMMANDS "on") # compile_commands.json作成

include(cmake/config.cmake)
add_subdirectory(hr4c)

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/cpptoml/include
  ${CMAKE_CURRENT_SOURCE_DIR}/spdlog/include
  ${CMAKE_CURRENT_SOURCE_DIR}/hr4c/cpptoml/include
  ${CMAKE_CURRENT_SOURCE_DIR}/hr4c/spdlog/include
  ${CMAKE_CURRENT_SOURCE_DIR}/hr4c/imgui
  ${CMAKE_CURRENT_SOURCE_DIR}/hr4c/implot
  ${GLEW_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OPENGL_INCLUDE_DIRS}
  ${GLFW_INCLUDE_DIRS}
)

file(GLOB sources ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
foreach(path IN LISTS sources)
  get_filename_component(target ${path} NAME_WE)
  message("new target : ${target} ${path}")
  add_executable(${target} ${path})
  target_link_libraries(${target} ${ALL_LINKS})
endforeach()

