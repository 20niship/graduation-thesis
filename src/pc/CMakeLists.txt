cmake_minimum_required(VERSION 3.1)
project(hr4c)

set(CMAKE_EXPORT_COMPILE_COMMANDS "on") # compile_commands.json作成

enable_language(CXX)
set(CMAKE_CXX_STANDARD 20)

if(${COMPILER_SUPPORTS_CXX20})
  set(CMAKE_CXX_FLAGS "-std=c++20 ${CMAKE_CXX_FLAGS}")
elseif(${COMPILER_SUPPORTS_CXX2A})
  set(CMAKE_CXX_FLAGS "-std=c++2a ${CMAKE_CXX_FLAGS}")
endif()

find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/hr4c/cpptoml/include
  ${CMAKE_CURRENT_SOURCE_DIR}/hr4c/spdlog/include
  ${EIGEN3_INCLUDE_DIR}
)

set(ALL_LINKS 
  Threads::Threads
  modbus
)

file(GLOB hr4c_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/hr4c/*.cpp)
add_library(hr4c ${hr4c_SOURCE})
target_link_libraries(hr4c ${ALL_LINKS})
set(ALL_LINKS ${ALL_LINKS} hr4c)

file(GLOB sources ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
foreach(path IN LISTS sources)
  get_filename_component(target ${path} NAME_WE)
  message("new target : ${target}")
  add_executable(${target} ${path} )
  target_link_libraries(${target} ${ALL_LINKS})
endforeach()

