cmake_minimum_required(VERSION 3.1)
project(h4rc)

set(CMAKE_EXPORT_COMPILE_COMMANDS "on") # compile_commands.json作成

enable_language(CXX)
set(CMAKE_CXX_STANDARD 20)

if(${COMPILER_SUPPORTS_CXX20})
  set(CMAKE_CXX_FLAGS "-std=c++20 ${CMAKE_CXX_FLAGS}")
elseif(${COMPILER_SUPPORTS_CXX2A})
  set(CMAKE_CXX_FLAGS "-std=c++2a ${CMAKE_CXX_FLAGS}")
endif()

find_package(Threads REQUIRED)

include_directories(
  ${OpenCV_INCLUDE_DIRS} 
  ${CMAKE_CURRENT_SOURCE_DIR}
  # ${CMAKE_CURRENT_SOURCE_DIR}/h4rc/libmodbus/src
)

set(ALL_LINKS 
  Threads::Threads
)

file(GLOB H4RC_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/h4rc/*.cpp)
add_library(h4rc ${H4RC_SOURCE})
target_link_libraries(h4rc ${ALL_LINKS})
set(ALL_LINKS ${ALL_LINKS} h4rc)

file(GLOB sources ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
foreach(path IN LISTS sources)
  get_filename_component(target ${path} NAME_WE)
  message("new target : ${target}")
  add_executable(${target} ${path} )
  target_link_libraries(${target} ${ALL_LINKS})
endforeach()

