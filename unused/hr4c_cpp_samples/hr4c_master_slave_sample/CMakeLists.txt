cmake_minimum_required (VERSION 3.5)

project(hr4c_cpp_sample)
add_definitions(-Wno-sign-compare -Wno-enum-compare)
set (CMAKE_CXX_STANDARD 17)

include_directories("$ENV{HR4C_LIBDIR}")
link_directories("$ENV{HR4C_LIBDIR}")
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)

file(GLOB_RECURSE SRC
    "${CMAKE_SOURCE_DIR}/sample.cpp"
)

add_executable(hr4c_cpp_sample ${SRC})
target_link_libraries(hr4c_cpp_sample hr4c_comm)
