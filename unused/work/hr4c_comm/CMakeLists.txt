cmake_minimum_required (VERSION 3.5)

project(hr4c_comm)
add_definitions(-Wno-sign-compare -Wno-enum-compare)
set (CMAKE_CXX_STANDARD 17)

include_directories("${CMAKE_SOURCE_DIR}/include")
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/cmake-build-debug)

file(GLOB_RECURSE SRC
    "src/*.cpp"
)

add_library(hr4c_comm SHARED ${SRC})
target_link_libraries(hr4c_comm pthread ssh2)
set_property(TARGET hr4c_comm PROPERTY POSITION_INDEPENDENT_CODE ON)
install(TARGETS hr4c_comm LIBRARY DESTINATION ${CMAKE_SOURCE_DIR}/lib)
install(FILES src/config.yaml DESTINATION ${CMAKE_SOURCE_DIR}/lib PERMISSIONS OWNER_READ GROUP_READ WORLD_READ)
install(FILES src/zerog_mode_correction.yaml DESTINATION ${CMAKE_SOURCE_DIR}/lib PERMISSIONS OWNER_READ GROUP_READ WORLD_READ)
install(FILES include/hr4c_comm_wrapper.h DESTINATION ${CMAKE_SOURCE_DIR}/lib RENAME libhr4c_comm.h PERMISSIONS OWNER_READ GROUP_READ WORLD_READ)
