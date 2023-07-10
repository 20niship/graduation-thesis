cmake_minimum_required (VERSION 3.5)

project(libhr4c_core CXX C)
add_definitions(-Wno-sign-compare -Wno-enum-compare)
set (CMAKE_CXX_STANDARD 17)

include_directories("${CMAKE_SOURCE_DIR}/include")
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/cmake-build-debug)

find_package(yaml-cpp REQUIRED)
find_package(spdlog REQUIRED)

file(GLOB_RECURSE SRC
    "src/hr4c_core.cpp" "src/algorithm/*.cpp"  "src/motor_controller/*.cpp"  "src/net/*.cpp"  "src/services/*.cpp"  "src/utils/*.cpp"
)

add_library(hr4c_core SHARED ${SRC})
target_link_libraries(hr4c_core pthread)
set_property(TARGET hr4c_core PROPERTY POSITION_INDEPENDENT_CODE ON)
install(TARGETS hr4c_core LIBRARY DESTINATION ${CMAKE_SOURCE_DIR}/lib)
install(FILES include/hr4c_core.h DESTINATION ${CMAKE_SOURCE_DIR}/lib RENAME libhr4c_core.h PERMISSIONS OWNER_READ GROUP_READ WORLD_READ)
install(FILES include/motor_controller/motor_control.h DESTINATION ${CMAKE_SOURCE_DIR}/lib/motor_controller PERMISSIONS OWNER_READ GROUP_READ WORLD_READ)
install(FILES include/motor_controller/motor_state.h DESTINATION ${CMAKE_SOURCE_DIR}/lib/motor_controller PERMISSIONS OWNER_READ GROUP_READ WORLD_READ)
install(FILES include/net/udp_client.h DESTINATION ${CMAKE_SOURCE_DIR}/lib/net PERMISSIONS OWNER_READ GROUP_READ WORLD_READ)