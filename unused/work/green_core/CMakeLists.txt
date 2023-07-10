cmake_minimum_required (VERSION 3.5)

project(green_core CXX C)
add_definitions(-Wno-sign-compare -Wno-enum-compare)
set (CMAKE_CXX_STANDARD 17)

include_directories("${CMAKE_SOURCE_DIR}/include")
include_directories("$ENV{HR4C_CORE_LIBDIR}")
link_directories("$ENV{HR4C_CORE_LIBDIR}")
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/cmake-build-debug)
configure_file(${CMAKE_SOURCE_DIR}/config/config.yaml ${CMAKE_BINARY_DIR} COPYONLY)

find_package(yaml-cpp REQUIRED)
find_package(spdlog REQUIRED)
find_package(LibFTDI1 REQUIRED)
include ( ${LIBFTDI_USE_FILE} )

file(GLOB_RECURSE SRC
        "src/devices/*.cpp" "src/motor_controller/*.cpp"
)

add_executable(green_core "src/main.cpp" ${SRC})
target_link_libraries(green_core PRIVATE yaml-cpp pthread hr4c_core spdlog::spdlog ${LIBFTDI_LIBRARIES})

add_executable(green_core_rt "src/main_rt.cpp" ${SRC})
target_link_libraries(green_core_rt PRIVATE yaml-cpp pthread hr4c_core spdlog::spdlog ${LIBFTDI_LIBRARIES})

# install(TARGETS hr4c_core RUNTIME DESTINATION bin)
# install(FILES config.yaml DESTINATION bin PERMISSIONS OWNER_READ GROUP_READ WORLD_READ)
