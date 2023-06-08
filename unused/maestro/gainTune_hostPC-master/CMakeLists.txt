SET(execName test)
cmake_minimum_required(VERSION 2.8)
project(${execName})

if (COMMAND cmake_policy) # come from zed svo
    cmake_policy(SET CMP0003 OLD)
    cmake_policy(SET CMP0015 OLD)
endif (COMMAND cmake_policy)

#find_package(ZED 2 REQUIRED) # from ZED
#find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED) # from ZED

#include_directories(${CUDA_INCLUDE_DIRS})  # from ZED
#include_directories(${ZED_INCLUDE_DIRS}) # from ZED
#include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include) # from ZED

#set(CMAKE_CXX_FLAGS "-g -Wall")# from ZED
FILE(GLOB_RECURSE SRC_FILES src/*.cpp)# encoder
#FILE(GLOB_RECURSE HDR_FILES include/*.hpp) # from ZED and etc

add_definitions(-std=c++11 -g -O3) # from ZED

add_executable(${execName} ${SRC_FILES} ${HDR_FILES})
target_link_libraries(${execName} usb usb-1.0 modbus boost_system boost_filesystem modbus) # from encoder
#target_link_libraries(${execName}
#        ${ZED_LIBRARIES}
#        ${SPECIAL_OS_LIBS}
#        ${CUDA_LIBRARIES} ${CUDA_npps_LIBRARY} ${CUDA_NPP_LIBRARIES_ZED}
#        boost_system boost_filesystem modbus
#        ) # ZED
