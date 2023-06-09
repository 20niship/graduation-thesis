cmake_minimum_required(VERSION 3.0)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# set(CMAKE_C_COMPILER "C:/cygwin/opt/crosstool/gcc-linaro-arm-linux-gnu/bin/arm-linux-gnueabihf-gcc.exe")
# set(CMAKE_CXX_COMPILER "C:/cygwin/opt/crosstool/gcc-linaro-arm-linux-gnu/bin/arm-linux-gnueabihf-g++.exe")
set(CMAKE_SYSTEM_NAME Linux)

set(COMPILER_PATH /home/mech-user/Downloads/gcc-linaro-7.5.0-2019.12-x86_64_arm-linux-gnueabihf/bin)
set(CMAKE_CXX_COMPILER ${COMPILER_PATH}/arm-linux-gnueabihf-g++)
set(CMAKE_C_COMPILER  ${COMPILER_PATH}/arm-linux-gnueabihf-gcc) 

project(ExampleProject)

set(EXTRA_INCLUDES 
  ${CMAKE_CURRENT_SOURCE_DIR}/GMAS-elmo-lib/includes
  ${CMAKE_CURRENT_SOURCE_DIR}/GMAS-elmo-lib/includes/CPP
  ${CMAKE_CURRENT_SOURCE_DIR}/GMAS-elmo-lib/includes/EMBL
  ${CMAKE_CURRENT_SOURCE_DIR}
)

include_directories(${EXTRA_INCLUDES})

set(CMAKE_CXX_STANDARD 11)

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/test_project2.cpp
)

add_executable(example ${SOURCES})
link_directories(path/to/lib_folder)

file(GLOB LIB_SOURCES 
  ${CMAKE_CURRENT_SOURCE_DIR}/GMAS-elmo-lib/lib/*.a
  ${CMAKE_CURRENT_SOURCE_DIR}/GMAS-elmo-lib/lib/*.lib
)
target_link_libraries(example ${LIB_SOURCES})



