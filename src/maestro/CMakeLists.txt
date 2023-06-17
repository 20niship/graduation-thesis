cmake_minimum_required(VERSION 3.0)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# set(CMAKE_C_COMPILER "C:/cygwin/opt/crosstool/gcc-linaro-arm-linux-gnu/bin/arm-linux-gnueabihf-gcc.exe")
# set(CMAKE_CXX_COMPILER "C:/cygwin/opt/crosstool/gcc-linaro-arm-linux-gnu/bin/arm-linux-gnueabihf-g++.exe")

# set(CMAKE_SYSTEM_NAME Arm)

# set(COMPILER_PATH /home/mech-user/Downloads/gcc-linaro-7.5.0-2019.12-x86_64_arm-linux-gnueabihf/bin)
# set(CMAKE_CXX_COMPILER ${COMPILER_PATH}/arm-linux-gnueabihf-g++)
# set(CMAKE_C_COMPILER  ${COMPILER_PATH}/arm-linux-gnueabihf-gcc) 

SET(CMAKE_SYSTEM_NAME Linux)
# check if windows
if(WIN32)
  set(CMAKE_C_COMPILER "C:/cygwin/opt/crosstool/gcc-linaro-arm-linux-gnu/bin/arm-linux-gnueabihf-gcc.exe")
  set(CMAKE_CXX_COMPILER "C:/cygwin/opt/crosstool/gcc-linaro-arm-linux-gnu/bin/arm-linux-gnueabihf-g++.exe")
else()
  # set(CMAKE_C_COMPILER /usr/bin/arm-linux-gnueabihf-gcc)
  # set(CMAKE_CXX_COMPILER /usr/bin/arm-linux-gnueabihf-g++)
  set(CMAKE_CXX_COMPILER /usr/bin/powerpc-linux-gnu-g++)
  set(CMAKE_C_COMPILER  /usr/bin/powerpc-linux-gnu-gcc)
endif()

# SET(CMAKE_C_COMPILER /usr/bin/arm-linux-gnueabihf-gcc)
# SET(CMAKE_CXX_COMPILER /usr/bin/arm-linux-gnueabihf-g++)

set(CMAKE_C_BYTE_ORDER LITTLE_ENDIAN)
set(CMAKE_CXX_BYTE_ORDER LITTLE_ENDIAN)


# set linker flags
set(CMAKE_EXE_LINKER_FLAGS  "-Wl,--hash-style=sysv" )

project(ExampleProject)

set(CMAKE_EXECUTABLE_SUFFIX ".pexe")
find_package(Threads REQUIRED)

set(EXTRA_INCLUDES 
  ${CMAKE_CURRENT_SOURCE_DIR}/GMAS-elmo-lib/includes
  ${CMAKE_CURRENT_SOURCE_DIR}/GMAS-elmo-lib/includes/CPP
  ${CMAKE_CURRENT_SOURCE_DIR}/GMAS-elmo-lib/includes/EMBL
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

include_directories(${EXTRA_INCLUDES})

set(CMAKE_CXX_STANDARD 11)

# -I"C:\cygwin\opt\crosstool\gcc-linaro-arm-linux-gnu\arm-linux-gnueabihf\libc\usr\include" 
# -I"C:\cygwin\opt\crosstool\gcc-linaro-arm-linux-gnu\arm-linux-gnueabihf\include\c++\4.7.3"
# -I"C:/GMAS/includes"
# -I"C:/GMAS/includes/cpp"
# -I../ -I"C:/cygwin/tmp"
# -I"C:/GMAS/includes/embl"

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -Wall -fmessage-length=0 -std=c++11 -g -Wno-deprecated -Wno-write-strings")

link_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/GMAS-elmo-lib/lib
  ${CMAKE_CURRENT_SOURCE_DIR}/GMAS-elmo-lib/lib/platform
)

file(GLOB MAESTRO_LIBS 
  ${CMAKE_CURRENT_SOURCE_DIR}/GMAS-elmo-lib/lib/*.a
  ${CMAKE_CURRENT_SOURCE_DIR}/GMAS-elmo-lib/lib/*.lib
  ${CMAKE_CURRENT_SOURCE_DIR}/GMAS-elmo-lib/lib/platform/*.a
  ${CMAKE_CURRENT_SOURCE_DIR}/GMAS-elmo-lib/lib/platform/*.lib
)

file(GLOB LIB_MAESTRO_SOURCES 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
)

add_library(my-maestro ${LIB_MAESTRO_SOURCES})
target_link_libraries(my-maestro Threads::Threads ${MAESTRO_LIBS})

# foreach cpp files
file(GLOB SOURCES 
  ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
)

foreach(sourcefile ${SOURCES})
  get_filename_component(name ${sourcefile} NAME)
  add_executable(${name} ${sourcefile})
  target_link_libraries(${name} my-maestro Threads::Threads)
endforeach()

