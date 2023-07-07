cmake_minimum_required(VERSION 3.0)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(WIN32)
  set(CMAKE_C_COMPILER "C:/cygwin/opt/crosstool/gcc-linaro-arm-linux-gnu/bin/arm-linux-gnueabihf-gcc.exe")
  set(CMAKE_CXX_COMPILER "C:/cygwin/opt/crosstool/gcc-linaro-arm-linux-gnu/bin/arm-linux-gnueabihf-g++.exe")
else()
  set(COMPILER_PATH "/home/owner/.myenv/gcc-linaro-5.5.0-2017.10-x86_64_arm-linux-gnueabihf/bin")
  set(CMAKE_C_COMPILER   ${COMPILER_PATH}/arm-linux-gnueabihf-gcc)
  set(CMAKE_CXX_COMPILER ${COMPILER_PATH}/arm-linux-gnueabihf-g++)

  # デフォルトだとコンパイラが新しすぎてGLIBCのバージョンが対応していないので、
  # Linaro Resourcecsからダウンロードしたコンパイラ（↑)を使うこと！
  # set(CMAKE_CXX_COMPILER /usr/bin/powerpc-linux-gnu-g++)
  # set(CMAKE_C_COMPILER  /usr/bin/powerpc-linux-gnu-gcc)
endif()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(ExampleProject)

find_package(Threads REQUIRED)

set(EXTRA_INCLUDES 
  ${CMAKE_CURRENT_SOURCE_DIR}/GMAS-elmo-lib/includes
  ${CMAKE_CURRENT_SOURCE_DIR}/GMAS-elmo-lib/includes/CPP
  ${CMAKE_CURRENT_SOURCE_DIR}/GMAS-elmo-lib/includes/EMBL
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/external/spdlog/include
)

include_directories(${EXTRA_INCLUDES})

set(CMAKE_CXX_FLAGS "-D_PLATINUM -O0 -Wall -fmessage-length=0 -std=c++11 -g -Wno-deprecated -Wno-write-strings")

set(MAESTRO_LIBS 
  ${CMAKE_CURRENT_SOURCE_DIR}/GMAS-elmo-lib/lib/platinum/libMMC_APP_LIB.a
  ${CMAKE_CURRENT_SOURCE_DIR}/GMAS-elmo-lib/lib/platinum/libMMCPPLIB.a
)

if(WIN32)
  link_directories(
    c:/GMAS/lib/platinum/
    c:/cygwin/opt/crosstool/gcc-linaro-arm-linux-gnu/arm-linux-gnueabihf/lib/
    c:/cygwin/opt/crosstool/gcc-linaro-arm-linux-gnu/arm-linux-gnueabihf/lib/arm-linux-gnueabi/
    c:/cygwin/opt/crosstool/gcc-linaro-arm-linux-gnu/arm-linux-gnueabihf/libc/usr/lib/arm-linux-gnueabihf/
    c:/cygwin/opt/crosstool/gcc-linaro-arm-linux-gnu/arm-linux-gnueabihf/libc/usr/lib/
    c:/cygwin/opt/crosstool/gcc-linaro-arm-linux-gnu/arm-linux-gnueabihf/libc/lib/
  )
else()
  link_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/GMAS-elmo-lib/lib/platinum
  )
endif()

set (CMAKE_EXE_LINKER_FLAGS "-Wl,-Bstatic -lMMC_APP_LIB -lMMCPPLIB -Wl,-Bdynamic -lpthread -lrt")

set(HOST "192.168.2.52")
set(USER "user")
set(REMOTE_PATH "/mnt/jffs/usr/")

file(GLOB LIB_MAESTRO_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
add_library(mahr4c OBJECT ${LIB_MAESTRO_SOURCES})

file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
foreach(sourcefile ${SOURCES})
  get_filename_component(name ${sourcefile} NAME)
  string(REPLACE ".cpp" ".pexe" TARGET ${name})
  add_executable(${TARGET} ${sourcefile})
  target_link_libraries(${TARGET} Threads::Threads mahr4c ${MAESTRO_LIBS})
endforeach()

add_custom_target(write)
add_custom_command(
  TARGET write
  POST_BUILD
  COMMAND 
  scp ${CMAKE_CURRENT_BINARY_DIR}/*.pexe ${USER}@${HOST}:${REMOTE_PATH}
  COMMENT "Transferring via SCP"
)

