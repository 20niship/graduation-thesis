cmake_minimum_required(VERSION 3.0)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

SET(CMAKE_SYSTEM_NAME Linux)
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

set(CMAKE_C_BYTE_ORDER LITTLE_ENDIAN)
set(CMAKE_CXX_BYTE_ORDER LITTLE_ENDIAN)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXE_LINKER_FLAGS  "-Wl,--hash-style=sysv" )

project(ExampleProject)

set(CMAKE_EXECUTABLE_SUFFIX ".pexe")
find_package(Threads REQUIRED)

set(EXTRA_INCLUDES 
  ${CMAKE_CURRENT_SOURCE_DIR}/GMAS-elmo-lib/includes
  ${CMAKE_CURRENT_SOURCE_DIR}/GMAS-elmo-lib/includes/CPP
  ${CMAKE_CURRENT_SOURCE_DIR}/GMAS-elmo-lib/includes/EMBL
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/external/spdlog/include
)

include_directories(${EXTRA_INCLUDES})

set(CMAKE_CXX_FLAGS "-D_PLATINUM -O0 -c -Wall -fmessage-length=0 -std=c++11 -g -Wno-deprecated -Wno-write-strings")

file(GLOB MAESTRO_LIBS 
  ${CMAKE_CURRENT_SOURCE_DIR}/GMAS-elmo-lib/lib/*.a
  ${CMAKE_CURRENT_SOURCE_DIR}/GMAS-elmo-lib/lib/*.lib
  ${CMAKE_CURRENT_SOURCE_DIR}/GMAS-elmo-lib/lib/platform/*.a
  ${CMAKE_CURRENT_SOURCE_DIR}/GMAS-elmo-lib/lib/platform/*.lib
)

file(GLOB LIB_MAESTRO_SOURCES 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
)
  
# add_library(my-maestro ${LIB_MAESTRO_SOURCES})
# target_link_libraries(my-maestro Threads::Threads)

file(GLOB SOURCES 
  ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
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
    ${CMAKE_CURRENT_SOURCE_DIR}/GMAS-elmo-lib/lib
    ${CMAKE_CURRENT_SOURCE_DIR}/GMAS-elmo-lib/lib/platform
  )
endif()

set (CMAKE_EXE_LINKER_FLAGS "-Wl,-Bstatic -lMMC_APP_LIB -lMMCPPLIB -Wl,-Bdynamic -lpthread -lrt")

foreach(sourcefile ${SOURCES})
  get_filename_component(name ${sourcefile} NAME)
  add_executable(${name} ${sourcefile} ${LIB_MAESTRO_SOURCES})
  # target_link_libraries(${name} Threads::Threads ${MAESTRO_LIBS})
  add_custom_command(TARGET ${name} POST_BUILD
  COMMAND 
    ${CMAKE_CXX_COMPILER} -otest_project2.pexe 
    CMakeFiles/test_project2.cpp.dir/test_project2.cpp.o 
    CMakeFiles/test_project2.cpp.dir/src/common.cpp.o 
    CMakeFiles/test_project2.cpp.dir/src/TorqueControl.cpp.o 
    -L"${CMAKE_CURRENT_SOURCE_DIR}/GMAS-elmo-lib/lib/platinum"
    # -LC:/GMAS/lib/platinum/ 
    -Wl,-Bstatic -lMMC_APP_LIB -lMMCPPLIB -Wl,-Bdynamic -lpthread -lrt
  COMMENT "Linking to pexe"
  )
endforeach()

# make writeするとscpしてデータを送信してくれる
add_custom_target(write-transfer DEPENDS test_project2.pexe)
add_custom_target(write DEPENDS write-transfer)
set(HOST "192.168.2.52")
set(USER "user")
set(PASS "user")
set(REMOTE_PATH "/mnt/jffs/usr/main.pexe")

# ファイルを転送するためのカスタムコマンド
add_custom_command(
  TARGET write-transfer
  POST_BUILD
  COMMAND 
   scp test_project2.pexe ${USER}@${HOST}:${REMOTE_PATH}
  COMMENT "Transferring file via SCP"
)

add_custom_command(
  TARGET write-transfer
  POST_BUILD
  COMMAND 
   ssh ${USER}@${HOST} chmod +x ${REMOTE_PATH}
  COMMENT "Transferring file via SCP"
)

