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
    # ${CMAKE_CURRENT_SOURCE_DIR}/GMAS-elmo-lib/lib
    ${CMAKE_CURRENT_SOURCE_DIR}/GMAS-elmo-lib/lib/platform
  )
endif()

set (CMAKE_EXE_LINKER_FLAGS "-Wl,-Bstatic -lMMC_APP_LIB -lMMCPPLIB -Wl,-Bdynamic -lpthread -lrt")

set(HOST "192.168.2.52")
set(USER "user")
set(PASS "user")
set(REMOTE_PATH "/mnt/jffs/usr/")

set(WRITE_TARGET "")

# add_library(my-maestro ${LIB_MAESTRO_SOURCES})
foreach(sourcefile ${SOURCES})
  get_filename_component(name ${sourcefile} NAME)
  string(REPLACE ".cpp" ".cmake" TARGET_NAME ${name})
  string(REPLACE ".cpp" ".cmake" REMOTE_TARGET_NAME ${name})

  add_library(${name} OBJECT ${sourcefile} ${LIB_MAESTRO_SOURCES})
export(
    TARGETS ${name}
    FILE ${TARGET_NAME}
)

  # target_link_libraries(${name} Threads::Threads my-maestro)
  # set_target_properties(${TARGET_NAME} PROPERTIES OUTPUT_NAME "better_name")
  # set_target_properties(${name} PROPERTIES OUTPUT_NAME ${name} SUFFIX ".pexe")
  message("new : ${TARGET_NAME}  <-- ${sourcefile}")

  add_custom_target(write-transfer-${TARGET_NAME} DEPENDS ${name})
  add_custom_target(write-${TARGET_NAME} DEPENDS write-transfer-${TARGET_NAME})
  add_custom_command(
    TARGET write-transfer-${TARGET_NAME}
    POST_BUILD
    COMMAND 
    scp ${TARGET_NAME} ${USER}@${HOST}:${REMOTE_PATH}/${TARGET_NAME}
    COMMENT "Transferring via SCP  : ${TARGET_NAME}"
  )
  add_custom_command(
    TARGET write-${TARGET_NAME}
    POST_BUILD
    COMMAND 
    ssh ${USER}@${HOST} chmod +x ${REMOTE_PATH}/${REMOTE_TARGET_NAME}
    COMMENT "chomod +x ${name}"
  )

  set(WRITE_TARGET ${WRITE_TARGET} "write-${REMOTE_TARGET_NAME}")
endforeach()

add_custom_target(write DEPENDS ${WRITE_TARGET} COMMENT "send all pexe files")

