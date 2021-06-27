# The crosscompiler needs toolchain and rootfs path
# TOOLCHAIN or ENV{TOOLCHAIN} has to be the toolchain root path
# ROOTFS or ENV{ROOTFS} has to be the the / directory of an pi image

if(DEFINED ENV{ROBOTPI_TOOLCHAIN})
    set(TOOLCHAIN $ENV{ROBOTPI_TOOLCHAIN})
endif()

if(DEFINED ENV{ROBOTPI_ROOTFS})
    set(ROOTFS $ENV{ROBOTPI_ROOTFS})
endif()

set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")

set(CMAKE_C_COMPILER ${TOOLCHAIN}/bin/arm-linux-gnueabihf-gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN}/bin/arm-linux-gnueabihf-g++)

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR armv7l)
set(CMAKE_LIBRARY_ARCHITECTURE "arm-linux-gnueabihf")

set(CMAKE_FIND_ROOT_PATH ${ROOTFS})

set(CMAKE_CXX_FLAGS "--sysroot=${TOOLCHAIN}/arm-linux-gnueabihf/sysroot ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "--sysroot=${ROOTFS} ${CMAKE_CXX_FLAGS}")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

set(FOR_PI 1)
