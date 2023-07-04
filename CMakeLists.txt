cmake_minimum_required(VERSION 3.16)

option(BUILD_FOR_PI "Build the actual binary for the RPi" ON)
option(BUILD_FOR_CAM "Build the actual binary for the RPi Cam instead of test mode" ON)

if(BUILD_FOR_PI)
    set(CMAKE_TOOLCHAIN_FILE cmake/arm-none-linux-gnueabihf.toolchain.cmake)
endif()

set(CMAKE_VERBOSE_MAKEFILE OFF)

include(cmake/git_version.cmake)

project(robotpi VERSION ${GIT_VERSION})

# store GIT version info to file
configure_file(src/git_version.h.in ${CMAKE_CURRENT_SOURCE_DIR}/src/git_version.h)

set(default_build_type "Release")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
      STRING "Choose the type of build." FORCE)
endif()

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_C_FLAGS_RELEASE "-Ofast")
set(CMAKE_CXX_FLAGS_RELEASE "-Ofast")
set(CMAKE_ASM_FLAGS "-x assembler-with-cpp -c")

if(NOT CMAKE_RUNTIME_OUTPUT_DIRECTORY)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY bin)
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/Modules)

if(BUILD_FOR_PI)
    find_package(MMAL REQUIRED)
    
    add_compile_options(
        -Wall
        -fmessage-length=0
        -march=armv8-a+crc
        -mfpu=neon-fp-armv8
        -mtune=cortex-a53
        -mfloat-abi=hard
        -mhard-float
        -munaligned-access
        -mvectorize-with-neon-quad
        -funroll-loops
        -ftree-vectorize
        -funsafe-math-optimizations
        -Wno-int-in-bool-context
        -Wno-psabi
        -funroll-loops
    )
else()
    set(CMAKE_VERBOSE_MAKEFILE 1)

    add_compile_options(
            -D_USE_MATH_DEFINES
            -march=native
            -mtune=native
            -ftree-vectorize
            -funroll-loops
#            -fopt-info-vec-optimized-missed
    )
endif()

if(BUILD_FOR_CAM)
    find_package(MMAL REQUIRED)

    list(APPEND app_SOURCES
        src/rpi/RPiCamera.cpp
        src/rpi/TigerComm.cpp
        src/rpi/main.cpp
        src/rpi/RobotPi.cpp
        src/rpi/StatGenerator.cpp
        src/rpi/tests.cpp
    )
else()
    list(APPEND app_SOURCES
        src/pc/test_main.cpp
    )
endif()

list(APPEND common_SOURCES
    src/steps/CameraCalibration.cpp
    src/steps/ColorClassifierYUV.cpp
    src/steps/MiniPreview.cpp
    src/steps/RegionExtractor.cpp
    src/steps/BallLocalisation.cpp
    src/steps/RleRecorder.cpp
    src/steps/PointDistanceSensor.cpp
    src/steps/YPresummer.cpp

    src/interface/Command.cpp
    src/interface/FrameYUV420.cpp
    src/interface/ProcessingStep.cpp
    src/interface/RLEFrame.cpp

    src/util/BallTracker.cpp
    src/util/Camera.cpp
    src/util/Circle.cpp
    src/util/Color.cpp
    src/util/crc.cpp
    src/util/Line.cpp
    src/util/Presummer.cpp
    src/util/RLEFile.cpp
    src/util/TimeSync.cpp
    src/util/WorkerPool.cpp
    src/util/UDPSocket.cpp
    src/util/YUVFile.cpp
)

add_executable(robotpi ${app_SOURCES} ${common_SOURCES})

target_include_directories(robotpi PRIVATE src ${MMAL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(robotpi PRIVATE ${MMAL_LIBRARIES} pthread ${OpenCV_LIBS})

install(TARGETS robotpi DESTINATION /usr/local/bin)
install(FILES systemd/robotpi.service DESTINATION /usr/lib/systemd/system)
