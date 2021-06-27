cmake_minimum_required(VERSION 3.0)
project(robotpi)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(NOT CMAKE_RUNTIME_OUTPUT_DIRECTORY)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY bin)
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/Modules)
list(APPEND CMAKE_PREFIX_PATH /opt/vc)

if(DEFINED FOR_PI)
    file(GLOB_RECURSE ADDITIONAL_SOURCES "src/rpi/*.cpp" "src/RobotPi.cpp" "src/tests.cpp" "src/main.cpp")
    find_package(MMAL REQUIRED)
    
    add_compile_options(
        -std=c++17
        -Wall
        -c
        -fmessage-length=0
        -march=armv8-a+crc
        -mfpu=neon-fp-armv8
        -mtune=cortex-a53
        -mvectorize-with-neon-quad
        -ftree-vectorize
        -Wno-int-in-bool-context
        -Wno-psabi
    )
else()
    # If cmake doesn't find your OpenCV_DIR, add -DOpenCV_DIR=[path/to/opencv] to your cmake command

    file(GLOB_RECURSE ADDITIONAL_SOURCES "src/pc/*.cpp")
    find_package(OpenCV REQUIRED)
endif()

file(GLOB_RECURSE SOURCES "src/detector/*.cpp" "src/interface/*.cpp" "src/util/*.cpp")

SET(CMAKE_CXX_FLAGS_RELEASE "-Ofast")

add_executable(robotpi ${SOURCES} ${ADDITIONAL_SOURCES})

target_include_directories(robotpi PRIVATE src ${MMAL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(robotpi PRIVATE ${MMAL_LIBRARIES} pthread ${OpenCV_LIBS})
