# Export all of the compile commands so I can use them in vim
SET( CMAKE_EXPORT_COMPILE_COMMANDS 1 )

CMAKE_MINIMUM_REQUIRED(VERSION 2.8.0 FATAL_ERROR)

# Revision1 project
PROJECT(realsense_test)

FIND_PACKAGE(OpenCV REQUIRED)

# Manually include the librealsense libraries and headers
FILE(GLOB Realsense_LIBS "/usr/local/lib/librealsense2.so")
INCLUDE_DIRECTORIES("/usr/local/include/librealsense2/")

# Compile source files
FILE(GLOB SOURCES "src/*.cpp")
INCLUDE_DIRECTORIES("./include")

# Use c++11 and clang
SET (CMAKE_CXX_COMPILER "/usr/bin/clang++")
SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 ")

# Make all of the sources executable
ADD_EXECUTABLE(realsense_test ${SOURCES})

# Link libraries into the executable
TARGET_LINK_LIBRARIES(realsense_test ${OpenCV_LIBS} ${Realsense_LIBS})
