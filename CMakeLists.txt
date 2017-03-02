##
# Corx CMake file
#
# This is a mess, I know.
##

cmake_minimum_required (VERSION 2.6)
project (corx)
enable_language(CXX)

# show all errors
add_definitions(-Wall)
add_definitions(-Wextra)
add_definitions(-Wno-unused-parameter)
add_definitions(-Wno-unused)
add_definitions(-Wsign-compare)
# add_definitions(-fext-numeric-literals)

# use c++11
add_definitions(-std=c++11)

# select the release build type by default to get optimization flags
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
    message(STATUS "Build type not specified: defaulting to release.")
endif()

# find dependencies
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/Modules)
find_package(PkgConfig)
find_package(Fastdet)
find_package(Fastcard)
find_package(Volk)

if(NOT FASTDET_FOUND)
    message(FATAL_ERROR "Fastdet required to compile corx")
endif()

include_directories(
    ${FASTDET_INCLUDE_DIRS}
    ${FASTCARD_INCLUDE_DIRS}
    ${VOLK_INCLUDE_DIRS}
)

add_executable(corx_rx receiver.cpp sine_lookup.cpp)
target_link_libraries (corx_rx
                       ${FASTDET_LIBRARIES}
                       ${FASTCARD_LIBRARIES}
                       ${VOLK_LIBRARIES}
                       m)

# add install targets
install (TARGETS corx_rx DESTINATION bin)
