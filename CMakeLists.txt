##
# Corx CMake file
#
# This is a mess, I know.
##

cmake_minimum_required (VERSION 2.6)
project (cdetect)
enable_language(CXX)

# show all errors
# -fgnu89-inline: workaround for linking to argp
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -fext-numeric-literals")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")

# use c++11
add_definitions(-std=c++11)

# select the release build type by default to get optimization flags
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
    message(STATUS "Build type not specified: defaulting to release.")
endif()

include_directories("/home/sw/Nagraads/Code/Thrifty/fastdet")
include_directories("/home/sw/Nagraads/Code/Thrifty/fastcard")


# FIXME
find_library(FASTCARD NAMES fastcard
    PATHS
    /usr/lib
    /usr/local/lib
    ~/.local/lib
  )


# FIXME
find_library(FASTCARD_BASE64 NAMES Base64Functions
    PATHS
    /usr/lib
    /usr/local/lib
    ~/.local/lib
  )


# FIXME
find_library(FASTCARD_MODULES NAMES fastcard_modules
    PATHS
    /usr/lib
    /usr/local/lib
    ~/.local/lib
  )


find_library(FASTDET NAMES corr_detector
    PATHS
    /home/sw/Nagraads/Code/Thrifty/fastdet/build
    /usr/lib
    /usr/local/lib
    ~/.local/lib
  )


# FIXME: copy-pasta from fastdet
#  (fastdet should include librtlsdr automatically -- i.e. use PkgConfig)
list(APPEND CMAKE_MODULE_PATH /home/sw/Nagraads/Code/Thrifty/fastcard/cmake/Modules)  # FIXME
find_package(Threads)
find_package(LibRTLSDR)
include_directories(${LIBRTLSDR_INCLUDE_DIRS})


add_executable(corx_rx receiver.cpp sine_lookup.cpp)
target_link_libraries (corx_rx
                       ${FASTDET}
                       volk
                       ${FASTCARD}  # FIXME: dependency hell
                       ${FASTCARD_MODULES}  # FIXME: dependency hell
                       ${FASTCARD_BASE64}  # FIXME: dependency hell
                       ${CMAKE_THREAD_LIBS_INIT}
                       ${LIBRTLSDR_LIBRARIES}  # FIXME
                       fftw3f  # FIXME
                       m  # FIXME
                       )

# add install targets
install (TARGETS corx_rx DESTINATION bin)
