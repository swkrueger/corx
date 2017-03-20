find_package(PkgConfig)
pkg_check_modules (PC_FASTDET librtlsdr)

find_path(
    FASTDET_INCLUDE_DIRS
    NAMES fastdet/corr_detector.h
    HINTS ${PC_FASTDET_INCLUDE_DIRS}
    PATHS /usr/include
          /usr/local/include
)

find_library(
    FASTDET_LIBRARIES
    NAMES fastdet
    HINTS ${PC_FASTDET_LIBRARY_DIRS}
    PATHS /usr/lib
          /usr/local/lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(FASTDET DEFAULT_MSG
                                  FASTDET_LIBRARIES FASTDET_INCLUDE_DIRS)

mark_as_advanced(FASTDET_LIBRARIES FASTDET_INCLUDE_DIRS)
