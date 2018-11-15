#
# Taken from https://git.iws.uni-stuttgart.de/dumux-repositories/dumux/blob/master/cmake/modules/FindNLOPT.cmake
#
# Module that checks whether NLOPT is available and usable.
#
# Variables used by this module which you may want to set:
# NLOPT_ROOT         Path list to search for NLOPT
#
# Sets the follwing variable:
#
# NLOPT_FOUND           True if NLOPT available and usable.
# NLOPT_INCLUDE_DIRS    Path to the NLOPT include dirs.
# NLOPT_LIBRARIES       Name to the NLOPT library.
#

# look for header files, only at positions given by the user
find_path(NLOPT_INCLUDE_DIR
  NAMES nlopt.h
  PATHS ${NLOPT_PREFIX}
        ${NLOPT_ROOT}
        "${CMAKE_SOURCE_DIR}/../external/nlopt"
  PATH_SUFFIXES "nlopt" "include/nlopt" "include" "SRC" "src" "api"
  NO_DEFAULT_PATH
)

# look for header files, including default paths
find_path(NLOPT_INCLUDE_DIR
  NAMES nlopt.h
  PATH_SUFFIXES "nlopt" "include/nlopt" "include" "SRC" "src"
)

# look for library, only at positions given by the user
find_library(NLOPT_LIBRARY
  NAMES "nlopt"
  PATHS ${NLOPT_PREFIX}
        ${NLOPT_ROOT}
        ${NLOPT_ROOT}
        "${CMAKE_SOURCE_DIR}/../external/nlopt"
  PATH_SUFFIXES "lib" "lib32" "lib64" "libnlopt" ".libs"
  NO_DEFAULT_PATH
)

# look for library files, including default paths
find_library(NLOPT_LIBRARY
  NAMES "nlopt"
  PATH_SUFFIXES "lib" "lib32" "lib64" "libnlopt"
)

# check version specific macros
include(CheckCSourceCompiles)
include(CMakePushCheckState)
cmake_push_check_state()

# we need if clauses here because variable is set variable-NOTFOUND
#
if(NLOPT_INCLUDE_DIR)
  set(CMAKE_REQUIRED_INCLUDES ${CMAKE_REQUIRED_INCLUDES} ${NLOPT_INCLUDE_DIR})
endif(NLOPT_INCLUDE_DIR)
if(NLOPT_LIBRARY)
  set(CMAKE_REQUIRED_LIBRARIES ${CMAKE_REQUIRED_LIBRARIES} ${NLOPT_LIBRARY})
endif(NLOPT_LIBRARY)

# behave like a CMake module is supposed to behave
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
  "NLOPT"
  DEFAULT_MSG
  NLOPT_INCLUDE_DIR
  NLOPT_LIBRARY
)

mark_as_advanced(NLOPT_INCLUDE_DIR NLOPT_LIBRARY)

# if both headers and library are found, store results
if(NLOPT_FOUND)
  set(NLOPT_INCLUDE_DIRS ${NLOPT_INCLUDE_DIR})
  set(NLOPT_LIBRARIES    ${NLOPT_LIBRARY})
  # log result
  file(APPEND ${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/CMakeOutput.log
    "Determining location of NLOPT succeeded:\n"
    "Include directory: ${NLOPT_INCLUDE_DIRS}\n"
    "Library directory: ${NLOPT_LIBRARIES}\n\n")
  set(NLOPT_DUNE_COMPILE_FLAGS "${NLOPT_INCLUDE_DIRS}"
    CACHE STRING "Compile flags used by DUNE when compiling NLOPT programs")
  set(NLOPT_DUNE_LIBRARIES ${NLOPT_LIBRARIES}
    CACHE STRING "Libraries used by DUNE when linking NLOPT programs")
else(NLOPT_FOUND)
  # log errornous result
  file(APPEND ${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/CMakeError.log
    "Determining location of NLOPT failed:\n"
    "Include directory: ${NLOPT_INCLUDE_DIRS}\n"
    "Library directory: ${NLOPT_LIBRARIES}\n\n")
endif(NLOPT_FOUND)
