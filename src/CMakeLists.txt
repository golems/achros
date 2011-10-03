include(CheckIncludeFiles)
include(CheckFunctionExists)

# Not everybody has <ifaddrs.h> (e.g., embedded arm-linux)
CHECK_INCLUDE_FILES(ifaddrs.h HAVE_IFADDRS_H)
# Not everybody has trunc (e.g., Windows, embedded arm-linux)
CHECK_FUNCTION_EXISTS(trunc HAVE_TRUNC)

# Output test results to config.h
include_directories(${CMAKE_CURRENT_BINARY_DIR})

rosbuild_add_boost_directories()

rosbuild_add_library(achros
  achros.cpp
  )
