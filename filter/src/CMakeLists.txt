###############################################################################
#
# ./src CMake configuration file for geopcl
#
###############################################################################

###############################################################################
# Source files specification
#
# Naming format:
#   GEOPCL_CPP - all the .cpp files
#   GEOPCL_HPP - all the .hpp files
#   GEOPCL_<dirname>_CPP - all the .cpp files for the given subdir/namespace
#   ...

set(GEOPCL_HEADERS_DIR ${CMAKE_SOURCE_DIR}/filter/include/geopcl/filter)
set(GEOPCL_HPP "")
set(GEOPCL_CPP "")

#
# base
#
set(GEOPCL_BASE_H
  ${GEOPCL_HEADERS_DIR}/clip.h
)

set(GEOPCL_BASE_HPP
  ${GEOPCL_HEADERS_DIR}/impl/clip.hpp
)

set(GEOPCL_BASE_CPP
  clip.cpp
)

list (APPEND GEOPCL_CPP ${GEOPCL_BASE_CPP} )
list (APPEND GEOPCL_HPP ${GEOPCL_BASE_H} ${GEOPCL_BASE_HPP} )

#
# Group source files for IDE source explorers (e.g. Visual Studio)
#

source_group("CMake Files" FILES CMakeLists.txt)
source_group("Header Files" FILES ${GEOPCL_BASE_H} ${GEOPCL_BASE_HPP})
source_group("Source Files" FILES ${GEOPCL_BASE_CPP})

###############################################################################
# Targets settings

set(GEOPCL_SOURCES
  ${GEOPCL_HPP}
  ${GEOPCL_CPP})

if(WIN32)
  add_definitions("-DGEOPCL_DLL_EXPORT=1")
endif()

add_library(${GEOPCL_FILTERS_LIB_NAME} STATIC ${GEOPCL_SOURCES})

target_link_libraries(${GEOPCL_FILTERS_LIB_NAME}
  ${PCL_COMMON_LIBRARY} ${PCL_IO_LIBRARY} ${PCL_FILTERS_LIBRARY})

set_target_properties(${GEOPCL_FILTERS_LIB_NAME}
  PROPERTIES SOVERSION "${GEOPCL_LIB_SOVERSION}" )

if (APPLE)
  set_target_properties(
    ${GEOPCL_FILTERS_LIB_NAME}
    PROPERTIES
    INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib")
  set_target_properties(${GEOPCL_FILTERS_LIB_NAME} PROPERTIES XCODE_ATTRIBUTE_ENABLE_OPENMP_SUPPORT YES)
endif()

###############################################################################
# Targets installation

install(TARGETS ${GEOPCL_FILTERS_LIB_NAME}
  RUNTIME DESTINATION ${GEOPCL_BIN_DIR}
  LIBRARY DESTINATION ${GEOPCL_LIB_DIR}
  ARCHIVE DESTINATION ${GEOPCL_LIB_DIR})

install(DIRECTORY ${GEOPCL_HEADERS_DIR}
  DESTINATION ${GEOPCL_INCLUDE_DIR}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")
