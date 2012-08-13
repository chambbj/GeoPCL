cmake_minimum_required(VERSION 2.6)

set(INSTALL_DIR ${CMAKE_INSTALL_PREFIX})

project(GEOPCL)

set(CMAKE_COLOR_MAKEFILE ON)

# Allow advanced users to generate Makefiles printing detailed commands
mark_as_advanced(CMAKE_VERBOSE_MAKEFILE)

# Path to additional CMake modules
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules)

option(WITH_IDL "Build IDL bindings?" OFF)
mark_as_advanced(WITH_IDL)

option(WITH_P2G "Build with points2grid?" OFF)
mark_as_advanced(WITH_P2G)

#------------------------------------------------------------------------------
# installation path settings
#------------------------------------------------------------------------------

if(WIN32)
  set(DEFAULT_LIB_SUBDIR lib)
  set(DEFAULT_DATA_SUBDIR .)
  set(DEFAULT_INCLUDE_SUBDIR include)

  if (MSVC)
    set(DEFAULT_BIN_SUBDIR bin)
  else()
    set(DEFAULT_BIN_SUBDIR .)
  endif()
else()
  # Common locatoins for Unix and Mac OS X
  set(DEFAULT_BIN_SUBDIR bin)
  set(DEFAULT_LIB_SUBDIR lib)
  set(DEFAULT_DATA_SUBDIR share/blu)
  set(DEFAULT_INCLUDE_SUBDIR include)
endif()

# Locations are changeable by user to customize layout of blu installation
# (default values are platform-specific)
set(GEOPCL_BIN_SUBDIR ${DEFAULT_BIN_SUBDIR} CACHE STRING
  "Subdirectory where executables will be installed")
set(GEOPCL_LIB_SUBDIR ${DEFAULT_LIB_SUBDIR} CACHE STRING
  "Subdirectory where libraries will be installed")
set(GEOPCL_INCLUDE_SUBDIR ${DEFAULT_INCLUDE_SUBDIR} CACHE STRING
  "Subdirectory where header files will be installed")
set(GEOPCL_DATA_SUBDIR ${DEFAULT_DATA_SUBDIR} CACHE STRING
  "Subdirectory where data will be installed")

# Mark *_SUBDIR variables as advanced and dedicated to use by power-users only.
mark_as_advanced(GEOPCL_BIN_SUBDIR
  GEOPCL_LIB_SUBDIR GEOPCL_INCLUDE_SUBDIR GEOPCL_DATA_SUBDIR)

# Full paths for the installation
set(GEOPCL_BIN_DIR ${GEOPCL_BIN_SUBDIR})
set(GEOPCL_LIB_DIR ${GEOPCL_LIB_SUBDIR})
set(GEOPCL_INCLUDE_DIR ${GEOPCL_INCLUDE_SUBDIR})
set(GEOPCL_DATA_DIR ${GEOPCL_DATA_SUBDIR})

#------------------------------------------------------------------------------
# settings for Boost dependencies
#------------------------------------------------------------------------------

# Default to using static, multithreaded libraries for 
# linking under MSVC.  This is because we show users how to 
# use boostpro.com installer and install those options when linking 
# on windows in the compilation documentation.
if(WIN32)
  if (MSVC)
    set(Boost_USE_STATIC_LIBS   ON)
    set(Boost_USE_MULTITHREADED ON)
  endif(MSVC)
endif(WIN32)

find_package(Boost 1.47 REQUIRED COMPONENTS program_options filesystem system)

if(Boost_FOUND AND Boost_PROGRAM_OPTIONS_FOUND AND Boost_FILESYSTEM_FOUND AND Boost_SYSTEM_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
endif()

# make these available for the user to set.
mark_as_advanced(CLEAR Boost_INCLUDE_DIR) 
mark_as_advanced(CLEAR Boost_LIBRARY_DIRS) 
link_directories(${Boost_LIBRARY_DIRS})

#------------------------------------------------------------------------------
# settings for GDAL dependencies
#------------------------------------------------------------------------------

# GDAL/OGR support - optional, default=OFF
set(WITH_GDAL FALSE CACHE BOOL "Choose if GDAL support should be built")

if(WITH_GDAL)
  find_package(GDAL 1.9.0)
  include_directories(${GDAL_INCLUDE_DIR})
  mark_as_advanced(CLEAR GDAL_INCLUDE_DIR) 
  mark_as_advanced(CLEAR GDAL_LIBRARY)
  message(STATUS "...building with GDAL")
else()
  set(GDAL_LIBRARY "") 
endif()

#------------------------------------------------------------------------------
# settings for PCL dependencies
#------------------------------------------------------------------------------

find_package(PCL 1.6 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#------------------------------------------------------------------------------
# settings for libLAS dependencies
#------------------------------------------------------------------------------

find_package(LibLAS 1.6 REQUIRED)
include_directories(${LIBLAS_INCLUDE_DIRS})
link_directories(${LIBLAS_LIBRARY_DIRS})
add_definitions(${LIBLAS_DEFINITIONS})

#------------------------------------------------------------------------------
# settings for PDAL
#------------------------------------------------------------------------------

find_package(PDAL)
mark_as_advanced(CLEAR PDAL_INCLUDE_DIR)
mark_as_advanced(CLEAR PDAL_LIBRARIES)
include_directories(${PDAL_INCLUDE_DIR})

#------------------------------------------------------------------------------
# settings for points2grid
#------------------------------------------------------------------------------

if(WITH_P2G)
  find_package(Points2Grid)
  #mark_as_advanced(CLEAR P2G_INCLUDE_DIR)
  #mark_as_advanced(CLEAR P2G_LIBRARIES)
  include_directories(${P2G_INCLUDE_DIR})
  link_directories(${P2G_LIBRARY_DIRS})
  add_definitions(${P2G_DEFINITIONS})
endif(WITH_P2G)

#------------------------------------------------------------------------------
# settings for Eigen dependencies
#------------------------------------------------------------------------------
find_package(Eigen REQUIRED)
include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})
add_definitions(-DEIGEN_USE_NEW_STDVECTOR
                -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET)

include_directories(${CMAKE_SOURCE_DIR}/io/include)
include_directories(${CMAKE_SOURCE_DIR}/filter/include)

set(GEOPCL_FILTERS_LIB_NAME geopcl_filters)
add_subdirectory(${CMAKE_SOURCE_DIR}/filter/src)

if(WITH_IDL)
  include_directories(${IDL_INCLUDE_DIR})
  set(IDL_LIB_NAME geopcl_idl_bindings)
  add_subdirectory(${CMAKE_SOURCE_DIR}/bindings/idl)
endif(WITH_IDL)

set(LASCLIP lasclip)
add_executable(${LASCLIP} apps/clip.cpp)
target_link_libraries(${LASCLIP} ${PCL_LIBRARIES} ${LIBLAS_LIBRARY} ${GEOPCL_FILTERS_LIB_NAME})

set(LASTOPCD lastopcd)
add_executable(${LASTOPCD} apps/LAStoPCD.cpp)
target_link_libraries(${LASTOPCD} ${PCL_LIBRARIES} ${LIBLAS_LIBRARY})

set(PCDTOLAS pcdtolas)
add_executable(${PCDTOLAS} apps/PCDtoLAS.cpp)
target_link_libraries(${PCDTOLAS} ${PCL_LIBRARIES} ${LIBLAS_LIBRARY})

set(PDALTOPCD pdaltopcd)
#add_executable(${PDALTOPCD} apps/PDALtoPCD.cpp)
#target_link_libraries(${PDALTOPCD} ${PCL_LIBRARIES} ${PDAL_LIBRARY})

set (TEST test)
add_executable(${TEST} apps/test.cpp)
target_link_libraries(${TEST} ${PCL_LIBRARIES} ${LIBLAS_LIBRARY})

set(HILLSHADE test_hillshade)
add_executable(${HILLSHADE} apps/test_hillshade.cpp)

if(WITH_P2G)
  set (TESTP2G test_p2g)
  add_executable(${TESTP2G} apps/test_p2g.cpp)
  target_link_libraries(${TESTP2G} ${PCL_LIBRARIES} ${LIBLAS_LIBRARY} ${P2G_LIBRARY})
endif(WITH_P2G)

install(TARGETS
  ${LASCLIP}
  ${LASTOPCD}
  ${PCDTOLAS}
#  ${PDALTOPCD}
  ${TEST}
  ${HILLSHADE}
  ${TESTP2G}
  RUNTIME DESTINATION ${INSTALL_DIR})

