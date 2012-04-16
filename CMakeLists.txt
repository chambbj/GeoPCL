cmake_minimum_required(VERSION 2.6)

set(INSTALL_DIR ${CMAKE_INSTALL_PREFIX})

project(LAStoPCD)

set(CMAKE_COLOR_MAKEFILE ON)

# Allow advanced users to generate Makefiles printing detailed commands
mark_as_advanced(CMAKE_VERBOSE_MAKEFILE)

# Path to additional CMake modules
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules)

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
# settings for PCL dependencies
#------------------------------------------------------------------------------

find_package(PCL 1.5 REQUIRED)
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

include_directories(${CMAKE_SOURCE_DIR}/io/include)

set(LASTOPCD lastopcd)
add_executable(${LASTOPCD} apps/LAStoPCD.cpp)
target_link_libraries(${LASTOPCD} ${PCL_LIBRARIES} ${LIBLAS_LIBRARY})

set(PCDTOLAS pcdtolas)
add_executable(${PCDTOLAS} apps/PCDtoLAS.cpp)
target_link_libraries(${PCDTOLAS} ${PCL_LIBRARIES} ${LIBLAS_LIBRARY})

install(TARGETS ${LASTOPCD} ${PCDTOLAS}
  RUNTIME DESTINATION ${INSTALL_DIR})

