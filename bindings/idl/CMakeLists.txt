set(IDL_SOURCES
  idl_test.cpp
  idl_hull.cpp
 )

add_library(${IDL_LIB_NAME} SHARED ${IDL_SOURCES})
target_link_libraries(${IDL_LIB_NAME} ${PCL_LIBRARIES} ${LIBLAS_LIBRARY})
