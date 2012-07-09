set(IDL_SOURCES
  idlConcaveHull.cpp
  idlConvexHull.cpp
  idlLAStoPCD.cpp
 )

add_library(${IDL_LIB_NAME} SHARED ${IDL_SOURCES})
target_link_libraries(${IDL_LIB_NAME} ${PCL_LIBRARIES} ${LIBLAS_LIBRARY})
