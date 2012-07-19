# LAS Clip

User specifies path and filename of an LAS file to clip spatially. File processes (out-of-core, if necessary) creating a new filename, as specified by the user.

## Pseudocode

### In memory

'''
#include <geopcl/LAStoPCD.h>

#include <pcl/common/common.h>

// read LAS file from disk

// call getPointsInBox() and ExtractIndices
'''

### Out-of-core

'''
#include <liblas/liblas.hpp>

// read chunk

// determine points to keep

// continue until all chunks processed

// write filtered file
'''

# Extract Indices for LAS

User imports XYZ(I) from LAS file and proceeds with filtering, storing indices to be extracted. User calls ExtractIndicesLAS() to create new, filtered LAS file. This could be implemented by adding an optional indices input, or setIndices method, to PCDtoLAS(). In this scenario, the file is loaded, perhaps by LAStoPCD, processed (tagging points to extract), and then only writing the selected indices in a subsequent call to PCDtoLAS.
