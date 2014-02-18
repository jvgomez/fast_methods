cmake_minimum_required (VERSION 2.6) 
project (FMM) 

# Set a default build type for single-configuration
# CMake generators if no build type is set.
IF(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
   SET(CMAKE_BUILD_TYPE RelWithDebInfo)
ENDIF(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)

# Select flags.
SET(CMAKE_CXX_FLAGS "-std=c++11")
SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g")
SET(CMAKE_CXX_FLAGS_RELEASE "-O3")
SET(CMAKE_CXX_FLAGS_DEBUG  "-Wall -g")


# Self-made includes
include_directories (${FMM_SOURCE_DIR}/console) 
include_directories (${FMM_SOURCE_DIR}/ndgridmap) 
include_directories (${FMM_SOURCE_DIR}/fmdata) 
include_directories (${FMM_SOURCE_DIR}/fmm) 

# External dependencis
include_directories (~/boost_1_55_0)

# Make sure the linker can find the Hello library once it is built. 
# link_directories (${FMM_BINARY_DIR}/Hello) 

# link_directories (/home/jabuntu/boost_1_55_0/stage/lib)

add_executable (fmm main.cpp
					console/console.cpp
					ndgridmap/cell.cpp
					fmdata/fmcell.cpp
					) 

# Link the executable to the Hello library. 
#target_link_libraries (fmm Hello) 
