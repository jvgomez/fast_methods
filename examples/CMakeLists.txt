cmake_minimum_required (VERSION 2.6) 
project (examples) 

# Set a default build type for single-configuration
# CMake generators if no build type is set.
IF(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
   SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)

# Select flags.
SET(CMAKE_CXX_FLAGS "-std=c++11")
SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g")
SET(CMAKE_CXX_FLAGS_RELEASE "-Ofast -fno-finite-math-only")
SET(CMAKE_CXX_FLAGS_DEBUG  "-Wall -Wno-unused-local-typedefs -g")

# Self-made includes
include_directories (${EXAMPLES_SOURCE_DIR}/../console) 
include_directories (${EXAMPLES_SOURCE_DIR}/../ndgridmap) 
include_directories (${EXAMPLES_SOURCE_DIR}/../fmdata) 
include_directories (${EXAMPLES_SOURCE_DIR}/../fmm) 
include_directories (${EXAMPLES_SOURCE_DIR}/../fm2)
include_directories (${EXAMPLES_SOURCE_DIR}/../fm2directional)
include_directories (${EXAMPLES_SOURCE_DIR}/../fm2star)   
include_directories (${EXAMPLES_SOURCE_DIR}/../io) 
include_directories (${EXAMPLES_SOURCE_DIR}/../gradientdescent) 


# Third party internal includes
include_directories (${EXAMPLES_SOURCE_DIR}/../thirdparty) 


# External dependencies
include_directories (~/boost_1_55_0)

add_executable (test_fmm test_fmm.cpp
                        ../console/console.cpp
                        ../ndgridmap/cell.cpp
                        ../fmm/fmdata/fmcell.cpp
                        ) 

add_executable (test_sfmm test_fmm.cpp
                        ../console/console.cpp
                        ../ndgridmap/cell.cpp
                        ../fmm/fmdata/fmcell.cpp
                        ) 

add_executable (test_fm2 test_fm2.cpp
                        ../console/console.cpp
                        ../ndgridmap/cell.cpp
                        ../fmm/fmdata/fmcell.cpp
                        ) 
                    
add_executable (test_fm2star test_fm2star.cpp
                        ../console/console.cpp
                        ../ndgridmap/cell.cpp
                        ../fmm/fmdata/fmcell.cpp
                        ) 

add_executable (test_fm2directional test_fm2directional.cpp
                        ../console/console.cpp
                        ../ndgridmap/cell.cpp
                        ../fmm/fmdata/fmcell.cpp
                        ) 

add_executable (test_gmm test_gmm.cpp
            ../console/console.cpp
            ../ndgridmap/cell.cpp
            ../fmm/fmdata/fmcell.cpp
            ) 

add_executable (test_fim test_fim.cpp
            ../console/console.cpp
            ../ndgridmap/cell.cpp
            ../fmm/fmdata/fmcell.cpp
            ) 

add_executable (test_untidy test_untidy.cpp
            ../console/console.cpp
            ../ndgridmap/cell.cpp
            ../fmm/fmdata/fmcell.cpp
            )



