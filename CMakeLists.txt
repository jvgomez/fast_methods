cmake_minimum_required (VERSION 2.6) 
project (FMM) 

# set a default build type for single-configuration
# CMake generators if no build type is set.
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)

set(BUILD_EXAMPLES true CACHE STRING "True (default) to build examples")
if(BUILD_EXAMPLES)
    message(STATUS "Examples are being built (NOT installed).")
endif(BUILD_EXAMPLES)

# Select flags.
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g")
set(CMAKE_CXX_FLAGS_RELEASE "-Ofast -fno-finite-math-only")
set(CMAKE_CXX_FLAGS_DEBUG  "-Wall -Wno-unused-local-typedefs -g")

# Finding Boost
find_package(Boost REQUIRED COMPONENTS system filesystem program_options)
if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
else()
    message(FATAL_ERROR "Boost NOT FOUND. Please install it following the instructions on the README file.")
endif()

# Finding CImg
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")
find_package(CImg)
if(CImg_FOUND)
    include_directories(${CIMG_INCLUDE_DIRS})
else()
    message(FATAL_ERROR "CImg NOT FOUND. Please install it following the instructions on the README file.")
endif()

# Self-made includes
include_directories (
    ${Boost_INCLUDE_DIRS}
    ${CImg_INCLUDE_DIRS}
    console
    ndgridmap
    fmm
    fmm/fmdata
    fm2
    io
    gradientdescent
    thirdparty
)

# Create main library
add_library(fastmarching SHARED
    console/console.cpp
    ndgridmap/cell.cpp
    fmm/fmdata/fmcell.cpp
)

# Linking 
target_link_libraries(fastmarching
    ${Boost_LIBRARIES}
    ${CImg_SYSTEM_LIBS}
)

# Add benchmarking capabilities
add_executable (fmmbenchmark benchmark/fmm_benchmark.cpp)  

target_link_libraries (fmmbenchmark fastmarching)

# Create main example
if(BUILD_EXAMPLES)
    add_executable(fmm main.cpp)
    target_link_libraries(fmm fastmarching)
    add_subdirectory(examples)
endif(BUILD_EXAMPLES)

# Install section
# We are not using CMakePackageConfigHelpers because CIMG variables are lists
# and those functions do not manage them properly.
configure_file(
    cmake/FMConfig.cmake.in
    ${CMAKE_BINARY_DIR}/FMConfig.cmake
)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/FMConfig.cmake
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/cmake/FM
)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/FMConfig.cmake
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/cmake/FM
)

install(TARGETS fastmarching
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

install(TARGETS fmmbenchmark
    DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
)

install(DIRECTORY scripts DESTINATION ${CMAKE_INSTALL_PREFIX}/share/FM)

install(DIRECTORY benchmark/cfg DESTINATION ${CMAKE_INSTALL_PREFIX}/share/FM)

install(DIRECTORY ${CMAKE_SOURCE_DIR} DESTINATION ${CMAKE_INSTALL_PREFIX}/include
    FILES_MATCHING
        PATTERN "*.hpp"
        PATTERN "*.h"
    REGEX "/build$" EXCLUDE
    REGEX ".git$" EXCLUDE
    REGEX "/examples$" EXCLUDE
    REGEX "/doc$" EXCLUDE
    REGEX "/data$" EXCLUDE
    REGEX "/cfg$" EXCLUDE
    REGEX "/cmake$" EXCLUDE
    REGEX "/scripts$" EXCLUDE
)


# Uninstall target
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
