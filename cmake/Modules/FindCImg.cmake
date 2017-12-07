# Taken from https://github.com/charmie11/CImg-hello-world

# - Try to find CImg lib
#
# The following variables are defined
#
#  CImg_FOUND - system has CImg lib
#  CImg_INCLUDE_DIRS - the CImg include directory
#  CImg_SYSTEM_LIBS - external libraries that CImg uses
#  CImg_SYSTEM_LIBS_DIR - external library directories
#  CImg_CFLAGS - compilation flags


find_path(CImg_INCLUDE_DIR
  NAMES CImg.h
  PATHS
    ${CMAKE_INSTALL_PREFIX}/include
    /usr/include
)
mark_as_advanced(CImg_INCLUDE_DIR)

if (CImg_INCLUDE_DIR)
  set(CImg_FOUND TRUE)
endif(CImg_INCLUDE_DIR)

# To use PKG_CHECK_MODULES to find some optional packages
find_package(PkgConfig)

# ### CIMG related stuff
# Flags to enable fast image display, using the XSHM library.
SET(CIMG_XSHM_CCFLAGS  -Dcimg_use_xshm)

# Flags to enable screen mode switching, using the XRandr library.
SET(CIMG_XRANDR_CCFLAGS  -Dcimg_use_xrandr)

# Flags to enable native support for JPEG image files, using the JPEG library.
# ( http://www.ijg.org/ )
SET(CIMG_JPEG_CCFLAGS  -Dcimg_use_jpeg)

# Flags to enable native support for TIFF image files, using the TIFF library.
# ( http://www.libtiff.org/ )
SET(CIMG_TIFF_CCFLAGS  -Dcimg_use_tiff)

# Flags to enable native support for PNG image files, using the PNG library.
# ( http://www.libpng.org/ )
SET(CIMG_PNG_CCFLAGS  -Dcimg_use_png)

#Flags to enable OPENCV support (Camera)
# ( http://www.opencv.org/ )
SET(CIMG_OPENCV_CCFLAGS -Dcimg_use_opencv)

# Flags to enable native support for EXR image files, using the OpenEXR library.
# ( http://www.openexr.com/ )
SET(CIMG_OPENEXR_CCFLAGS  -Dcimg_use_openexr)

# Flags to enable native support for various video files, using the FFMPEG library.
# ( http://www.ffmpeg.org/ )
SET(CIMG_FFMPEG_CCFLAGS  -Dcimg_use_ffmpeg)

# Flags to enable native support of most classical image file formats, using the Magick++ library.
# ( http://www.imagemagick.org/Magick++/ )
SET(CIMG_MAGICK_CCFLAGS -Dcimg_use_magick)

# Flags to enable faster Discrete Fourier Transform computation, using the FFTW3 library
# ( http://www.fftw.org/ )
SET(CIMG_FFTW3_CCFLAGS  -Dcimg_use_fftw3)

# Flags to enable zlib.
# ( http://www.zlib.net/ )
SET(CIMG_ZLIB_CCFLAGS  -Dcimg_use_zlib)

# ### Search Additional Libraries ##########
FIND_PACKAGE(OpenCV)
FIND_PACKAGE(JPEG)
FIND_PACKAGE(TIFF)
FIND_PACKAGE(PNG)
FIND_PACKAGE(ZLIB)
FIND_PACKAGE(LAPACK)
FIND_PACKAGE(BLAS)

PKG_CHECK_MODULES(FFTW3 fftw3)
PKG_CHECK_MODULES(OPENEXR OpenEXR)
PKG_CHECK_MODULES(MAGICK Magick++)

# PKG_CHECK_MODULES(LIBAVCODEC libavcodec)
# PKG_CHECK_MODULES(LIBAVFORMAT libavformat)
# PKG_CHECK_MODULES(LIBSWSCALE libswscale)
# PKG_CHECK_MODULES(LIBAVUTIL libavutil)

if(NOT WIN32)
  FIND_PACKAGE(X11)
  FIND_PACKAGE(Threads REQUIRED)
endif()

# #### End of additional libraries search ##########

### Configure Paths according to detected packages
if(TIFF_FOUND)
  get_filename_component(TIFF_LIB_DIRS ${TIFF_LIBRARIES} PATH)
  SET(CIMG_CFLAGS "${CIMG_CFLAGS} ${CIMG_TIFF_CCFLAGS}")
#  link_directories(${TIFF_LIB_DIRS})
#  include_directories(${TIFF_INCLUDE_DIR})
#  SET(CImg_SYSTEM_LIBS ${CImg_SYSTEM_LIBS} ${TIFF_LIBRARIES})
  list(APPEND CImg_INCLUDE_DIRS
    ${TIFF_INCLUDE_DIR}
  )
  list(APPEND CImg_SYSTEM_LIBS_DIR
    ${TIFF_LIB_DIRS}
  )
  list(APPEND CImg_SYSTEM_LIBS
    ${TIFF_LIBRARIES}
  )
endif()

if(JPEG_FOUND)
  get_filename_component(JPEG_LIB_DIRS ${JPEG_LIBRARIES} PATH)
  SET(CIMG_CFLAGS "${CIMG_CFLAGS} ${CIMG_JPEG_CCFLAGS}")
#  link_directories(${JPEG_LIB_DIRS})
#  include_directories(${JPEG_INCLUDE_DIR})
#  SET(CImg_SYSTEM_LIBS ${CImg_SYSTEM_LIBS} ${JPEG_LIBRARIES})
  list(APPEND CImg_INCLUDE_DIRS
    ${JPEG_INCLUDE_DIR}
  )
  list(APPEND CImg_SYSTEM_LIBS_DIR
    ${JPEG_LIB_DIRS}
  )
  list(APPEND CImg_SYSTEM_LIBS
    ${JPEG_LIBRARIES}
  )
endif()

if (ZLIB_FOUND)
  SET(CIMG_CFLAGS "${CIMG_CFLAGS} ${CIMG_ZLIB_CCFLAGS}")
#  link_directories(${ZLIB_LIB_DIRS})
#  include_directories(${ZLIB_INCLUDE_DIR})
#  SET(CImg_SYSTEM_LIBS ${CImg_SYSTEM_LIBS} ${ZLIB_LIBRARIES})
  list(APPEND CImg_INCLUDE_DIRS
    ${ZLIB_INCLUDE_DIR}
  )
  list(APPEND CImg_SYSTEM_LIBS_DIR
    ${ZLIB_LIB_DIRS}
  )
  list(APPEND CImg_SYSTEM_LIBS
    ${ZLIB_LIBRARIES}
  )
  # PNG requires ZLIB
  if(PNG_FOUND)
    SET(CIMG_CFLAGS "${CIMG_CFLAGS} ${CIMG_PNG_CCFLAGS}")
 #   link_directories(${PNG_LIB_DIRS})
 #   include_directories(${PNG_INCLUDE_DIR} )
 #   SET( CImg_SYSTEM_LIBS ${CImg_SYSTEM_LIBS} ${PNG_LIBRARIES} )
  list(APPEND CImg_INCLUDE_DIRS
    ${PNG_INCLUDE_DIR}
  )
  list(APPEND CImg_SYSTEM_LIBS_DIR
    ${PNG_LIB_DIRS}
  )
  list(APPEND CImg_SYSTEM_LIBS
    ${PNG_LIBRARIES}
  )
  endif()
endif()

if(FFTW3_FOUND)
  SET(CIMG_CFLAGS "${CIMG_CFLAGS} ${CIMG_FFTW3_CCFLAGS}")
  #link_directories( ${FFTW3_LIBRARY_DIRS} )
  #include_directories( ${FFTW3_INCLUDE_DIRS} )
  #SET( CImg_SYSTEM_LIBS ${CImg_SYSTEM_LIBS} ${FFTW3_LIBRARIES} )
  list(APPEND CImg_INCLUDE_DIRS
    ${FFTW3_INCLUDE_DIRS}
  )
  list(APPEND CImg_SYSTEM_LIBS_DIR
    ${FFTW3_LIBRARY_DIRS}
  )
  list(APPEND CImg_SYSTEM_LIBS
    ${FFTW3_LIBRARIES}
  )
endif()

if(OPENEXR_FOUND)
  SET(CIMG_CFLAGS "${CIMG_CFLAGS} ${CIMG_OPENEXR_CCFLAGS}")
  #link_directories( ${OPENEXR_LIBRARY_DIRS} )
  #include_directories( ${OPENEXR_INCLUDE_DIRS} )
  #SET( CImg_SYSTEM_LIBS ${CImg_SYSTEM_LIBS} ${OPENEXR_LIBRARIES} )
  list(APPEND CImg_INCLUDE_DIRS
    ${OPENEXR_INCLUDE_DIRS}
  )
  list(APPEND CImg_SYSTEM_LIBS_DIR
    ${OPENEXR_LIBRARY_DIRS}
  )
  list(APPEND CImg_SYSTEM_LIBS
    ${OPENEXR_LIBRARIES}
  )
endif()

if(MAGICK_FOUND)
  SET(CIMG_CFLAGS "${CIMG_CFLAGS} ${CIMG_MAGICK_CCFLAGS}")
  #link_directories( ${MAGICK_LIBRARY_DIRS} )
  #include_directories( ${MAGICK_INCLUDE_DIRS} )
  #SET( CImg_SYSTEM_LIBS ${CImg_SYSTEM_LIBS} ${MAGICK_LIBRARIES} )
  list(APPEND CImg_INCLUDE_DIRS
    ${MAGICK_INCLUDE_DIRS}
  )
  list(APPEND CImg_SYSTEM_LIBS_DIR
    ${MAGICK_LIBRARY_DIRS}
  )
  list(APPEND CImg_SYSTEM_LIBS
    ${MAGICK_LIBRARIES}
  )
endif()

if( LIBAVCODEC_FOUND  AND LIBAVFORMAT_FOUND AND LIBSWSCALE_FOUND AND LIBAVUTIL_FOUND )
  SET(CIMG_CFLAGS "${CIMG_CFLAGS} ${CIMG_FFMPEG_CCFLAGS}")
  #link_directories( ${LIBAVFORMAT_LIBRARY_DIRS} )
  #link_directories( ${LIBAVCODEC_LIBRARY_DIRS} )
  #link_directories( ${LIBSWSCALE_LIBRARY_DIRS} )
  #link_directories( ${LIBAVUTIL_LIBRARY_DIRS} )
  #include_directories( ${LIBAVFORMAT_INCLUDE_DIRS} ${LIBAVFORMAT_INCLUDE_DIRS}/libavformat)
  #include_directories( ${LIBAVCODEC_INCLUDE_DIRS} ${LIBAVCODEC_INCLUDE_DIRS}/libavcodec )
  #include_directories( ${LIBSWSCALE_INCLUDE_DIRS} ${LIBSWSCALE_INCLUDE_DIRS}/libswscale)
  #include_directories( ${LIBAVUTIL_INCLUDE_DIRS} ${LIBAVUTIL_INCLUDE_DIRS}/libavutil )
  #SET( CImg_SYSTEM_LIBS ${CImg_SYSTEM_LIBS} ${LIBAVFORMAT_LIBRARIES} )
  #SET( CImg_SYSTEM_LIBS ${CImg_SYSTEM_LIBS} ${LIBAVCODEC_LIBRARIES} )
  #SET( CImg_SYSTEM_LIBS ${CImg_SYSTEM_LIBS} ${LIBSWSCALE_LIBRARIES} )
  #SET( CImg_SYSTEM_LIBS ${CImg_SYSTEM_LIBS} ${LIBAVUTIL_LIBRARIES} )
  list(APPEND CImg_INCLUDE_DIRS
    ${LIBAVFORMAT_INCLUDE_DIRS} ${LIBAVFORMAT_INCLUDE_DIRS}/libavformat
    ${LIBAVCODEC_INCLUDE_DIRS} ${LIBAVCODEC_INCLUDE_DIRS}/libavcodec 
    ${LIBSWSCALE_INCLUDE_DIRS} ${LIBSWSCALE_INCLUDE_DIRS}/libswscale
    ${LIBAVUTIL_INCLUDE_DIRS} ${LIBAVUTIL_INCLUDE_DIRS}/libavutil 
  )
  list(APPEND CImg_SYSTEM_LIBS_DIR
    ${LIBAVFORMAT_LIBRARY_DIRS}
    ${LIBAVCODEC_LIBRARY_DIRS}
    ${LIBSWSCALE_LIBRARY_DIRS}
    ${LIBAVUTIL_LIBRARY_DIRS}
  )
  list(APPEND CImg_SYSTEM_LIBS
    ${LIBAVFORMAT_LIBRARIES}
    ${LIBAVCODEC_LIBRARIES}
    ${LIBSWSCALE_LIBRARIES}
    ${LIBAVUTIL_LIBRARIES}
  )
endif()

if(NOT APPLE)
  if(NOT WIN32)
    if(X11_FOUND)
      SET(CIMG_CFLAGS "${CIMG_CFLAGS} ${CIMG_XSHM_CCFLAGS} ${CIMG_XRANDR_CCFLAGS}")
      SET(CImg_SYSTEM_LIBS ${CImg_SYSTEM_LIBS} Xext Xrandr)
    endif()
  endif(NOT WIN32)
endif(NOT APPLE)

if(X11_FOUND)
  #link_directories(${X11_LIB_DIRS})
  #include_directories(${X11_INCLUDE_DIR})
  #SET( CImg_SYSTEM_LIBS ${CImg_SYSTEM_LIBS} ${X11_LIBRARIES} )
  list(APPEND CImg_INCLUDE_DIRS
    ${X11_INCLUDE_DIR}
  )
  list(APPEND CImg_SYSTEM_LIBS_DIR
    ${X11_LIB_DIRS}
  )
  list(APPEND CImg_SYSTEM_LIBS
    ${X11_LIBRARIES}
  )
endif()

if (NOT WIN32)
  #SET( CImg_SYSTEM_LIBS ${CImg_SYSTEM_LIBS} ${CMAKE_THREAD_LIBS_INIT} )
  list(APPEND CImg_SYSTEM_LIBS
    ${CMAKE_THREAD_LIBS_INIT} 
  )
endif()

if( WIN32)
  #SET( CImg_SYSTEM_LIBS  ${CImg_SYSTEM_LIBS}  gdi32 )
  list(APPEND CImg_SYSTEM_LIBS
    gdi32
  )
endif()

if (OpenCV_FOUND)
  message("OpenCV Found")
  SET(CIMG_CFLAGS "${CIMG_CFLAGS} ${CIMG_OPENCV_CCFLAGS}")
  #include_directories(${OpenCV_INCLUDE_DIRS})
  #link_directories(${OpenCV_LIB_DIRS})
  #SET( CImg_SYSTEM_LIBS  ${CImg_SYSTEM_LIBS}  ${OpenCV_LIBS} )
  list(APPEND CImg_INCLUDE_DIRS
    ${OpenCV_INCLUDE_DIRS}
  )
  list(APPEND CImg_SYSTEM_LIBS_DIR
    ${OpenCV_LIB_DIRS}
  )
  list(APPEND CImg_SYSTEM_LIBS
    ${OpenCV_LIBS}
  )
endif()

if(LAPACK_FOUND)
  SET(CIMG_CFLAGS "${CIMG_CFLAGS} ${CIMG_LAPACK_CCFLAGS}")
  #link_directories( ${LAPACK_LIBRARY_DIRS} )
  #include_directories( ${LAPACK_INCLUDE_DIRS} )
  #SET( CImg_SYSTEM_LIBS ${CImg_SYSTEM_LIBS} ${LAPACK_LIBRARIES} )
  list(APPEND CImg_INCLUDE_DIRS
    ${LAPACK_INCLUDE_DIRS}
  )
  list(APPEND CImg_SYSTEM_LIBS_DIR
    ${LAPACK_LIBRARY_DIRS}
  )
  list(APPEND CImg_SYSTEM_LIBS
    ${LAPACK_LIBRARIES}
  )
endif()

if(BLAS_FOUND)
  SET(CIMG_CFLAGS "${CIMG_CFLAGS} ${CIMG_BLAS_CCFLAGS}")
  #link_directories( ${BLAS_LIBRARY_DIRS} )
  #include_directories( ${BLAS_INCLUDE_DIRS} )
  #SET( CImg_SYSTEM_LIBS ${CImg_SYSTEM_LIBS} ${BLAS_LIBRARIES} )
  list(APPEND CImg_INCLUDE_DIRS
    ${BLAS_INCLUDE_DIRS}
  )
  list(APPEND CImg_SYSTEM_LIBS_DIR
    ${BLAS_LIBRARY_DIRS}
  )
  list(APPEND CImg_SYSTEM_LIBS
    ${BLAS_LIBRARIES}
  )
endif()

# Add CIMG Flags to Compilation Flags
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CIMG_CFLAGS}")

foreach(program ${CIMG_FILES})
  add_executable(${program} ${program}.cpp)
  target_link_libraries(${program} ${CImg_SYSTEM_LIBS} )
endforeach(program)
