N-Dimensional Fast Marching Method V1.0

Authors: 
 - Javier V. Gomez javvgomez __at__ gmail.com www.javiervgomez.com
 - Jose Pardeiro jose.pardeiro __at__ gmail.com


===========================
DISCLAIMER and IMPORTANT NOTES

License GNU/GPL V3.

IMPORTANT NOTES:

When using in 2D it works really fine. However, some problems appear for higher dimensions. I am working on that already.

This is a source code intended for my research. Although I want it to be useful for other people it is not intended to act as a library (there are many many points to improve). However, if you show interest or have feature request do not hesitate to contact me and I will try my best to improve the code for whoever needs it. I am also open to contributions and to create a formal library if necessary.

Because of this, the code is not deeply tested. I've just tested it works for the cases I need.

The compilation time is highly increased due to CImg library. Please, omit it when possible. When doing this, you can erase the following lines of the CMakeLists.txt:

# Linking CImg dependencies.
target_link_libraries (fmm X11 pthread)

The CImg dependency is included in the thirdparty folder. For more information:
http://cimg.sourceforge.net/
===========================

===========================
Dependencies:

This code uses C++11 so a compiler g++ 4.8 or equivalent is required. With GCC 4.7 some runtime problems can occur.

The current version relies on Boost 1.55 (probably previous versions work as well). The current CMakeLists.txt assumes you have uncompressed boost into the home directory.

# External dependencies
include_directories (~/boost_1_55_0)

Change this line if necessary


CImg dependencies:
The code provides a copy of the CImg library. This will take carge of loading and savig images. Depending on the extension used, you will need to install another libraries as said in the main page of CImg: http://cimg.sourceforge.net/

The example code uses png, therefore examples of libraries to be installed are libpng, Magick++, ImageMagick, etc.

===========================

===========================
To build the documentation:

$ cd doc
$ doxygen

Go into the HTML folder and open index.html.
===========================

===========================
To build the code:

$ cd build
$ cmake .. -DCMAKE_BUILD_TYPE=Release (Release, RelWithDebInf or Debug, Debug by default)
$ make
$ ./fmm -map1 ../data/testimg.png -map2 ../data/map.png -vel ../data/velocities.png

This main shows most of the utilities implemented so far.
===========================


===========================
Folder structure:

Although there are a lot of folders, they are quite simple. It is organized this way because I'm focusing on an upload to Biicode (www.biicode.com)

+ build: where the code should be compiled.
+ console: the console class.
+ data: additional files and maps to test.
+ doc: doxygen documentation.
+ fmdata: clases related to the data types of the Fast Marching Method.
+ fmm: Fast Marching Algorithms.
+ gradientdescent: under development.
+ io: input/output helper classes.
+ ndgridmap: main container.
+ scripts: matlab scripts to parse outputs.
+ thirdparty: others' software included.

===========================


===========================
Known issues:

- There is not reset in the grid values if the same grid is used in Fast Marching Methods twice. Be aware of this, it can lead to wrong FMM results.

===========================



===========================
TODO:

In the top part of each file there are specific TODO comments.

- Restructure the folder and the CMake files in order to properly have examples and that stuff.

- Substitute arg parsing with boost_options.

fm2_algs branch:

- remove the mandatory parameter in computeFM methods.
- check function naming.
- provide default arguments for examples.
- Remove relative file dependencies (#include "../../fmm", filename = "../../data", CMakeLists.txt deps, etc).

===========================
