N-Dimensional Fast Marching Method V1.0

**Authors:**
 - Javier V. Gomez javvgomez _at_ gmail.com www.javiervgomez.com
 - Jose Pardeiro jose.pardeiro _at_ gmail.com
 - Pablo Gely

**FMM Versions:**
- Fast Marching Method (FMM) with Binary Queue and Fibonacci Queue.
- Simplified Fast Marhching Method (SFMM, FMM with a simple priority queue).
- Fast Marching Square Method (FM2).
- Fast Marching Square Star (FM2*) FM2 with CostToGo heuristics.
- Fast Marching Square Directioanl (FM2Dir)

**O(n) FMM versions:**
- Group Marching Method (GMM).
- Untidy Fast Marching Method (By Yatziv et al, circular queue).
- Fast Iterative Method (FIM).

**ROS**

ROS nodes using this code (tested in the TurtleBot) are provided in a separate repo:
https://github.com/jpardeiro/fastmarching_node

**BIICODE**
A version of this code is available at the [biicode](https://www.biicode.com) C/C++ dependency manager. Concretely
the corresponding blocks are:

https://www.biicode.com/jvgomez
jvgomez/console
jvgomez/ndgridmap
jvgomez/fmm
jvgomez/fm2examples

https://www.biicode.com/endher
endher/fm2


## DISCLAIMER and IMPORTANT NOTES

License GNU/GPL V3: http://www.gnu.org/copyleft/gpl.html

When using in 2D it works really fine. However, some problems appear for higher dimensions. I am working on that already.

This is a source code intended for my research. Although I want it to be useful for other people it is not intended to act as a library (there are many many points to improve). However, if you show interest or have feature request do not hesitate to contact me and I will try my best to improve the code for whoever needs it. I am also open to contributions and to create a formal library if necessary.

Because of this, the code is not deeply tested. I've just tested it works for the cases I need.

The compilation time is highly increased due to CImg library. Please, omit it when possible.

## Dependencies:

This code uses C\++11 so a compiler g++ 4.8 or equivalent is required. With GCC 4.7 some runtime problems can occur.

### Linking CImg dependencies.
If you want to compile code that uses the CImg library, you will need to add the following line to the CMakeLists.txt

    target_link_libraries (fmm X11 pthread)

The code provides a copy of the CImg library. This will take care of loading and savig images. Depending on the extension used, you will need to install another libraries as said in the main page of CImg: http://cimg.sourceforge.net/

The example code uses png, therefore examples of libraries to be installed are libpng, Magick++, ImageMagick, etc.

### Boost dependencies
The current version relies on header-only Boost 1.55 libraries (probably previous versions work as well). The current CMakeLists.txt assumes you have uncompressed boost into the home directory.

    include_directories (~/boost_1_55_0)

Change this line if necessary

## Documentation

To build the documentation:

    $ cd doc
    $ doxygen

Go into the HTML folder and open index.html.

## Building the code
To build the code:

    $ cd build
    $ cmake .. -DCMAKE_BUILD_TYPE=Release (Release, RelWithDebInf or Debug, Debug by default)
    $ make
    $ ./fmm -map1 ../data/testimg.png -map2 ../data/map.png -vel ../data/velocities.png

This main shows most of the utilities implemented so far.

## Folder structure

Although there are a lot of folders, they are quite simple. It is organized this way because I'm focusing on an upload to Biicode (www.biicode.com)

+ build: where the code should be compiled.
+ console: the console class.
+ data: additional files and maps to test.
+ doc: doxygen documentation.
+ examples: easy examples to understand how to use the code.
+ fm2: Fast Marching Square (FM2) algorithms.
+ fmm: Fast Marching Algorithms.
  + fmdata: classes related to the data types of the Fast Marching Method and other algorithms.
+ gradientdescent: under development.
+ io: input/output helper classes.
+ ndgridmap: main container.
+ scripts: matlab scripts to parse outputs.
+ thirdparty: others' software included.

## Known issues:

- There is not reset in the grid values if the same grid is used in Fast Marching Methods twice. Be aware of this, it can lead to wrong FMM results.


## TODO

At the top of each file there are specific TODO comments.

- Reimplement FM2* as FM2: as a high level FMM user, with underlying FMM templated.
- Better setInitialAndGoalPoints() methods for FM2 versions, and compute fmm2_sources automatically or pass them in another different method.
- Restructure the folder and the CMake files in order to properly have examples and that stuff.
- Substitute arg parsing with boost_options.
- Remove relative file dependencies (#include "../../fmm", filename = "../../data", CMakeLists.txt deps, etc).
- Implement a grid copy constructor and operator, clear, etc.
- Review code of fm2star, fm2directional and auxiliar classes (gridplotter, etc) not checked yet but it works well.
