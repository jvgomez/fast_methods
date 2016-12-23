N-Dimensional Fast Marching Method V1.0

**Authors:**
 - [Javier V. Gomez](http://jvgomez.github.io) javvgomez _at_ gmail.com
 - Jose Pardeiro jose.pardeiro _at_ gmail.com
 - Pablo Gely

## ALGORITHMS
**Fast Marching Methods:**
- [FMM](http://jvgomez.github.io/fastmarching/classFMM.html): Fast Marching Method with Binary Queue and Fibonacci Queue (binary by default).
- [FMM*](http://jvgomez.github.io/fastmarching/classFMMStar.html): FMM with CostToGo heuristics.
- [SFMM](http://jvgomez.github.io/fastmarching/classSFMM.html): Simplified Fast Marhching Method.
- [SFMM*](http://jvgomez.github.io/fastmarching/classSFMMStar.html): SFMM with CostToGo heuristics..

**O(n) Fast Marching Methods:**
- [GMM](http://jvgomez.github.io/fastmarching/classGMM.html): Group Marching Method.
- [UFMM](http://jvgomez.github.io/fastmarching/classUFMM.html): Untidy Fast Marching Method.
- [FIM](http://jvgomez.github.io/fastmarching/classFIM.html): Fast Iterative Method.

**Fast Sweeping Methods:**
- [FSM](http://jvgomez.github.io/fastmarching/classFSM.html): Fast Sweeping Method.
- [LSM](http://jvgomez.github.io/fastmarching/classLSM.html): Lock Sweeping Method.

**Other methods:**
- [DDQM](http://jvgomez.github.io/fastmarching/classDDQM.html): Dynamic Double Queue Method.

**Fast Marching Square motion planning algorithms:**
- [FM2](http://jvgomez.github.io/fastmarching/classFM2.html): Fast Marching Square Method.
- [FM2*](http://jvgomez.github.io/fastmarching/classFM2Star.html): Fast Marching Square Star FM2 with CostToGo heuristics.

**ROS**

ROS nodes using this code (tested in the TurtleBot) are provided in a separate repo:
https://github.com/jpardeiro/fastmarching_node

## DISCLAIMER and IMPORTANT NOTES

- The code is not deeply tested. I've just tested it works for the cases I need. If you find any problem, have any question (or whatever), please write to: javvgomez _at_ gmail.com

- The compilation time is highly increased due to CImg library. Please, omit it when possible.

- License GNU/GPL V3: http://www.gnu.org/copyleft/gpl.html

- This is a source code intended for my research. Although I want it to be useful for other people it is not intended to act as a library (there are many many points to improve). However, if you show interest or have feature request do not hesitate to contact me and I will try my best to improve the code for whoever needs it. I am also open to contributions and to create a formal library if necessary.


## Documentation

- [API Reference](jvgomez.github.io/fastmarching/)
- This README is important as well.


To build latest the documentation:

    $ cd doc
    $ doxygen

Go into the HTML folder and open index.html


## Dependencies:

This code uses C\++11 so a compiler g++ 4.8 or equivalent is required. With GCC 4.7 some runtime problems can occur.

Additional dependencies: Boost, imagemafick and CImg.

    $ sudo apt-get install imagemagick libboost-all-dev cimg-dev

## Building the code
To build the code:

    $ cd build
    $ cmake .. -DCMAKE_BUILD_TYPE=Release (Release, RelWithDebInf or Debug, Release by default)
    $ make -j8 (or the number of cores you want to use during compilation)
    $ sudo make install (only if you want to install the library)
    $ ./fmm -map1 ../data/testimg.png -map2 ../data/map.png -vel ../data/velocities.png

This main shows most of the utilities implemented so far.

To uninstall:

    $ sudo make uninstall


## Folder structure

Although there are a lot of folders, they are quite simple. It is organized this way because I'm focusing on an upload to Biicode (www.biicode.com)

+ build: where the code should be compiled.
+ console: the console class.
+ data: additional files and maps to test.
  + alpha: contains code the be refactorized and included in the library in the future.
+ doc: doxygen documentation.
+ examples: easy examples to understand how to use the code.
+ fm2: Fast Marching Square (FM2) algorithms.
+ fmm: Fast Marching Algorithms.
  + fmdata: classes related to the data types of the Fast Marching Method and other algorithms.
+ gradientdescent: under development.
+ io: input/output helper classes.
+ ndgridmap: main container.
+ scripts: matlab scripts to parse outputs and automatic benchmarking bash script.
+ thirdparty: others' software included.

### Installation
Default installation folder is` /usr/local` for Linux distributions (or equivalent on other OS). Libraries are installed in subfolder `lib`, together with CMake modules. All the includes are installed under `include` subfolder. Additionally, in the `share` subfolder the helper scripts and some samples of benchmark configurations files are installed. Finally, the benchmark binary is installed in `bin`.

In order to change the default installation folder you can do:

	$ cmake ../.. -DCMAKE_INSTALL_PREFIX=<your folder>


In this case, you will have to adapt your environment variables so that the libraries and CMake modules are found.


## KNOWN ISSUES

- Gradient Descent for FM2* could fail if very narrow passages are in the way of the path.
- It seems that UFMM can fail in maps with random (or similar) velocity changes.

## TODO

At the top of each file there are specific TODO comments. Here are some others.

### Code TODOs
- Remove relative file dependencies (#include "../../fmm", filename = "../../data", CMakeLists.txt deps, etc).
- MapLoader, GridWriter... get a naming convention. MapReader or GridSaver for instance.
- printRunInfo implementation missing for most of the solvers.
- Unify GridWriter and GridPlotter functions parameter order: (grid, name)
- Fix Doxygen warnings.
- Convert all scripts to python (or similar) so that they keep completely open source.
- Reimplement FM2Dir from scratch. Currently in data/alpha folder.
- Substitute arg parsing with boost_options.
- Implement a grid copy constructor and assignment operator, etc.
- Most of the unsigned int should be replaced by size_t to be type-correct.
- Use smart pointers (specially for grid).
- Create a testing framework.
- BenchmarkCFG::configure, parse ctorParams_ with a variadic templated function, something like:` parseParams<int, bool>(param1, param2)`, `parseParams<string,double,bool>(p1,p2,p3)`.
- GridPlotter code can be refactorized so that the same code is not repeated many times.
- Review template template parameters, perhaps it can be simplified (specially for benchmark): `template <grid_t>` to `template <nDGridMap <cell_t, ndims>>` so we can use cell_t without doing `template <grid_t, cell_t>` (redundant).
- Unify names: one thing is Fast Methods, other Fast Marching, etc.

### Algorithmic TODOs
- Improve untidy queue implementation with hash maps (specially remove element in increase_priority()).
- Mix SFMM and UFMM (researchy TODO).
- Improve the way FM2 and its versions deal with the grid when running multiple times on the same grid. Concretely, avoid recomputation of velocities map.
- For most methods, neighbors for same cell are computed many times. Store them like FMT to save some computation time.

### Documentation TODOs
- Review and update nDGridMap.pdf
- Link thesis and papers.
