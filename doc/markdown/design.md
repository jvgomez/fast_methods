# Library design
We provide here a small overview of the software design choices.

## Criterions
- Code easy to use for the user.
- Avoid long and nested namespaces and variable names.
- Create a hierarchy that allows to reuse the code.
- Do not rely on weird C++ tricks.
- Do not rely on too many dependencies.
- Code flexibility and high performance.

## Design
- To reach high performance, `std::array` are mainly used.
- To avoid many dependencies, C++11 was chosen.
- To make the code easier to use, we only rely on Boost libraries and CImg (for visualization)
- To reuse code as much as possible, solvers are grouped depending on how they work. FMM-based solvers have many common procedures. The same as FM2-based solvers.
- Also, we tried to use the `information expert` pattern.
- In order to reach the flexibility and high performance, we chose to use a combination of static (templates) and dynamic  polymorphism. That is, all solvers inherit from the Solver class. However, Solver itself is templated depending the grid type.

This way, the compiler can carry out tons of optimizations by knowing in advance the number of dimensions on the grid. Actually, presumably the performance is the same as if the operations were hardcoded for any number of dimensions.

FMM-based solvers, following the `policies` design patter, have other parameter templates that change the behaviour. Concretely, the heap types.

The dynamic polymorphism allows to use all solvers under a common interface, as the examples included.

![Solvers hierarchy](solvers.png)

## Folder structure
+ __cmake__: cmake helper files and modules.
+ __data__: files and maps to test the code.
    + __alpha__: contains code the be refactorized and included in the library in the future.
+ __doc__: doxygen documentation.
+ __examples__: examples to understand how to use the library.
+ __include/fast_methods__: header files with most of the code, as the library is mostly templated.
    + __benchmark__: code for the benchmark tool.
    + __console__: the console class, useful for printing and parsing arguments.
    + __datastructures__: data types of the Fast Marching Method and other algorithms such as heaps or queues.
    + __fm__: Fast Methods Algorithms.
    + __fm2__: Fast Marching Square (FM2) algorithms.
    + __gradientdescent__: gradient descent implementation to compute shortests paths from the Fast Methods outputs.
    + __io__: input/output and visualization helper classes.
    + __ndgridmap__: main container and types of the Fast Methods.
    + __thirdparty__: others' software used.
    + __utils__: utility functions used in the library.
+ __scripts__: matlab scripts to parse outputs and automatic benchmarking bash script.
+ __src__: source code for non-templated classes.


