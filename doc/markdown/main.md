These pages provide basic documentation about the Fast Methods library.

- [Software design](markdown/design.md)
- [Building and installing](markdown/building.md)
- [Benchmarking](markdown/benchmarking.md)
- [nDGridMap implementation](ndgridmap.pdf): Probably, the most complex part of the code. It encapsulates the complexity of the n-dimensional generalization, so that solvers are implemented almost independently of the number of dimensions.

\note Some of the classes are just wrappers that can be used with lower level classes. For instance, declaring a `SFMM<nDGridMap<FMCell,2>>` is the same as using `FMM<nDGridMap<FMCell,2>, FMPriorityQueue<FMCell>>`. The same happens with the *(Star) solvers.

\note Due to the nDimensional implementation, the Eikonal solver code for a given index is probably not very optimized, since it computes from 1D to nD until a better solution is not possible. For 2D and 3D (and probably nD) this is not necessary, but my tests while implementing FSM obligated me to do it that way in order to have consistent results.


## ALGORITHMS
All the theory and algorithms implemented in this library can be found in my [PhD thesis](phd_thesis.pdf). In fact, all the benchmarking data in chapter 4 has been produced with this library and it is stored on the `experiments` branch.


**Fast Marching Methods:**
- [FMM](http://jvgomez.github.io/fast_methods/classFMM.html): Fast Marching Method with Binary Queue and Fibonacci Queue (binary by default).
- [FMM*](http://jvgomez.github.io/fast_methods/classFMMStar.html): FMM with CostToGo heuristics.
- [SFMM](http://jvgomez.github.io/fast_methods/classSFMM.html): Simplified Fast Marhching Method.
- [SFMM*](http://jvgomez.github.io/fast_methods/classSFMMStar.html): SFMM with CostToGo heuristics..

**O(n) Fast Marching Methods:**
- [GMM](http://jvgomez.github.io/fast_methods/classGMM.html): Group Marching Method.
- [UFMM](http://jvgomez.github.io/fast_methods/classUFMM.html): Untidy Fast Marching Method.
- [FIM](http://jvgomez.github.io/fast_methods/classFIM.html): Fast Iterative Method.

**Fast Sweeping Methods:**
- [FSM](http://jvgomez.github.io/fast_methods/classFSM.html): Fast Sweeping Method.
- [LSM](http://jvgomez.github.io/fast_methods/classLSM.html): Lock Sweeping Method.
- [DDQM](http://jvgomez.github.io/fast_methods/classDDQM.html): Dynamic Double Queue Method.

**Fast Marching Square motion planning algorithms:**
- [FM2](http://jvgomez.github.io/fast_methods/classFM2.html): Fast Marching Square Method.
- [FM2*](http://jvgomez.github.io/fast_methods/classFM2Star.html): Fast Marching Square Star FM2 with CostToGo heuristics.

## Authors
 - [Javier V. Gomez](http://jvgomez.github.io) javvgomez _at_ gmail.com
 - Jose Pardeiro jose.pardeiro _at_ gmail.com
 - Pablo Gely