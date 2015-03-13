# Overview
/** \mainpage */

These pages provide basic documentation about the Fast Marching code packages I have created.

- [Software design](markdown/design.md)
- [Detailed building](markdown/building.md)
- [Benchmarking](markdown/benchmarking.md)
- [nDGridMap implementation](../nDGridMap.pdf): Probably, the most complex part of the code. It encapsulates the complexity of the n-dimensional generalization, so that solvers are implemented almost independently of the number of dimensions.

\note Due to the nDimensional implementation, the Eikonal solver code for a given index is probably not very optimized, since it computes from 1D to nD until a better solution is not possible. For 2D and 3D (and probably nD) this is not necessary, but my tests while implementing FSM obligated me to do it that way in order to have consistent results.