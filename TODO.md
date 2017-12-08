# TODO

At the top of each file there are specific TODO comments. (List [here](http://jvgomez.github.io/fast_methods/todo.html)). Here are some others.

## Code TODOs
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
- Add proper versioning in CMake.
- use Git LFS for big files (and clean the experiments branch).

## Algorithmic TODOs
- Improve untidy queue implementation with hash maps (specially remove element in increase_priority()).
- Mix SFMM and UFMM (researchy TODO).
- Improve the way FM2 and its versions deal with the grid when running multiple times on the same grid. Concretely, avoid recomputation of velocities map.
- For most methods, neighbors for same cell are computed many times. Store them like FMT to save some computation time.

## Documentation TODOs
- Review and update nDGridMap.pdf
- Add a make doc target.
