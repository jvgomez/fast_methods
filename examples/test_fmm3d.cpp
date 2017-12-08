/* An example to run FMM on a 3D grid loaded from a given text file. */

#include <iostream>
#include <array>

#include <fast_methods/ndgridmap/fmcell.h>
#include <fast_methods/ndgridmap/ndgridmap.hpp>

#include <fast_methods/fm/fmm.hpp>
#include <fast_methods/io/maploader.hpp>
#include <fast_methods/io/gridwriter.hpp>

using namespace std;
using namespace std::chrono;

int main(int argc, char **argv)
{
    // A bit of shorthand.
    typedef nDGridMap<FMCell, 3> FMGrid3D;
    typedef array<unsigned int, 3> Coord3D;

    // Grid, start and goal definition.
    FMGrid3D grid_fmm;
    MapLoader::loadMapFromText(argv[1], grid_fmm);
    Coord3D init_point = {5, 5, 5};

    // Solvers declaration.
    std::vector<Solver<FMGrid3D>*> solvers;
    solvers.push_back(new FMM<FMGrid3D>);

    // Executing every solver individually over the same grid.
    for (Solver<FMGrid3D>* s :solvers)
    {
        s->setEnvironment(&grid_fmm);
        s->setInitialPoints(init_point);
        s->compute();
        cout << "\tElapsed "<< s->getName() <<" time: " << s->getTime() << " ms" << '\n';
        GridWriter::saveGridValues("3dresult.grid", grid_fmm);
    }

    return 0;
}
