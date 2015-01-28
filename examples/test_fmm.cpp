#include <iostream>
#include <cmath>
#include <chrono>
#include <array>
#include <string>

#include "../fmm/fmdata/fmcell.h"
#include "../ndgridmap/ndgridmap.hpp"
#include "../console/console.h"

#include "../fmm/fastmarching.hpp"
#include "../fmm/fim.hpp"
#include "../fmm/groupmarching.hpp"
#include "../fmm/fmm_untidy.hpp"

#include "../fmm/fmdata/fmfibheap.hpp"
#include "../fmm/fmdata/fmdaryheap.hpp"
#include "../fmm/fmdata/fmpriorityqueue.hpp"

#include "../gradientdescent/gradientdescent.hpp"

#include "../io/gridwriter.hpp"

using namespace std;
using namespace std::chrono;

int main(int argc, const char ** argv)
{
    constexpr int ndims2 = 2; // Setting two dimensions.

    // A bit of shorthand.
    typedef nDGridMap<FMCell, ndims2> FMGrid2D;
    typedef nDGridMap<FMUntidyCell, ndims2> UFMGrid2D;
    typedef array<int, ndims2> Coord2D;

    time_point<std::chrono::system_clock> start, end; // Time measuring.
    double time_elapsed;

    Coord2D dimsize {300,300};
    FMGrid2D grid_fmm (dimsize);

    Coord2D init_point = {150, 150};
    vector<int> init_points;
    int idx;
    grid_fmm.coord2idx(init_point , idx);
    init_points.push_back(idx);

    std::vector<Solver<FMGrid2D>*> solvers;
    solvers.push_back(new FastMarching<FMGrid2D>);
    solvers.push_back(new FastIterativeMethod<FMGrid2D>);
    solvers.push_back(new GroupMarching<FMGrid2D>);
    //solvers.push_back(new FMM_Untidy<UFMGrid2D>);

    for (Solver<FMGrid2D>* s :solvers)
    {
        s->setEnvironment(&grid_fmm);
            start = system_clock::now();
        s->setInitialPoints(init_points);
        s->init();
        s->compute();
            end = system_clock::now();
            time_elapsed = duration_cast<milliseconds>(end-start).count();
            cout << "\tElapsed "<< s->getName() <<" heap time: " << time_elapsed << " ms" << '\n';

        std::string filename ("test_");
        filename += s->getName();
        filename += ".txt";
        GridWriter::saveGridValues(filename.c_str(), grid_fmm);
    }

}
