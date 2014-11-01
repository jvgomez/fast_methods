#include <iostream>
#include <cmath>
#include <chrono>
#include <array>
#include <string>

#include "../fmm/fmdata/fmcell.h"
#include "../ndgridmap/ndgridmap.hpp"
#include "../console/console.h"

#include "../fmm/groupmarching.hpp"

#include "../io/gridwriter.hpp"

using namespace std;
using namespace std::chrono;

int main(int argc, const char ** argv)
{
    constexpr int ndims2 = 2;

    typedef nDGridMap<FMCell, ndims2> FMGrid2D;
    typedef array<int, ndims2> Coord2D;

    time_point<std::chrono::system_clock> start, end; // Time measuring.
    double time_elapsed;

    Coord2D dimsize = {1000,1000};
    FMGrid2D grid_gmm (dimsize);

    Coord2D init_point = {500,500};
    vector<int> init_points;
    int idx;
    grid_gmm.coord2idx(init_point , idx);
    init_points.push_back(idx);

    GroupMarching<FMGrid2D> gmm;
    gmm.setEnvironment(&grid_gmm);
        start = system_clock::now();
    gmm.setInitialPoints(init_points);
    gmm.computeFM();
        end = system_clock::now();
        time_elapsed = duration_cast<milliseconds>(end-start).count();
        cout << "\tElapsed GMM time: " << time_elapsed << " ms" << endl;

    GridWriter::saveGridValues("test_gmm.txt", grid_gmm);
}
