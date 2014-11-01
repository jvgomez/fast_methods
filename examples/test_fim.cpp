#include <iostream>
#include <cmath>
#include <chrono>
#include <array>
#include <string>

#include "../fmm/fmdata/fmcell.h"
#include "../ndgridmap/ndgridmap.hpp"
#include "../console/console.h"

#include "../fmm/fim.hpp"

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

    Coord2D dimsize = {200,200};
    FMGrid2D grid_fim (dimsize);

    Coord2D init_point = {4,4};
    vector<int> init_points;
    int idx;
    grid_fim.coord2idx(init_point , idx);
    init_points.push_back(idx);

    FastIterativeMethod<FMGrid2D> fim;
    fim.setEnvironment(&grid_fim);
        start = system_clock::now();
    fim.setInitialPoints(init_points);
    fim.computeFM();
        end = system_clock::now();
        time_elapsed = duration_cast<milliseconds>(end-start).count();
        cout << "\tElapsed FIM time: " << time_elapsed << " ms" << endl;

    GridWriter::saveGridValues("test_fim.txt", grid_fim);

}
