#include <iostream>
#include <cmath>
#include <array>
#include <string>

#include "../fmm/fmdata/fmcell.h"
#include "../ndgridmap/ndgridmap.hpp"

#include "../fmm/fmm.hpp"
#include "../fmm/fim.hpp"
#include "../fmm/gmm.hpp"
#include "../fmm/ufmm.hpp"

#include "../fmm/fmdata/fmfibheap.hpp"
#include "../fmm/fmdata/fmdaryheap.hpp"
#include "../fmm/fmdata/fmpriorityqueue.hpp"

#include "../fmm/benchmark.hpp"

using namespace std;

int main(int argc, const char ** argv)
{
    constexpr int ndims2 = 2; // Setting two dimensions.

    // A bit of shorthand.
    typedef nDGridMap<FMCell, ndims2> FMGrid2D;
    typedef array<int, ndims2> Coord2D;

    Coord2D dimsize {300,300};
    FMGrid2D grid (dimsize);

    Coord2D init_point = {150, 150};
    Coord2D goal_point = {250, 250};
    vector<int> init_points;
    int idx, goal_idx;
    grid.coord2idx(init_point, idx);
    grid.coord2idx(goal_point, goal_idx);
    init_points.push_back(idx);

    Benchmark<FMGrid2D> b(true);
    b.setEnvironment(&grid);
    b.setInitialAndGoalPoints(init_points,goal_idx);

    b.addSolver(new FMM<FMGrid2D>);
    b.addSolver(new FMM<FMGrid2D, FMFibHeap<FMCell> >("FMFib"));
    b.addSolver(new FMM<FMGrid2D, FMPriorityQueue<FMCell> >("SFMM"));
    b.addSolver(new FIM<FMGrid2D>);
    b.addSolver(new GMM<FMGrid2D>);
    b.addSolver(new UFMM<FMGrid2D>);

    b.run();
}
