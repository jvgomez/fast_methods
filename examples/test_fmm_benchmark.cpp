/* Configure a FMM Benchmark from code. Compare this code with test_fmm.cpp */

#include <iostream>
#include <array>

#include "../fmm/fmdata/fmcell.h"
#include "../ndgridmap/ndgridmap.hpp"

#include "../fmm/fmm.hpp"
#include "../fmm/fmmstar.hpp"
#include "../fmm/fmdata/fmfibheap.hpp"
#include "../fmm/fmdata/fmpriorityqueue.hpp"
#include "../fmm/fim.hpp"
#include "../fmm/gmm.hpp"
#include "../fmm/ufmm.hpp"

#include "../benchmark/benchmark.hpp"

using namespace std;
using namespace std::chrono;

int main()
{
    // A bit of shorthand.
    typedef nDGridMap<FMCell, 2> FMGrid2D;
    typedef array<unsigned int, 2> Coord2D;

    // Grid, start and goal definition. The grid can be initialized form an image as well.
    Coord2D dimsize {300,300};
    FMGrid2D grid (dimsize);
    Coord2D init_point = {150, 150};
    Coord2D goal_point = {250, 250};

    // Configuring benchmark metadata
    Benchmark<FMGrid2D> benchmark;
    benchmark.setSaveLog(false); // Shows output in terminal.
    benchmark.setName("Testing_Benchmark");
    benchmark.setNRuns(3);
    benchmark.setEnvironment(&grid);

    // Setting initial point.
    unsigned int startIdx;
    std::vector<unsigned int> startIndices;
    grid.coord2idx(init_point, startIdx);
    startIndices.push_back(startIdx);

    // Setting goal point (can be omitted)
    unsigned int goalIdx;
    grid.coord2idx(goal_point, goalIdx);
    benchmark.setInitialAndGoalPoints(startIndices, goalIdx);

    // Call this if goal point is omitted.
    //benchmark.setInitialPoints(startIndices);

    // Adding solvers.
    benchmark.addSolver(new FMM<FMGrid2D>);
    benchmark.addSolver(new FMMStar<FMGrid2D>);
    benchmark.addSolver(new FMMStar<FMGrid2D>("FMM*_Dist", DISTANCE));
    benchmark.addSolver(new FMM<FMGrid2D, FMFibHeap<FMCell> >("FMFib"));
    benchmark.addSolver(new FMMStar<FMGrid2D, FMFibHeap<FMCell> >("FMM*Fib"));
    benchmark.addSolver(new FMMStar<FMGrid2D, FMFibHeap<FMCell> >("FMM*Fib_Dist", DISTANCE));
    benchmark.addSolver(new FMM<FMGrid2D, FMPriorityQueue<FMCell> >("SFMM"));
    benchmark.addSolver(new FMMStar<FMGrid2D, FMPriorityQueue<FMCell> >("SFMM*"));
    benchmark.addSolver(new FMMStar<FMGrid2D, FMPriorityQueue<FMCell> >("SFMM*_Dist", DISTANCE));
    benchmark.addSolver(new GMM<FMGrid2D>);
    benchmark.addSolver(new FIM<FMGrid2D>("FIM"));
    benchmark.addSolver(new UFMM<FMGrid2D>("UFMM"));

    // Run benchmark
    benchmark.run();

    return 0;
}
