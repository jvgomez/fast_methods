/* Configure a Fast Methods benchmark from code. Compare this code with test_fm.cpp */

#include <iostream>
#include <array>

#include <fast_methods/ndgridmap/fmcell.h>
#include <fast_methods/ndgridmap/ndgridmap.hpp>

#include <fast_methods/fm/fmm.hpp>
#include <fast_methods/fm/sfmm.hpp>
#include <fast_methods/fm/fmmstar.hpp>
#include <fast_methods/fm/sfmmstar.hpp>
#include <fast_methods/datastructures/fmfibheap.hpp>
#include <fast_methods/datastructures/fmpriorityqueue.hpp>
#include <fast_methods/fm/fim.hpp>
#include <fast_methods/fm/gmm.hpp>
#include <fast_methods/fm/ufmm.hpp>
#include <fast_methods/fm/fsm.hpp>
#include <fast_methods/fm/lsm.hpp>
#include <fast_methods/fm/ddqm.hpp>

#include <fast_methods/benchmark/benchmark.hpp>

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
    benchmark.addSolver(new SFMM<FMGrid2D>);
    benchmark.addSolver(new SFMMStar<FMGrid2D>);
    benchmark.addSolver(new SFMMStar<FMGrid2D>("SFMM*_Dist", DISTANCE));
    benchmark.addSolver(new GMM<FMGrid2D>);
    benchmark.addSolver(new FIM<FMGrid2D>);
    benchmark.addSolver(new UFMM<FMGrid2D>);
    benchmark.addSolver(new FSM<FMGrid2D>);
    benchmark.addSolver(new LSM<FMGrid2D>);
    benchmark.addSolver(new DDQM<FMGrid2D>);

    // Run benchmark
    benchmark.run();

    return 0;
}
