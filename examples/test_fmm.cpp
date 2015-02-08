/* Runs different versions of FMM over an empty, generated grid. */

#include <iostream>
#include <cmath>
#include <chrono>
#include <array>
#include <string>

#include "../fmm/fmdata/fmcell.h"
#include "../ndgridmap/ndgridmap.hpp"
#include "../console/console.h"

#include "../fmm/fmm.hpp"
#include "../fmm/fmdata/fmfibheap.hpp"
#include "../fmm/fmdata/fmpriorityqueue.hpp"
#include "../fmm/fim.hpp"
#include "../fmm/gmm.hpp"
#include "../fmm/ufmm.hpp"

#include "../io/gridplotter.hpp"

using namespace std;
using namespace std::chrono;

int main(int argc, const char ** argv)
{
    // A bit of shorthand.
    typedef nDGridMap<FMCell, 2> FMGrid2D;
    typedef array<unsigned int, 2> Coord2D;

    // Time measuring variables.
    time_point<std::chrono::system_clock> start, end; // Time measuring.
    double time_elapsed;

    // Grid, start and goal definition.
    Coord2D dimsize {300,300};
    FMGrid2D grid_fmm (dimsize);
    Coord2D init_point = {150, 150};
    Coord2D goal_point = {250, 250};

    // Solvers declaration.
    std::vector<Solver<FMGrid2D>*> solvers;
    solvers.push_back(new FMM<FMGrid2D>);
    solvers.push_back(new FMM<FMGrid2D, FMFibHeap<FMCell> >("FMFib"));
    solvers.push_back(new FMM<FMGrid2D, FMPriorityQueue<FMCell> >("SFMM"));
    solvers.push_back(new GMM<FMGrid2D>("GMM"));
    solvers.push_back(new FIM<FMGrid2D>("FIM"));
    solvers.push_back(new UFMM<FMGrid2D>("UFMM"));

    // Executing every solver individually over the same grid.
    for (Solver<FMGrid2D>* s :solvers)
    {
        s->setEnvironment(&grid_fmm);
            start = system_clock::now();
        //s->setInitialPoints(init_point); // If no goal_idx is set.
        s->setInitialAndGoalPoints(init_point, goal_point);
        s->compute();
            end = system_clock::now();
            time_elapsed = duration_cast<milliseconds>(end-start).count();
            cout << "\tElapsed "<< s->getName() <<" time: " << time_elapsed << " ms" << '\n';
        GridPlotter::plotArrivalTimes(grid_fmm);
    }

    // Preventing memory leaks.
    for (auto & s : solvers)
        delete s;
}
