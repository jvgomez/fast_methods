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

#include "../io/gridwriter.hpp"

using namespace std;
using namespace std::chrono;

int main(int argc, const char ** argv)
{
    // A bit of shorthand.
    typedef nDGridMap<FMCell, 2> FMGrid2D;
    typedef array<unsigned int, 2> Coord2D;

    time_point<std::chrono::system_clock> start, end; // Time measuring.
    double time_elapsed;

    Coord2D dimsize {300,300};
    FMGrid2D grid_fmm (dimsize);

    Coord2D init_point = {150, 150};
    Coord2D goal_point = {250, 250};
    vector<unsigned int> init_points;
    unsigned int idx, goal_idx;
    grid_fmm.coord2idx(init_point , idx);
    grid_fmm.coord2idx(goal_point , goal_idx);
    init_points.push_back(idx);

    std::vector<Solver<FMGrid2D>*> solvers;
    solvers.push_back(new FMM<FMGrid2D>);
    solvers.push_back(new FMM<FMGrid2D, FMFibHeap<FMCell> >("FMFib"));
    solvers.push_back(new FMM<FMGrid2D, FMPriorityQueue<FMCell> >("SFMM"));

    for (Solver<FMGrid2D>* s :solvers)
    {
        s->setEnvironment(&grid_fmm);
            start = system_clock::now();
        //s->setInitialPoints(init_points); // If no goal_idx is set.
        s->setInitialAndGoalPoints(init_points, goal_idx);
        s->compute();
            end = system_clock::now();
            time_elapsed = duration_cast<milliseconds>(end-start).count();
            cout << "\tElapsed "<< s->getName() <<" time: " << time_elapsed << " ms" << '\n';

        std::string filename ("test_");
        filename += s->getName();
        filename += ".txt";
        GridWriter::saveGridValues(filename.c_str(), grid_fmm);
    }

    // Preventing memory leaks.
    for (auto & s : solvers)
        delete s;
}
