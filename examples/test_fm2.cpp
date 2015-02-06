/* n dimensional Fast Marching example with the main functions used */

#include <iostream>
#include <cmath>
#include <chrono>
#include <array>
#include <string>
#include <algorithm>

#include "../fmm/fmdata/fmcell.h"
#include "../ndgridmap/ndgridmap.hpp"
#include "../console/console.h"
#include "../fm2/fm2.hpp"
#include "../fmm/fmdata/fmfibheap.hpp"
#include "../fmm/fmdata/fmpriorityqueue.hpp"
#include "../fmm/fmdata/fmdaryheap.hpp"
#include "../io/maploadertext.hpp"

#include "../fmm/gmm.hpp"
#include "../fmm/fim.hpp"
#include "../fmm/ufmm.hpp"

using namespace std;
using namespace std::chrono;


int main(int argc, const char ** argv)
{
    constexpr unsigned int ndims2 = 2; // Setting two dimensions.

    console::info("Parsing input arguments.");
    string filename;
    if (argc > 2)
        console::parseArguments(argc,argv, "-map", filename);
    else {
        console::info("No enough arguments given. Use as ./test_fm2 -map path_to_file.txt");
        exit(1);
    }

    // A bit of shorthand.
    typedef nDGridMap<FMCell, ndims2> FMGrid2D;
    typedef array<unsigned int, ndims2> Coord2D;

    time_point<std::chrono::system_clock> start, end; // Time measuring.
    double time_elapsed;

    FMGrid2D grid_fm2;

    if(!MapLoaderText::loadMapFromText(filename.c_str(), grid_fm2))
        exit(1);

    Coord2D init_point = {377, 664};
    Coord2D goal_point = {379, 91};
    vector<unsigned int> init_points;
    unsigned int start_idx, goal_idx;
    grid_fm2.coord2idx(init_point , start_idx);
    init_points.push_back(start_idx);
    grid_fm2.coord2idx(goal_point , goal_idx);

    std::vector<Solver<FMGrid2D>*> solvers;
    solvers.push_back(new FM2<FMGrid2D>("FM2_Dary"));
    solvers.push_back(new FM2<FMGrid2D, FMM<FMGrid2D, FMFibHeap<FMCell> > >("FM2_Fib"));
    solvers.push_back(new FM2<FMGrid2D, FMM<FMGrid2D, FMPriorityQueue<FMCell> > >("FM2_SFMM"));
    solvers.push_back(new FM2<FMGrid2D, GMM<FMGrid2D> >("FM2_GMM"));
    solvers.push_back(new FM2<FMGrid2D, FIM<FMGrid2D> >("FM2_FIM"));
    solvers.push_back(new FM2<FMGrid2D, UFMM<FMGrid2D> >("FM2_UFMM"));

    for (Solver<FMGrid2D>* s :solvers)
    {
        s->setEnvironment(&grid_fm2);
            start = system_clock::now();
        s->setInitialAndGoalPoints(init_points, goal_idx);
        s->compute();
            end = system_clock::now();
            time_elapsed = duration_cast<milliseconds>(end-start).count();
            cout << "\tElapsed "<< s->getName() <<" time: " << time_elapsed << " ms" << '\n';
    }

    // Preventing memory leaks.
    for (auto & s : solvers)
        delete s;
}
