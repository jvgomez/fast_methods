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
#include "../fm2/fastmarching2.hpp"
#include "../fmm/fmdata/fmfibheap.hpp"
#include "../fmm/fmdata/fmpriorityqueue.hpp"
#include "../fmm/fmdata/fmdaryheap.hpp"
#include "../fmm/fmdata/fmdaryheap.hpp"
#include "../io/maploadertext.hpp"

using namespace std;
using namespace std::chrono;


int main(int argc, const char ** argv)
{
    constexpr int ndims2 = 2; // Setting two dimensions.

    console::info("Parsing input arguments.");
    string filename;
    if (argc > 2)
        console::parseArguments(argc,argv, "-map", filename);
    else {
        console::info("No enough arguments given. Loading default example map: examples/data/grid.txt");
        filename = "../data/grid.txt";
    }

    // A bit of shorthand.
    typedef nDGridMap<FMCell, ndims2> FMGrid2D;
    typedef array<int, ndims2> Coord2D;

    time_point<std::chrono::system_clock> start, end; // Time measuring.
    double time_elapsed;

    FMGrid2D grid_fm2;
    FMGrid2D grid_fm2_fib;
    FMGrid2D grid_sfm2;

    vector<int> fm2_sources;

    // TODO: implement a grid copy constructor and operator.
    MapLoaderText::loadMapFromText(filename.c_str(), grid_fm2, fm2_sources);
    MapLoaderText::loadMapFromText(filename.c_str(), grid_fm2_fib);
    MapLoaderText::loadMapFromText(filename.c_str(), grid_sfm2);

    Coord2D init_point = {377, 664};
    Coord2D goal_point = {379, 91};
    vector<int> init_points;
    int idx, goal;
    grid_fm2.coord2idx(init_point , idx);
    init_points.push_back(idx);
    grid_fm2.coord2idx(goal_point , goal);

    FastMarching2<FMGrid2D, FMFibHeap<FMCell> > fm2;
    fm2.setEnvironment(&grid_fm2_fib);
        start = system_clock::now();
    fm2.setInitialAndGoalPoints(init_points, fm2_sources, goal);
    fm2.computeFM2();
        end = system_clock::now();
        time_elapsed = duration_cast<milliseconds>(end-start).count();
        cout << "\tElapsed FMM - Fibonacci heap time:\t" << time_elapsed << " ms" << endl;

    FastMarching2<FMGrid2D> fm2_dary;
    fm2_dary.setEnvironment(&grid_fm2);
        start = system_clock::now();
    fm2_dary.setInitialAndGoalPoints(init_points, fm2_sources, goal);
    fm2_dary.computeFM2();
        end = system_clock::now();
        time_elapsed = duration_cast<milliseconds>(end-start).count();
        cout << "\tElapsed FMM - Binary heap time:\t\t" << time_elapsed << " ms" << endl;

    // Using priority queue implies the use of the SFMM. Priority queue uses by default FMCell.
    FastMarching2<FMGrid2D, FMPriorityQueue<> > sfm2; //Choosing the default cell class.
    sfm2.setEnvironment(&grid_sfm2);
        start = system_clock::now();
    sfm2.setInitialAndGoalPoints(init_points, fm2_sources, goal);
    sfm2.computeFM2();
        end = system_clock::now();
        time_elapsed = duration_cast<milliseconds>(end-start).count();
        cout << "\tElapsed SFMM-based FM2 time:\t\t" << time_elapsed << " ms" << endl;
}
