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
#include "../fm2/fastmarching2star.hpp"
#include "../fmm/fmdata/fmfibheap.hpp"
#include "../fmm/fmdata/fmpriorityqueue.hpp"
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

    FastMarching2Star<FMGrid2D, FMFibHeap<FMCell> > fm2star_fib;
    fm2star_fib.setEnvironment(&grid_fm2_fib);
        start = system_clock::now();
    fm2star_fib.setInitialAndGoalPoints(init_points, fm2_sources, goal);
    fm2star_fib.computeFM2Star();
        end = system_clock::now();
        time_elapsed = duration_cast<milliseconds>(end-start).count();
        cout << "\tElapsed FM2Star - Fibonacci heap time: " << time_elapsed << " ms" << endl;

    FastMarching2Star<FMGrid2D> fm2star;
    fm2star.setEnvironment(&grid_fm2);
        start = system_clock::now();
    fm2star.setInitialAndGoalPoints(init_points, fm2_sources, goal);
    fm2star.computeFM2Star();
        end = system_clock::now();
        time_elapsed = duration_cast<milliseconds>(end-start).count();
        cout << "\tElapsed FM2Star - Binary heap time: " << time_elapsed << " ms" << endl;

    // Using priority queue implies the use of the SFMM. Priority queue uses by default FMCell.
    FastMarching2Star<FMGrid2D, FMPriorityQueue<> > sfm2star; //Choosing the default cell class.
    sfm2star.setEnvironment(&grid_sfm2);
        start = system_clock::now();
    sfm2star.setInitialAndGoalPoints(init_points, fm2_sources, goal);
    sfm2star.computeFM2Star();
        end = system_clock::now();
        time_elapsed = duration_cast<milliseconds>(end-start).count();
        cout << "\tElapsed SFMM-based FM2Star time: " << time_elapsed << " ms" << endl;
}
