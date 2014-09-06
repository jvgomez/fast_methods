/* n dimensional Fast Marching example with the main functions used */

#include <iostream>
#include <cmath>
#include <chrono>
#include <array>
#include <string>
#include <algorithm>

#include "../fmdata/fmdirectionalcell.h"
#include "../ndgridmap/ndgridmap.hpp"
#include "../console/console.h"
#include "../fm2directional/fastmarching2directional.hpp"
#include "../fmdata/fmfibheap.hpp"
#include "../fmdata/fmpriorityqueue.hpp"
#include "../fmdata/fmdaryheap.hpp"
#include "../fmdata/fmdaryheap.hpp"
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
    typedef nDGridMap<FMDirectionalCell, ndims2> FMGrid2D;
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

    FastMarching2Directional<FMGrid2D, FMFibHeap<FMDirectionalCell> > fm2directional_fib;
    fm2directional_fib.setEnvironment(&grid_fm2_fib);
        start = system_clock::now();
    fm2directional_fib.setInitialAndGoalPoints(init_points, fm2_sources, goal);
    fm2directional_fib.computeFM2Directional();
        end = system_clock::now();
        time_elapsed = duration_cast<milliseconds>(end-start).count();
        cout << "\tElapsed FM2Directional - Fibonacci heap time:\t" << time_elapsed << " ms" << endl;

    FastMarching2Directional<FMGrid2D> fm2directional;
    fm2directional.setEnvironment(&grid_fm2);
        start = system_clock::now();
    fm2directional.setInitialAndGoalPoints(init_points, fm2_sources, goal);
    fm2directional.computeFM2Directional();
        end = system_clock::now();
        time_elapsed = duration_cast<milliseconds>(end-start).count();
        cout << "\tElapsed FM2Directiona - Binary heap time:\t" << time_elapsed << " ms" << endl;

    // Using priority queue implies the use of the SFMM. Priority queue uses by default FMCell.
    FastMarching2Directional<FMGrid2D, FMPriorityQueue<> > sfm2directional; //Choosing the default cell class.
    sfm2directional.setEnvironment(&grid_sfm2);
        start = system_clock::now();
    sfm2directional.setInitialAndGoalPoints(init_points, fm2_sources, goal);
    sfm2directional.computeFM2Directional();
        end = system_clock::now();
        time_elapsed = duration_cast<milliseconds>(end-start).count();
        cout << "\tElapsed SFMM-based FM2Directional time:\t\t" << time_elapsed << " ms" << endl;
}
