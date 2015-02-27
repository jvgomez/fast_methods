/* Runs different versions of FM2 and FM2* over grid map generated from a text file. */

#include <iostream>
#include <array>

#include "../fmm/fmdata/fmcell.h"
#include "../ndgridmap/ndgridmap.hpp"
#include "../console/console.h"

#include "../fm2/fm2.hpp"
#include "../fm2/fm2star.hpp"
#include "../fmm/fmdata/fmfibheap.hpp"
#include "../fmm/fmdata/fmpriorityqueue.hpp"

#include "../io/maploader.hpp"
#include "../io/gridplotter.hpp"
#include "../io/gridwriter.hpp"

using namespace std;
using namespace std::chrono;

int main(int argc, const char ** argv)
{
    console::info("Parsing input arguments.");
    string filename;
    if (argc > 2)
        console::parseArguments(argc, argv, "-map", filename);
    else {
        console::info("No enough arguments given. Use as ./test_fm2 -map path_to_file.txt");
        exit(1);
    }

    // A bit of shorthand.
    constexpr unsigned int ndims2 = 2; // Setting two dimensions.
    typedef nDGridMap<FMCell, ndims2> FMGrid2D;
    typedef typename std::vector< std::array<double, ndims2> > Path2D; // A bit of short-hand.

    // Loading grid.
    FMGrid2D grid_fm2;

    // Solvers declaration.
    std::vector<Solver<FMGrid2D>*> solvers;
    solvers.push_back(new FM2<FMGrid2D>("FM2_Dary"));
    solvers.push_back(new FM2<FMGrid2D, FMFibHeap<FMCell> >("FM2_Fib"));
    solvers.push_back(new FM2<FMGrid2D, FMPriorityQueue<FMCell> >("FM2_SFMM"));
    solvers.push_back(new FM2Star<FMGrid2D>("FM2*_Dary_Dist", DISTANCE));
    solvers.push_back(new FM2Star<FMGrid2D>("FM2*_Dary_Time"));
    solvers.push_back(new FM2Star<FMGrid2D, FMFibHeap<FMCell> >("FM2*_Fib_Time"));
    solvers.push_back(new FM2Star<FMGrid2D, FMFibHeap<FMCell> >("FM2*_Fib_Dist", DISTANCE));
    solvers.push_back(new FM2Star<FMGrid2D, FMPriorityQueue<FMCell> >("FM2*_SFMM_Time"));
    solvers.push_back(new FM2Star<FMGrid2D, FMPriorityQueue<FMCell> >("FM2*_SFMM_Dist", DISTANCE));

    // Executing every solver individually over the same grid.
    for (Solver<FMGrid2D>* s :solvers)
    {
        // For FM2 and its variations, it is better to completely reinitialize the grid.
        //if(!MapLoader::loadMapFromText(filename.c_str(), grid_fm2)) // Loading from text file.
            //exit(1);
        MapLoader::loadMapFromImg(filename.c_str(), grid_fm2);
        s->setEnvironment(&grid_fm2);
        s->setInitialAndGoalPoints({30, 20}, {375, 280}); // Init and goal points directly set.
        s->compute();
        cout << "\tElapsed "<< s->getName() <<" time: " << s->getTime() << " ms" << '\n';

        Path2D path;
        vector<double> path_vels;
        s->as<FM2<FMGrid2D>>()->computePath(&path, &path_vels);
        GridPlotter::plotArrivalTimesPath(grid_fm2, path);
    }

    // Preventing memory leaks.
    for (auto & s : solvers)
        delete s;

    return 0;
}
