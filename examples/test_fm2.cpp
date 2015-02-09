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

using namespace std;
using namespace std::chrono;

int main(int argc, const char ** argv)
{
    console::info("Parsing input arguments.");
    string filename;
    if (argc > 2)
        console::parseArguments(argc,argv, "-map", filename);
    else {
        console::info("No enough arguments given. Use as ./test_fm2 -map path_to_file.txt");
        exit(1);
    }

    // A bit of shorthand.
    constexpr unsigned int ndims2 = 2; // Setting two dimensions.
    typedef nDGridMap<FMCell, ndims2> FMGrid2D;

    // Loading grid.
    FMGrid2D grid_fm2;
    //if(!MapLoader::loadMapFromText(filename.c_str(), grid_fm2))
    MapLoader::loadMapFromImg(filename.c_str(), grid_fm2);
        //exit(1);

    // Solvers declaration.
    std::vector<Solver<FMGrid2D>*> solvers;
    solvers.push_back(new FM2<FMGrid2D>("FM2_Dary"));
    //solvers.push_back(new FM2<FMGrid2D, FMFibHeap<FMCell> >("FM2_Fib"));
    //solvers.push_back(new FM2<FMGrid2D, FMPriorityQueue<FMCell> >("FM2_SFMM"));
    solvers.push_back(new FM2Star<FMGrid2D>("FM2*_Dary"));
    //solvers.push_back(new FM2Star<FMGrid2D, FMFibHeap<FMCell> >("FM2*_Fib"));
    //solvers.push_back(new FM2Star<FMGrid2D, FMPriorityQueue<FMCell> >("FM2*_SFMM"));

    for (int i = 0; i < 10; ++i) {
    // Executing every solver individually over the same grid.
        for (Solver<FMGrid2D>* s :solvers)
        {
            s->setEnvironment(&grid_fm2);
            s->setInitialAndGoalPoints({20, 20}, {375, 280}); // Init and goal points directly set.
            s->compute();
                cout << "\tElapsed "<< s->getName() <<" time: " << s->getTime() << " ms" << '\n';
            GridPlotter::plotArrivalTimes(grid_fm2);
        }
    }

    // Preventing memory leaks.
    for (auto & s : solvers)
        delete s;
}
