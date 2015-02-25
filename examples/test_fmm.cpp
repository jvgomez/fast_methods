/* Runs different versions of FMM over an empty, generated grid. */

#include <iostream>
#include <array>

#include "../fmm/fmdata/fmcell.h"
#include "../ndgridmap/ndgridmap.hpp"

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
        //s->setInitialPoints(init_point); // If no goal_idx is set.
        s->setInitialAndGoalPoints(init_point, goal_point);
        s->compute();
        cout << "\tElapsed "<< s->getName() <<" time: " << s->getTime() << " ms" << '\n';
        GridPlotter::plotArrivalTimes(grid_fmm);
    }

    // Showing different type conversions methodologies
    // Direct cast
    FMM<FMGrid2D>* FMMtest = dynamic_cast< FMM<FMGrid2D>* >(solvers[2]);
    if (FMMtest)
        std::cout << "Solver type SFMM" <<'\n';
    else
        std::cout << "Solver NOT type SFMM" <<'\n';

    FMM<FMGrid2D, FMPriorityQueue<FMCell>>* FMMtest2 = dynamic_cast< FMM<FMGrid2D, FMPriorityQueue<FMCell>>* >(solvers[2]);
    if (FMMtest2)
        std::cout << "Solver type SFMM" <<'\n';
    else
        std::cout << "Solver NOT type SFMM" <<'\n';

    // Solver::as member function.
    // solvers[2]->setHeuristics(true) would not compile.
    solvers[2]->as< FMM<FMGrid2D, FMPriorityQueue<FMCell>> >()->setHeuristics(true);
    std::cout << solvers[2]->as< FMM<FMGrid2D, FMPriorityQueue<FMCell>> >()->getHeuristics() << '\n';

    // Preventing memory leaks.
    for (auto & s : solvers)
        delete s;
}
