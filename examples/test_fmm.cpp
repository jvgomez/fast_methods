/* Runs different versions of FMM over an empty, generated grid. */

#include <iostream>
#include <array>

#include "../fmm/fmdata/fmcell.h"
#include "../ndgridmap/ndgridmap.hpp"

#include "../fmm/fmm.hpp"
#include "../fmm/sfmm.hpp"
#include "../fmm/sfmmstar.hpp"
#include "../fmm/fmmstar.hpp"
#include "../fmm/fmdata/fmfibheap.hpp"
#include "../fmm/fmdata/fmpriorityqueue.hpp"
#include "../fmm/fim.hpp"
#include "../fmm/gmm.hpp"
#include "../fmm/ufmm.hpp"
#include "../fmm/fsm.hpp"
#include "../fmm/lsm.hpp"

#include "../io/gridplotter.hpp"

using namespace std;
using namespace std::chrono;

int main()
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
    solvers.push_back(new FMMStar<FMGrid2D>);
    solvers.push_back(new FMMStar<FMGrid2D>("FMM*_Dist", DISTANCE));
    solvers.push_back(new FMM<FMGrid2D, FMFibHeap<FMCell> >("FMFib"));
    solvers.push_back(new FMMStar<FMGrid2D, FMFibHeap<FMCell> >("FMM*Fib"));
    solvers.push_back(new FMMStar<FMGrid2D, FMFibHeap<FMCell> >("FMM*Fib_Dist", DISTANCE));
    solvers.push_back(new SFMM<FMGrid2D>);
    solvers.push_back(new SFMMStar<FMGrid2D>);
    solvers.push_back(new SFMMStar<FMGrid2D>("SFMM*_Dist", DISTANCE));
    solvers.push_back(new GMM<FMGrid2D>);
    solvers.push_back(new FIM<FMGrid2D>);
    solvers.push_back(new UFMM<FMGrid2D>);
    solvers.push_back(new FSM<FMGrid2D>);
    solvers.push_back(new LSM<FMGrid2D>);
    // GMM, FIM and UFMM have some parameters, can be set by overloaded constructors.

    // Executing every solver individually over the same grid.
    for (Solver<FMGrid2D>* s :solvers)
    {
        s->setEnvironment(&grid_fmm);
        //s->setInitialPoints(init_point); // If no goal_idx is set. Comment next line if this one is uncommented.
        s->setInitialAndGoalPoints(init_point, goal_point);
        s->setup(); // Not mandatory, but in FMMstar we want to precompute distances
                    // out of compute().
        s->compute();
        cout << "\tElapsed "<< s->getName() <<" time: " << s->getTime() << " ms" << '\n';
        GridPlotter::plotArrivalTimes(grid_fmm, s->getName());
    }

    // Showing different type conversions methodologies. Solver names could be compared
    // but it is less reliable since they are user-defined.
    // Direct cast
    /*FMM<FMGrid2D>* FMMtest = dynamic_cast< FMM<FMGrid2D>* >(solvers[6]);
    if (FMMtest)
        std::cout << "Solver type SFMM" <<'\n';
    else
        std::cout << "Solver NOT type SFMM" <<'\n';

    FMM<FMGrid2D, FMPriorityQueue<FMCell>>* FMMtest2 = dynamic_cast< FMM<FMGrid2D, FMPriorityQueue<FMCell>>* >(solvers[6]);
    if (FMMtest2)
        std::cout << "Solver type SFMM" <<'\n';
    else
        std::cout << "Solver NOT type SFMM" <<'\n';

    // Solver::as member function.
    // solvers[6]->setHeuristics(true) would not compile since there is not a Solver::setHeuristics
    solvers[6]->as< FMM<FMGrid2D, FMPriorityQueue<FMCell>> >()->setHeuristics(HeurStrategy::DISTANCE);
    std::cout << solvers[6]->as< FMM<FMGrid2D, FMPriorityQueue<FMCell>> >()->getHeuristics() << '\n';
*/
    // Preventing memory leaks.
    for (auto & s : solvers)
        delete s;

    return 0;
}
