/* */

#include <iostream>
#include <array>
#include <chrono>

#include "../ndgridmap/ndgridmap.hpp"
#include "../fmm/fmdata/fmcell.h"
#include "../fmm/fsm.hpp"

#include "../io/gridplotter.hpp"
#include "../io/gridwriter.hpp"

//#define TRES

using namespace std;

// A bit of shorthand.
constexpr unsigned int ndims = 2;
typedef nDGridMap<FMCell, ndims> Grid2D;
typedef array<unsigned int, ndims> Coord2D;

int main()
{
    Coord2D dimsize_ = {300,300};
    //Coord2D dimsize_ = {3,3};
    Grid2D grid (dimsize_);
    Coord2D init_point = {150, 150};
    Coord2D goal_point = {250, 250};
    //Coord2D init_point = {1, 1};
    //Coord2D goal_point = {2, 2};

    FSM<Grid2D> fsm;
    fsm.setEnvironment(&grid);
    fsm.setInitialAndGoalPoints(init_point, goal_point);
    fsm.maxSweeps = 4;
    fsm.compute();

    cout << "\tElapsed "<< fsm.getName() <<" time: " << fsm.getTime() << " ms" << '\n';
    GridPlotter::plotArrivalTimes(grid, fsm.getName());

    GridWriter::saveGridValues("FMM", grid);

    fsm.reset();
    fsm.maxSweeps = 200;
    fsm.compute();

    cout << "\tElapsed "<< fsm.getName() <<" time: " << fsm.getTime() << " ms" << '\n';
    GridPlotter::plotArrivalTimes(grid, fsm.getName());

    GridWriter::saveGridValues("FSM", grid);


    return 0;
}
