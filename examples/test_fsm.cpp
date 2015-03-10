/* */

#include <iostream>
#include <array>
#include <chrono>

#include "../ndgridmap/ndgridmap.hpp"
#include "../fmm/fmdata/fmcell.h"
#include "../fmm/fsm.hpp"
#include "../fmm/fmm.hpp"

#include "../io/gridplotter.hpp"
#include "../io/gridwriter.hpp"
#include "../io/maploader.hpp"

#define FROMIMG

using namespace std;

// A bit of shorthand.
constexpr unsigned int ndims = 2;
typedef nDGridMap<FMCell, ndims> Grid2D;
typedef array<unsigned int, ndims> Coord2D;

int main(int argc, const char** argv)
{

#ifndef FROMIMG
    unsigned dim = atoi(argv[1]);
    Coord2D dimsize_ = {dim,dim};
    Grid2D grid (dimsize_);
    Coord2D init_point = {dim/2, dim/2};
#else
    Grid2D grid;
    MapLoader::loadMapFromImg(argv[1], grid);
    Coord2D init_point = {50, 50};
#endif
    //Coord2D goal_point = {333, 227};


    //Coord2D goal_point = {dim-50, dim-50};
    //Coord2D goal_point = {3, 3};

    FSM<Grid2D> fsm(atoi(argv[2]));
    fsm.setEnvironment(&grid);
    //fsm.setInitialAndGoalPoints(init_point, goal_point);
    fsm.setInitialPoints(init_point);
    fsm.compute();
    fsm.printRunInfo();
    //cout << "\tElapsed "<< fsm.getName() <<" time: " << fsm.getTime() << " ms" << '\n';
    //GridPlotter::plotArrivalTimes(grid, fsm.getName());
    GridWriter::saveGridValues("FSM", grid);

    FMM<Grid2D> fmm;
    fmm.setEnvironment(&grid);
    //fmm.setInitialAndGoalPoints(init_point, goal_point);
    fmm.setInitialPoints(init_point);
    fmm.compute();
    fmm.printRunInfo();
    //GridPlotter::plotArrivalTimes(grid, fsm.getName());
    GridWriter::saveGridValues("FMM", grid);


    return 0;
}
