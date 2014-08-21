/* n dimensional Fast Marching example with the main functions used */
 
#include <iostream>
#include <cmath>
#include <chrono>
#include <array>
#include <string>
#include <algorithm>

#include "fmdata/fmdirectionalcell.h"
#include "ndgridmap/ndgridmap.hpp"
#include "console/console.h"
#include "fm2directional/fm2directional.hpp"
#include "fm2/fm2.hpp"
#include "fmdata/fmfibheap.hpp"
#include "fmdata/fmpriorityqueue.hpp"
#include "fmdata/fmdaryheap.hpp"
#include "fmdata/fmdaryheap.hpp"
#include "io/maploader.hpp"
#include "io/gridplotter.hpp"
#include "io/gridwriter.hpp"
#include "io/gridpoints.hpp"
#include "gradientdescent/gradientdescent.hpp"

using namespace std;
using namespace std::chrono;
using namespace cimg_library;

int main(int argc, const char ** argv)
{
    constexpr int ndims = 2; // Setting two dimensions.

    time_point<std::chrono::system_clock> start, end; // Time measuring.
    double time_elapsed;

    console::info("Parsing input arguments.");
    string filename;
    float leafsize, maxDistance;

    console::parseArguments(argc,argv, "-map", filename);
    console::parseArguments(argc,argv, "-ls", leafsize);
    console::parseArguments(argc,argv, "-md", maxDistance);

    vector<int> init_points;
    vector<int> fmm2_sources;
    console::info("Now using all black points as wave sources");
    nDGridMap<FMDirectionalCell, ndims> grid;
    nDGridMap<FMCell, ndims> gridFM2;

    MapLoader::loadMapFromImg(filename.c_str(), grid, fmm2_sources); // This is the only thing that changes.
    grid.setLeafSize(leafsize);


    MapLoader::loadMapFromImg(filename.c_str(), gridFM2, fmm2_sources); // This is the only thing that changes.
    gridFM2.setLeafSize(leafsize);

    std::array<int, ndims> coords_init, coords_goal;
    GridPoints::selectMapPoints(grid, coords_init, coords_goal);

    int idx, goal;
    grid.coord2idx(coords_init, idx);
    init_points.push_back(idx);
    grid.coord2idx(coords_goal, goal);

    typedef typename std::vector< std::array<double, ndims> > Path; // A bit of short-hand.
    Path pathFM2Directional;

    std::vector <double> path_velocity; // Velocity of the path

    FastMarching2Directional< nDGridMap<FMDirectionalCell, ndims>, Path > fm2directional;

    fm2directional.setEnvironment(&grid);
        start = system_clock::now();
    fm2directional.setInitialAndGoalPoints(init_points, fmm2_sources, goal);
    fm2directional.computeFM2Directional(true);
        end = system_clock::now();
         time_elapsed = duration_cast<milliseconds>(end-start).count();
        cout << "\tElapsed FM time: " << time_elapsed << " ms" << endl;

        start = system_clock::now();
    fm2directional.computePath(&pathFM2Directional, &path_velocity);
        end = system_clock::now();
        time_elapsed = duration_cast<milliseconds>(end-start).count();
        cout << "\tElapsed gradient descent time: " << time_elapsed << " ms" << endl;

        GridWriter::savePathVelocity("path_velocity.txt", grid, pathFM2Directional, path_velocity);

    Path pathFM2;

    FastMarching2< nDGridMap<FMCell, ndims>, Path > fm2;

    fm2.setEnvironment(&gridFM2);
        start = system_clock::now();
    fm2.setInitialAndGoalPoints(init_points, fmm2_sources, goal);
    fm2.computeFM2(maxDistance);
        end = system_clock::now();
         time_elapsed = duration_cast<milliseconds>(end-start).count();
        cout << "\tElapsed FM time: " << time_elapsed << " ms" << endl;

        start = system_clock::now();
    fm2.computePath(&pathFM2);
        end = system_clock::now();
        time_elapsed = duration_cast<milliseconds>(end-start).count();
        cout << "\tElapsed gradient descent time: " << time_elapsed << " ms" << endl;

    std::vector<Path> paths;

    paths.push_back(pathFM2Directional);
    paths.push_back(pathFM2);
    GridPlotter::plotMapPath(grid,paths);

    return 0;
}
