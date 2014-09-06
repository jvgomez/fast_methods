/* n dimensional Fast Marching example with the main functions used */

#include <iostream>
#include <cmath>
#include <chrono>
#include <array>
#include <string>

#include "fmdata/fmcell.h"
#include "ndgridmap/ndgridmap.hpp"
#include "console/console.h"
#include "fmm/fastmarching.hpp"
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

int main(int argc, const char ** argv)
{
    constexpr int ndims = 2; // Setting two dimensions.
    constexpr int ndims3 = 3; // Setting three dimensions.

    time_point<std::chrono::system_clock> start, end; // Time measuring.
    double time_elapsed;

    console::info("Parsing input arguments.");
    string filename1, filename2, filename_vels;
    console::parseArguments(argc,argv, "-map1", filename1);
    console::parseArguments(argc,argv, "-map2", filename2);
    console::parseArguments(argc,argv, "-vel", filename_vels);

    console::info("Creating grid from image.");
    nDGridMap<FMCell, ndims> grid;
    MapLoader::loadMapFromImg(filename1.c_str(), grid);

    console::info("Showing the grid and the mirror effect.");
    GridPlotter::plotMap(grid, 0); // It looks "inverted" because the CImg (0,0) coordinates and the Y orientation.
    GridPlotter::plotMap(grid);

    console::info("Testing Fast Marching Method.");
    MapLoader::loadMapFromImg(filename2.c_str(), grid);

    std::array<int, ndims> coords_init, coords_goal;
    GridPoints::selectMapPoints(grid, coords_init, coords_goal);

    vector<int> init_points;
    int idx, goal;
    grid.coord2idx(coords_init, idx);
    init_points.push_back(idx);
    grid.coord2idx(coords_goal, goal);

    FastMarching< nDGridMap<FMCell, ndims> > fmm;
    fmm.setEnvironment(&grid);
        start = system_clock::now();
    fmm.setInitialAndGoalPoints(init_points, goal);
    fmm.computeFM();
        end = system_clock::now();
        time_elapsed = duration_cast<milliseconds>(end-start).count();
        cout << "\tElapsed FM time: " << time_elapsed << " ms" << endl;
    console::info("Plotting the results and saving into test_fm.txt");
    GridPlotter::plotArrivalTimes(grid);
    GridWriter::saveGridValues("test_fm.txt", grid);

    console::info("Computing gradient descent ");
    typedef typename std::vector< std::array<double, ndims> > Path; // A bit of short-hand.

    Path path;
    std::vector <double> path_velocity; // Velocities profile

        start = system_clock::now();
    GradientDescent< nDGridMap<FMCell, ndims> > grad;
    grad.apply(grid,goal,path,path_velocity);
        end = system_clock::now();
        time_elapsed = duration_cast<milliseconds>(end-start).count();
        cout << "\tElapsed gradient descent time: " << time_elapsed << " ms" << endl;
    GridWriter::savePath("test_path.txt", grid, path);
    GridWriter::savePathVelocity("path_velocity.txt", grid, path, path_velocity);
    GridPlotter::plotMapPath(grid,path);

    console::info("Testing Fast Marching Square Method.");
    std::vector <int> fm2_sources;
    path_velocity.clear();
    grid.coord2idx(coords_goal, goal);
    Path pathFM2;

    MapLoader::loadMapFromImg(filename2.c_str(), grid, fm2_sources);

    FastMarching2<nDGridMap<FMCell, ndims>> fm2;
    fm2.setEnvironment(&grid);
        start = system_clock::now();
    fm2.setInitialAndGoalPoints(init_points, fm2_sources, goal);
    fm2.computeFM2();
        end = system_clock::now();
        time_elapsed = duration_cast<milliseconds>(end-start).count();
        cout << "\tElapsed FM2 time: " << time_elapsed << " ms" << endl;

        start = system_clock::now();
    fm2.computePath(&pathFM2, &path_velocity);
        end = system_clock::now();
        time_elapsed = duration_cast<milliseconds>(end-start).count();
        cout << "\tElapsed gradient descent time: " << time_elapsed << " ms" << endl;

    GridPlotter::plotMapPath(grid,pathFM2);

    console::info("Comparing FMM and FM2 paths.");
    std::vector <Path> paths;
    paths.push_back(path);
    paths.push_back(pathFM2);

    GridPlotter::plotMapPath(grid,paths);

    console::info("Now using all black points as wave sources");
    nDGridMap<FMCell, ndims> grid2;
    init_points.clear();
    MapLoader::loadMapFromImg(filename2.c_str(), grid2, init_points); // This is the only thing that changes.

    FastMarching< nDGridMap<FMCell, ndims> > fmm2;
    fmm2.setEnvironment(&grid2);
        start = system_clock::now();
    fmm2.setInitialAndGoalPoints(init_points, goal);
    fmm2.computeFM(false);
        end = system_clock::now();
        time_elapsed = duration_cast<milliseconds>(end-start).count();
        cout << "\tElapsed FM time: " << time_elapsed << " ms" << endl;

    console::info("Plotting the results ");
    GridPlotter::plotArrivalTimes(grid2);

    console::info("Saving into file test2_fm.txt");
    GridWriter::saveGridValues("test2_fm.txt", grid2);

    console::info("Now let's try different velocities.");
    nDGridMap<FMCell, ndims> grid_vels;
    MapLoader::loadVelocitiesFromImg(filename_vels.c_str(), grid_vels);

    FastMarching< nDGridMap<FMCell, ndims> , FMFibHeap<>> fmm_vels;
    init_points.clear();
    init_points.push_back(80000); // Init point randomly chosen.
    fmm_vels.setEnvironment(&grid_vels);
        start = system_clock::now();
    fmm_vels.setInitialAndGoalPoints(init_points, goal);
    fmm_vels.computeFM();
        end = system_clock::now();
        time_elapsed = duration_cast<milliseconds>(end-start).count();
        cout << "\tElapsed FM time: " << time_elapsed << " ms" << endl;

    console::info("Plotting the results ");
    GridPlotter::plotArrivalTimes(grid_vels);

    console::info("Saving velocities");
    GridWriter::saveVelocities("test_vels.txt", grid_vels);

    console::info("Testing 3D!");
    nDGridMap<FMCell, ndims3> grid3 (std::array<int,ndims3>{100,100,50});
    init_points.clear();
    grid3.coord2idx(std::array<int,ndims3>{50,50,25},idx); // Reusing the previous int.
    init_points.push_back(idx);
    grid3.coord2idx(std::array<int, ndims3> {20, 10, 45}, goal);

    FastMarching< nDGridMap<FMCell, ndims3> > fmm3;
    fmm3.setEnvironment(&grid3);
        start = system_clock::now();
    fmm3.setInitialAndGoalPoints(init_points, goal);
    fmm3.computeFM();
        end = system_clock::now();
        time_elapsed = duration_cast<milliseconds>(end-start).count();
        cout << "\tElapsed FM time: " << time_elapsed << " ms" << endl;

    console::info("Saving into file test_fm3d.txt");
    GridWriter::saveGridValues("test_fm3d.txt", grid3);

    console::info("Testing 3D gradient descent.");
    typedef typename std::vector< std::array<double, ndims3> > Path3D; // A bit of short-hand.

    grid3.coord2idx(std::array<int, ndims3> {20, 10, 45}, goal);

    Path3D path3D;
        start = system_clock::now();
    GradientDescent< nDGridMap<FMCell, ndims3> > grad3D;
    grad3D.apply(grid3,goal,path3D,path_velocity);
        end = system_clock::now();
        time_elapsed = duration_cast<milliseconds>(end-start).count();
        cout << "\tElapsed gradient descent time: " << time_elapsed << " ms" << endl;
    GridWriter::savePath("test_path3d.txt", grid3, path3D);

    return 0;
}
