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
#include "io/maploader.hpp"
#include "io/gridplotter.hpp"
#include "io/gridwriter.hpp"
#include "gradientdescent/gradientdescent.hpp"

using namespace std;
using namespace std::chrono;

int main(int argc, const char ** argv)
{
	constexpr int ndims = 2; // Setting two dimensions.
	constexpr int ndims3 = 3; // Setting three dimensions.
	
	time_point<std::chrono::system_clock> start, end; // Time measuring.
	
	console::info("Parsing input arguments.");
	string filename1, filename2, filename_vels;
	console::parseArguments(argc,argv, "-map1", filename1);
	console::parseArguments(argc,argv, "-map2", filename2);
	console::parseArguments(argc,argv, "-vel", filename_vels);
	
	
	console::info("Creating grid from image.");
	nDGridMap<FMCell, ndims> grid;
	/*MapLoader::loadMapFromImg(filename1.c_str(), grid);
	
	onsole::info("Showing the grid and the mirror effect.");
	GridPlotter::plotMap(grid, 0); // It looks "inverted" because the CImg (0,0) coordinates and the Y orientation.
	GridPlotter::plotMap(grid);*/
	
	
	console::info("Testing Fast Marching Method.");
	MapLoader::loadMapFromImg(filename2.c_str(), grid);
	vector<int> init_points;
	array<int,ndims> coords = {210, 140};
	int idx;
	grid.coord2idx(coords, idx);
	init_points.push_back(idx); // Setting the initial point.
	
	FastMarching<FMCell, ndims> fmm;
	fmm.setEnvironment(&grid);
		start = system_clock::now();
	fmm.setInitialPoints(init_points);
	fmm.computeFM();
		end = system_clock::now();
		double time_elapsed = duration_cast<milliseconds>(end-start).count();
		cout << "\tElapsed FM time: " << time_elapsed << " ms" << endl;
		
	console::info("Plotting the results ");
	//GridPlotter::plotArrivalTimes(grid);
	GridWriter::saveGridValues("test_fm.txt", grid);
	
	console::info("Computing gradient descent ");
	int goal;
	grid.coord2idx(std::array<int, ndims>{250,280}, goal);
	
	Path2D path;
		start = system_clock::now();
	GradientDescent::apply2D(grid,goal,path);
		end = system_clock::now();
		time_elapsed = duration_cast<milliseconds>(end-start).count();
		cout << "\tElapsed gradient descent time: " << time_elapsed << " ms" << endl;
	GridWriter::savePath2D("test_path.txt", grid, path);
	GridPlotter::plotMapPath(grid,path);

	
	
	/*
	console::info("Now using all black points as wave sources");
	nDGridMap<FMCell, ndims> grid2;
	init_points.clear();
	MapLoader::loadMapFromImg(filename2.c_str(), grid2, init_points); // This is the only thing that changes.
	
	FastMarching<FMCell, ndims> fmm2;
	fmm2.setEnvironment(&grid2);
		start = system_clock::now();
	fmm2.setInitialPoints(init_points);
	fmm2.computeFM();
		end = system_clock::now();
		 time_elapsed = duration_cast<milliseconds>(end-start).count();
		cout << "\tElapsed FM time: " << time_elapsed << " ms" << endl;
		
	console::info("Plotting the results ");
	GridPlotter::plotArrivalTimes(grid2);
	
	console::info("Saving into file test_fm.txt");
	GridWriter::saveGridValues("test_fm.txt", grid2);
	
	console::info("Now let's try different velocities.");
	nDGridMap<FMCell, ndims> grid_vels;
	MapLoader::loadVelocitiesFromImg(filename_vels.c_str(), grid_vels);
	FastMarching<FMCell, ndims> fmm_vels;
	init_points.clear();
	init_points.push_back(80000);
	fmm_vels.setEnvironment(&grid_vels);
		start = system_clock::now();
	fmm_vels.setInitialPoints(init_points);
	fmm_vels.computeFM();
		end = system_clock::now();
		 
		 time_elapsed = duration_cast<milliseconds>(end-start).count();
		cout << "\tElapsed FM time: " << time_elapsed << " ms" << endl;
		
	console::info("Plotting the results ");
	GridPlotter::plotArrivalTimes(grid_vels);
	
	console::info("Saving velocities");
	GridWriter::saveVelocities("test_vels.txt", grid_vels);
	
	console::info("Testing 3D!");
	nDGridMap<FMCell, ndims3> grid3 (std::array<int,ndims3>({100,100,50}));
	init_points.clear();
	grid3.coord2idx(std::array<int,ndims3>({50,50,25}),idx); // Reusing the previous int.
	init_points.push_back(idx);
	FastMarching<FMCell, ndims3> fmm3;
	fmm3.setEnvironment(&grid3);
		start = system_clock::now();
	fmm3.setInitialPoints(init_points);
	fmm3.computeFM();
		end = system_clock::now();
		 time_elapsed = duration_cast<milliseconds>(end-start).count();
		cout << "\tElapsed FM time: " << time_elapsed << " ms" << endl;
		
	console::info("Saving into file test_fm3d.txt");
	GridWriter::saveGridValues("test_fm3d.txt", grid3);
		
		*/
    return 0;
}
