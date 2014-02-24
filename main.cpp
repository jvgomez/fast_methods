#include <iostream>
#include <cmath>
#include <chrono>
#include <array>

#include "fmdata/fmcell.h"
#include "ndgridmap/ndgridmap.hpp"

#include "console/console.h"

#include "fmm/fastmarching.hpp"

#include "io/maploader.hpp"
#include "io/gridplotter.hpp"

using namespace std;
using namespace std::chrono;

int main(int argc, const char ** argv)
{
	
	console::info("Testing Loading from Image.");
	
	constexpr int ndims = 2;
	const char * filename = argv[1];
	
	nDGridMap<FMCell, ndims> grid;
	
	MapLoader loader;
	vector<int> init_points;
	//loader.loadMapFromImg(filename, grid, init_points);
	loader.loadMapFromImg(filename, grid);
	
	
	console::info("Testing Fast Marching Method.");
	
	time_point<std::chrono::system_clock> start, end;

	init_points.push_back(80000);
	FastMarching<FMCell, ndims> fmm;
	fmm.setEnvironment(&grid);
	fmm.setInitialPoints(init_points);

		start = system_clock::now();
	fmm.init();
	fmm.computeFM();

		end = system_clock::now();
		double time_elapsed = duration_cast<milliseconds>(end-start).count();
		cout << "Elapsed FM time: " << time_elapsed << " ms" << endl;
	
	//fmm.saveGrid("test_velocity.txt", 1);
	fmm.saveGrid("test_at.txt", 0);	
	
	GridPlotter plotter;
	plotter.plotArrivalTimes(grid);

    return 0;
}
