#include <iostream>
#include <cmath>
#include <chrono>

#include "fmdata/fmcell.h"
#include "ndgridmap/ndgridmap.hpp"

#include "console/console.h"

#include "fmm/fastmarching.hpp"

using namespace std;
using namespace std::chrono;

int main(int argc, const char ** argv)
{
	
	time_point<std::chrono::system_clock> start, end;
	
	console::info("Testing Fast Marching Method.");
	
	// Asigning input parameters.
	int ndims = argc-2;
	int source_point = atoi(argv[ndims+1]);
	
	vector<int> dimsize;
	
	for (int i = 0; i < ndims; ++i)
		dimsize.push_back(atoi(argv[i+1]));
	
	nDGridMap<FMCell> * grid = new nDGridMap<FMCell>(ndims, dimsize, 0.10);
	
		
	vector<int> init_points;
	init_points.push_back(source_point);
	
	FastMarching<FMCell> fmm;
	fmm.setEnvironment(grid);
	fmm.setInitialPoints(init_points);

		start = system_clock::now();
	fmm.init();
	fmm.computeFM();

		end = system_clock::now();
		double time_elapsed = duration_cast<milliseconds>(end-start).count();
		cout << "Elapsed FM time: " << time_elapsed << " ms" << endl;
	
	fmm.saveGrid("test_velocity.txt", 1);
	fmm.saveGrid("test_at.txt", 0);	

    return 0;
}
