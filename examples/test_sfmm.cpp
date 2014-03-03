#include <iostream>
#include <cmath>
#include <chrono>
#include <array>
#include <string>

#include "../fmdata/fmcell.h"
#include "../ndgridmap/ndgridmap.hpp"
#include "../console/console.h"
#include "../fmm/sfmm.hpp"

//#include "io/maploader.hpp"
//#include "../io/gridplotter.hpp"
#include "../io/gridwriter.hpp"
//#include "gradientdescent/gradientdescent.hpp"

using namespace std;
using namespace std::chrono;

int main(int argc, const char ** argv)
{
	constexpr int ndims2 = 3; // Setting two dimensions.
	
	time_point<std::chrono::system_clock> start, end; // Time measuring.
	double time_elapsed;
	
	array<int, ndims2> dimsize = {256,256,256};
	nDGridMap<FMCell, ndims2> grid (dimsize);
	
	vector<int> init_points;
	int idx;
	grid.coord2idx(array<int,ndims2>{150, 150,150} , idx);
	init_points.push_back(idx);
	
	
	SFMM<FMCell, ndims2> sfmm;
	//FastMarching<FMCell, ndims2> sfmm;
	sfmm.setEnvironment(&grid);
		start = system_clock::now();
	sfmm.setInitialPoints(init_points);
	sfmm.computeFM();
		end = system_clock::now();
		time_elapsed = duration_cast<milliseconds>(end-start).count();
		cout << "\tElapsed FM time: " << time_elapsed << " ms" << endl;
		
	console::info("Saving into test_fm.txt");
	//GridWriter::saveGridValues("test_fmm.txt", grid);
	
	
}
	
	
	
