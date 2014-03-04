#include <iostream>
#include <cmath>
#include <chrono>
#include <array>
#include <string>

#include "../fmdata/fmcell.h"
#include "../ndgridmap/ndgridmap.hpp"
#include "../console/console.h"
#include "../fmm/sfmm.hpp"
#include "../fmm/fmm_dary.hpp"

//#include "io/maploader.hpp"
//#include "../io/gridplotter.hpp"
#include "../io/gridwriter.hpp"
//#include "gradientdescent/gradientdescent.hpp"

using namespace std;
using namespace std::chrono;

int main(int argc, const char ** argv)
{
	constexpr int ndims2 = 2; // Setting two dimensions.
	
	time_point<std::chrono::system_clock> start, end; // Time measuring.
	double time_elapsed;
	
	array<int, ndims2> dimsize = {300,300};
	nDGridMap<FMCell, ndims2> grid_fmm (dimsize);
	nDGridMap<FMCell, ndims2> grid_sfmm (dimsize);
	nDGridMap<FMCell, ndims2> grid_fmm_dary (dimsize);
	
	vector<int> init_points;
	int idx;
	grid_fmm.coord2idx(array<int,ndims2>{150,150} , idx);
	init_points.push_back(idx);
	
	

	FastMarching<FMCell, ndims2> fmm;
	fmm.setEnvironment(&grid_fmm);
			start = system_clock::now();
	fmm.setInitialPoints(init_points);
	fmm.computeFM();
		end = system_clock::now();
		time_elapsed = duration_cast<milliseconds>(end-start).count();
		cout << "\tElapsed FMM time: " << time_elapsed << " ms" << endl;
		
		
	FMM_Dary<FMCell, ndims2> fmm_dary;
	fmm_dary.setEnvironment(&grid_fmm_dary);	
			start = system_clock::now();
	fmm_dary.setInitialPoints(init_points);
	fmm_dary.computeFM();
		end = system_clock::now();
		time_elapsed = duration_cast<milliseconds>(end-start).count();
		cout << "\tElapsed FMM_Dary time: " << time_elapsed << " ms" << endl;
	
	SFMM<FMCell, ndims2> sfmm;
	sfmm.setEnvironment(&grid_sfmm);	
			start = system_clock::now();
	sfmm.setInitialPoints(init_points);
	sfmm.computeFM();
		end = system_clock::now();
		time_elapsed = duration_cast<milliseconds>(end-start).count();
		cout << "\tElapsed SFFM time: " << time_elapsed << " ms" << endl;
		
		
	//console::info("Saving into test_fm.txt");
	//GridWriter::saveGridValues("test_fmm.txt", grid);
	
	
}
	
	
	
