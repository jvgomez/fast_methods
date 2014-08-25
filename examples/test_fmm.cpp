#include <iostream>
#include <cmath>
#include <chrono>
#include <array>
#include <string>

#include "../fmdata/fmcell.h"
#include "../ndgridmap/ndgridmap.hpp"
#include "../console/console.h"

#include "../fmm/fastmarching.hpp"

#include "../fmdata/fmfibheap.hpp"
#include "../fmdata/fmdaryheap.hpp"
#include "../fmdata/fmpriorityqueue.hpp"

#include "../gradientdescent/gradientdescent.hpp"

#include "../io/gridwriter.hpp"

using namespace std;
using namespace std::chrono;


int main(int argc, const char ** argv)
{
	constexpr int ndims2 = 2; // Setting two dimensions.
	constexpr int ndims3 = 3; // Setting three dimensions.

	typedef nDGridMap<FMCell, ndims2> FMGrid2D;
	typedef array<int, ndims2> Coord2D;

	time_point<std::chrono::system_clock> start, end; // Time measuring.
	double time_elapsed;

	Coord2D dimsize = {300,300};
	FMGrid2D grid_fmm (dimsize);
	FMGrid2D grid_fmm_dary (dimsize);
	FMGrid2D grid_sfmm (dimsize);

	Coord2D init_point = {150, 150};
	vector<int> init_points;
	int idx;
	grid_fmm.coord2idx(init_point , idx);
	init_points.push_back(idx);

	FastMarching<FMGrid2D, FMFibHeap<FMCell> > fmm;
	fmm.setEnvironment(&grid_fmm);
		start = system_clock::now();
	fmm.setInitialPoints(init_points, -1);
	fmm.computeFM();
		end = system_clock::now();
		time_elapsed = duration_cast<milliseconds>(end-start).count();
		cout << "\tElapsed FMM time: " << time_elapsed << " ms" << endl;
	//GridWriter::saveGridValues("test_fmm.txt", grid_fmm);

	FastMarching<FMGrid2D> fmm_dary;
	fmm_dary.setEnvironment(&grid_fmm_dary);
		start = system_clock::now();
	fmm_dary.setInitialPoints(init_points, -1);
	fmm_dary.computeFM();
		end = system_clock::now();
		time_elapsed = duration_cast<milliseconds>(end-start).count();
		cout << "\tElapsed FMM_Dary time: " << time_elapsed << " ms" << endl;
		//GridWriter::saveGridValues("test_fmmdary.txt", grid_fmm_dary);

	// Using priority queue implies the use of the SFMM. Priority queue uses by default FMCell.
	FastMarching<FMGrid2D, FMPriorityQueue<> > sfmm; //Choosing the default cell class.
	sfmm.setEnvironment(&grid_sfmm);
		start = system_clock::now();
	sfmm.setInitialPoints(init_points, -1);
	sfmm.computeFM();
		end = system_clock::now();
		time_elapsed = duration_cast<milliseconds>(end-start).count();
		cout << "\tElapsed SFMM time: " << time_elapsed << " ms" << endl;
	//GridWriter::saveGridValues("test_sfmm.txt", grid_sfmm);
}
