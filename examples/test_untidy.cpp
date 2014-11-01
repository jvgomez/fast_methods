#include <iostream>
#include <cmath>
#include <chrono>
#include <array>
#include <string>

#include "../fmdata/fmuntidycell.h"
#include "../ndgridmap/ndgridmap.hpp"
#include "../console/console.h"

#include "../ufmm/fmm_untidy.hpp"

#include "../io/gridwriter.hpp"

using namespace std;
using namespace std::chrono;

int main(int argc, const char ** argv)
{

	constexpr int ndims2 = 2;

	typedef nDGridMap<FMUntidyCell, ndims2> FMGrid2D;
	typedef array<int, ndims2> Coord2D;

	time_point<std::chrono::system_clock> start, end; // Time measuring.
	double time_elapsed;

	Coord2D dimsize = {1000,1000};
	FMGrid2D grid_ufmm (dimsize);

	Coord2D init_point = {500,500};
	vector<int> init_points;
	int idx;
	grid_ufmm.coord2idx(init_point , idx);
	init_points.push_back(idx);

	FMM_Untidy<FMGrid2D> ufmm;
	ufmm.setEnvironment(&grid_ufmm);
		start = system_clock::now();
	ufmm.setInitialPoints(init_points);
	ufmm.computeFM();
		end = system_clock::now();
		time_elapsed = duration_cast<milliseconds>(end-start).count();
		cout << "\tElapsed UFMM time: " << time_elapsed << " ms" << endl;

	GridWriter::saveGridValues("test_fmm_untidy.txt", grid_ufmm);
}
