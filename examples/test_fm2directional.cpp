/* n dimensional Fast Marching example with the main functions used */

#include <iostream>
#include <cmath>
#include <chrono>
#include <array>
#include <string>
#include <algorithm>

#include "../fmdata/fmcell.h"
#include "../ndgridmap/ndgridmap.hpp"
#include "../console/console.h"
#include "../fm2directional/fm2directional.hpp"
#include "../fmdata/fmfibheap.hpp"
#include "../fmdata/fmpriorityqueue.hpp"
#include "../fmdata/fmdaryheap.hpp"
#include "../fmdata/fmdaryheap.hpp"
#include "../io/maploadertext.hpp"

using namespace std;
using namespace std::chrono;


int main(int argc, const char ** argv)
{
	constexpr int ndims2 = 2; // Setting two dimensions.

    console::info("Parsing input arguments.");
    string filename;
    console::parseArguments(argc,argv, "-map", filename);

	typedef nDGridMap<FMDirectionalCell, ndims2> FMGrid2D;
	typedef array<int, ndims2> Coord2D;

	time_point<std::chrono::system_clock> start, end; // Time measuring.
	double time_elapsed;

    FMGrid2D grid_fmm;
    FMGrid2D grid_fmm_dary;
    FMGrid2D grid_sfmm;

    vector<int> fm2_sources;

    MapLoaderText::loadMapFromText(filename.c_str(), grid_fmm, fm2_sources);
    MapLoaderText::loadMapFromText(filename.c_str(), grid_fmm_dary, fm2_sources);
    MapLoaderText::loadMapFromText(filename.c_str(), grid_sfmm, fm2_sources);

	Coord2D init_point = {377, 664};
    Coord2D goal_point = {379, 91};
	vector<int> init_points;
	int idx, goal;
	grid_fmm.coord2idx(init_point , idx);
	init_points.push_back(idx);

    grid_fmm.coord2idx(goal_point , goal);

    typedef typename std::vector< std::array<double, ndims2> > Path; // A bit of short-hand.
    Path path;

	FastMarching2Directional<FMGrid2D, Path, FMFibHeap<FMDirectionalCell> > fm2directional;
	fm2directional.setEnvironment(&grid_fmm);
		start = system_clock::now();
	fm2directional.setInitialAndGoalPoints(init_points, fm2_sources, goal);
	fm2directional.computeFM2Directional();
		end = system_clock::now();
		time_elapsed = duration_cast<milliseconds>(end-start).count();
		cout << "\tElapsed FMM time: " << time_elapsed << " ms" << endl;

	FastMarching2Directional<FMGrid2D, Path> fm2directional_dary;
	fm2directional_dary.setEnvironment(&grid_fmm_dary);
		start = system_clock::now();
	fm2directional_dary.setInitialAndGoalPoints(init_points, fm2_sources, goal);
	fm2directional_dary.computeFM2Directional();
		end = system_clock::now();
		time_elapsed = duration_cast<milliseconds>(end-start).count();
		cout << "\tElapsed FMM_Dary time: " << time_elapsed << " ms" << endl;

	// Using priority queue implies the use of the SFMM. Priority queue uses by default FMCell.
	FastMarching2Directional<FMGrid2D, Path, FMPriorityQueue<> > sfm2directional; //Choosing the default cell class.
	sfm2directional.setEnvironment(&grid_sfmm);
		start = system_clock::now();
	sfm2directional.setInitialAndGoalPoints(init_points, fm2_sources, goal);
	sfm2directional.computeFM2Directional();
		end = system_clock::now();
		time_elapsed = duration_cast<milliseconds>(end-start).count();
		cout << "\tElapsed SFMM time: " << time_elapsed << " ms" << endl;
}
