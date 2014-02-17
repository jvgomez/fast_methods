#include <iostream>
#include <cmath>
#include <chrono>


#include "ndgridmap/ndgridmap.hpp"
#include "fmdata/fmcell.h"
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
	//int init_point = atoi(argv[ndims+1]);
	int source_point = atoi(argv[ndims+1]);
	
	vector<int> dimsize;
	
	for (int i = 0; i < ndims; ++i)
		dimsize.push_back(atoi(argv[i+1]));
	
	nDGridMap<FMCell> * grid = new nDGridMap<FMCell>(ndims, dimsize, 0.10);
	
	/*vector<int> c(2);
	cout << "Hasta aqui" << endl;
	grid.idx2coord(12, c);
	cout << c[0] << "\t" << c[1] << endl;
	grid.idx2coord(15, c);
	cout << c[0] << "\t" << c[1] << endl;
	grid.idx2coord(1, c);
	cout << c[0] << "\t" << c[1] << endl;*/
	
	//vector<int> neighs;
	//grid.getNeighbours8c3D(atoi(argv[argc-1]), neighs);
	//grid.getNeighbours8c2D(atoi(argv[argc-1]), neighs, false);
	
	/*cout << neighs.size() << endl << endl;
	for (int &i: neighs)
		cout << i << endl;*/
	
		
	vector<int> init_points;
	init_points.push_back(source_point); // 300x300 grid middle point
	//init_points.push_back(7000); 
	
	FastMarching<FMCell> fmm;
	fmm.setEnvironment(grid);
	fmm.setInitialPoints(init_points);

		start = system_clock::now();
	fmm.init();
	fmm.computeFM();

		end = system_clock::now();
		double time_elapsed = duration_cast<milliseconds>(end-start).count();
		cout << "Elapsed FM time: " << time_elapsed << " ms" << endl;
	
/*		start = system_clock::now();
	vector<int> path;
	fmm.computeGeodesic(init_point,path);
		end = system_clock::now();
		time_elapsed = duration_cast<milliseconds>(end-start).count();
		cout << "Elapsed GD time: " << time_elapsed << " ms" << endl;
*/
	fmm.saveGrid("test_velocity.txt", 1);
	fmm.saveGrid("test_at.txt", 0);	
	//fmm.saveGeodesic("test_path.txt", path);
    return 0;
}
