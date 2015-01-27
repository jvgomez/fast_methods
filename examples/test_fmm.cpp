#include <iostream>
#include <cmath>
#include <chrono>
#include <array>
#include <string>

#include "../fmm/fmdata/fmcell.h"
#include "../ndgridmap/ndgridmap.hpp"
#include "../console/console.h"

#include "../fmm/fastmarching.hpp"
#include "../fmm/fim.hpp"
#include "../fmm/groupmarching.hpp"

#include "../fmm/fmdata/fmfibheap.hpp"
#include "../fmm/fmdata/fmdaryheap.hpp"
#include "../fmm/fmdata/fmpriorityqueue.hpp"

#include "../gradientdescent/gradientdescent.hpp"

#include "../io/gridwriter.hpp"

using namespace std;
using namespace std::chrono;

int main(int argc, const char ** argv)
{
    constexpr int ndims2 = 2; // Setting two dimensions.

    // A bit of shorthand.
    typedef nDGridMap<FMCell, ndims2> FMGrid2D;
    typedef array<int, ndims2> Coord2D;

    time_point<std::chrono::system_clock> start, end; // Time measuring.
    double time_elapsed;

    Coord2D dimsize {300,300};
    FMGrid2D grid_fmm_fib (dimsize);
    FMGrid2D grid_fmm (dimsize);
    FMGrid2D grid_sfmm (dimsize);

    Coord2D init_point = {150, 150};
    vector<int> init_points;
    int idx;
    grid_fmm.coord2idx(init_point , idx);
    init_points.push_back(idx);

    std::vector<Solver<FMGrid2D>*> solvers;
    solvers.push_back(new FastMarching<FMGrid2D>);
    solvers.push_back(new FastIterativeMethod<FMGrid2D>);
    //solvers.push_back(new GroupMarching<FMGrid2D>);

    for (const Solver<FMGrid2D>* s :solvers)
    {
        std::cout << s->getName() << '\n';
    }

    /*FastMarching<FMGrid2D, FMFibHeap<FMCell> > fmm_fib;
    fmm_fib.setEnvironment(&grid_fmm_fib);
        start = system_clock::now();
    fmm_fib.setInitialPoints(init_points);
    fmm_fib.computeFM();
        end = system_clock::now();
        time_elapsed = duration_cast<milliseconds>(end-start).count();
        cout << "\tElapsed FMM - Fibonacci heap time: " << time_elapsed << " ms" << endl;
    //GridWriter::saveGridValues("test_fmm.txt", grid_fmm_fib);*/

    FastMarching<FMGrid2D> fmm;
    fmm.setEnvironment(&grid_fmm);
        start = system_clock::now();
    fmm.setInitialPoints(init_points);
    fmm.compute();
        end = system_clock::now();
        time_elapsed = duration_cast<milliseconds>(end-start).count();
        cout << "\tElapsed FMM_Dary time: " << time_elapsed << " ms" << endl;
        //GridWriter::saveGridValues("test_fmmdary.txt", grid_fmm);

    // Using priority queue implies the use of the SFMM. Priority queue uses by default FMCell.
    /*FastMarching<FMGrid2D, FMPriorityQueue<> > sfmm; //Choosing the default cell class.
    sfmm.setEnvironment(&grid_sfmm);
        start = system_clock::now();
    sfmm.setInitialPoints(init_points);
    sfmm.computeFM();
        end = system_clock::now();
        time_elapsed = duration_cast<milliseconds>(end-start).count();
        cout << "\tElapsed SFMM time: " << time_elapsed << " ms" << endl;
    //GridWriter::saveGridValues("test_sfmm.txt", grid_sfmm);*/
}
