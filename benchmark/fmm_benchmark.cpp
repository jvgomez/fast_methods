#include <iostream>
#include <cmath>
#include <array>
#include <string>

#include <boost/variant.hpp>

#include "../fmm/fmdata/fmcell.h"
#include "../ndgridmap/ndgridmap.hpp"

#include "../fmm/fmm.hpp"
#include "../fmm/fim.hpp"
#include "../fmm/gmm.hpp"
#include "../fmm/ufmm.hpp"

#include "../fmm/fmdata/fmfibheap.hpp"
#include "../fmm/fmdata/fmdaryheap.hpp"
#include "../fmm/fmdata/fmpriorityqueue.hpp"

#include "../benchmark/benchmark.hpp"
#include "../benchmark/benchmarkcfg.hpp"

using namespace std;

int main(int argc, const char ** argv)
{
    if (argc < 2)
    {
        std::cerr << "Usage:\n\t " << argv[0] << " problem.cfg" << std::endl;
        return 1;
    }

    BenchmarkCFG bcfg;
    if (bcfg.readOptions(argv[1]))
    {
        if(bcfg.getValue<std::string>("grid.cell") == "FMCell")
        {
            switch (bcfg.getValue<unsigned int>("grid.ndims"))
            {
                case 2:
                {
                    Benchmark<nDGridMap<FMCell,2> > b;
                    bcfg.configure<nDGridMap<FMCell,2>, FMCell>(b);
                    b.run();
                    break;
                }
                case 3:
                {
                    //Benchmark<nDGridMap<FMCell,3> > b;
                    //bcfg.configure(b);
                    //b.run();
                    break;
                }
                // Include here new dimensions.
            }
        }
        else
        {
            // Include here new celltypes and include the corresponding switch dimensions as for FMCell.
        }
    }
}
