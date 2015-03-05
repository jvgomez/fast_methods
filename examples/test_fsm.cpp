/* */

#include <iostream>
#include <array>
#include <chrono>

#include "../ndgridmap/ndgridmap.hpp"
#include "../ndgridmap/cell.h"

#define TRES

// Implement exponentiation by squaring: http://stackoverflow.com/questions/101439/the-most-efficient-way-to-implement-an-integer-based-power-function-powint-int
using namespace std;


#ifdef TRES
    constexpr unsigned int ndims = 3;
#else
    constexpr unsigned int ndims = 2;
#endif


// A bit of shorthand.
typedef nDGridMap<Cell, ndims> Grid2D;
typedef array<unsigned int, ndims> Coord2D;

#ifdef TRES
    Coord2D dimsize_ = {300,300,300};
#else
    Coord2D dimsize_ = {300,300};
#endif

Grid2D grid (dimsize_);

// Generate increments, begins and ends
void setSweep(unsigned int sweep, array<int, ndims> & increment, array<int,ndims> & inits, array<int,ndims> & ends)
{
    increment[0] *= -1; // Dimension 0 changes sweep direction every time.
    if (increment[0] == 1)
    {
        inits[0] = 0;
        ends[0] = dimsize_[0];
    }
    else
    {
        inits[0] = dimsize_[0]-1;
        ends[0] = -1;
    }

    for (size_t i = 1; i < ndims; ++i)
    {
        unsigned it = sweep % unsigned(pow(2,i+1));
        if (it > pow(2,i)-1)
            increment[i] = -1;
        else
            increment[i] = 1;

        if (increment[i] == 1)
        {
            inits[i] = 0;
            ends[i] = dimsize_[i];
        }
        else
        {
            inits[i] = dimsize_[i]-1;
            ends[i] = -1;
        }
    }
}


int main()
{
#ifdef TRES
    array<int, ndims> increment{-1,-1,-1};
#else
    array<int, ndims> increment{-1,-1};
#endif

    array<int, ndims> inits, ends;

    // Sweeping
    unsigned int sweep = 0;
    bool keepSweeping = true;
    unsigned idx = 0;

    chrono::time_point<chrono::steady_clock> start, end;

    start = chrono::steady_clock::now();

    while (keepSweeping)
    {
        setSweep(sweep, increment, inits, ends);

/*
#ifdef TRES
    cout << "Current increments: " << increment[0] << "  " << increment[1] << "  " << increment[2] << '\n';
#else
    cout << "Current increments: " << increment[0] << "  " << increment[1] << '\n';
    cout << "Inits and ends: " << inits[0] << "  " << inits[1] << "  " << ends[0] << "  " << ends[1] <<'\n';
#endif

        cout << "Sweeping" <<'\n';
        */
        for (int k = inits[2]; k != ends[2]; k+=increment[2])
        {
            for (int j = inits[1]; j != ends[1]; j+=increment[1])
            {
                for (int i = inits[0]; i != ends[0]; i+=increment[0])
                {
                    idx = k*dimsize_[0]*dimsize_[1] + j *dimsize_[0] + i;
                }
            }
            idx*idx;
        }
        ++sweep;
        //if (sweep > pow(2,ndims)-1)
        if (sweep > pow(2,ndims)-1)
            keepSweeping = false;
    }

    end = chrono::steady_clock::now();
    double time = chrono::duration_cast<chrono::milliseconds>(end-start).count();
    cout << time << endl;
    return 0;
}
