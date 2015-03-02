/* */

#include <iostream>
#include <array>

#include "../ndgridmap/ndgridmap.hpp"
#include "../ndgridmap/cell.h"

using namespace std;

// A bit of shorthand.
typedef nDGridMap<Cell, 2> Grid2D;
typedef array<unsigned int, 2> Coord2D;

bool terminateSweep(const Grid2D & grid, bool sweep, int idx) {
    if (sweep%2 == 1 && idx > grid.size())
        return true;
    return false;
}

int main()
{
    Coord2D dimsize_ = {3,3};
    Grid2D grid (dimsize_);

    unsigned int sweep = 1;

    array<unsigned int, 2> d_;
    size_t ncells_ = 1;
    for (unsigned int i = 0; i < 2; ++i) {
        ncells_ *= dimsize_[i];
        d_[i] = ncells_;
    }

    bool keepSweeping = true, stopSweep = false;
    unsigned int idx = 0;
    int inc = 1;
    while (keepSweeping) {
        while (!stopSweep) {
            cout << idx << "  ";
            idx += inc;
            if (terminateSweep(grid, sweep, idx))
                break;
        }
        cout << "\nSweep finished" << "\n";
        keepSweeping = false;
    }


    return 0;
}
