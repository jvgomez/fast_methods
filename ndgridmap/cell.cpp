#include "cell.h"
#include "../console/console.h"

#include <limits>

using namespace std;

ostream& operator <<
(ostream & os, Cell & c) {
    os << console::str_info("Basic cell information:");
    os << "\t" << "Index: " << c.index_ << '\n'
       << "\t" << "Value: " << c.value_ << '\n'
       << "\t" << "Occupancy: " << c.occupancy_ <<'\n';
    return os;
}

void Cell::setDefault
() {
    value_ = -1;
}

bool Cell::isOccupied
() const {
    if (occupancy_ < std::numeric_limits<double>::epsilon() * 1e3)
        return true;
    return false;
}
