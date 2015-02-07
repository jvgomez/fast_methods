#include "fmcell.h"
#include "../../console/console.h"

using namespace std;

ostream& operator << 
(ostream & os, const FMCell & c) {
    os << console::str_info("Fast Marching cell information:");
    os << "\t" << "Index: " << c.index_ << endl;
    os << "\t" << "Value: " << c.value_ << endl;
    os << "\t" << "Velocity: " << c.velocity_ << endl;
    os << "\t" << "State: " ;

    switch (c.state_) {
        case FMState::OPEN:
            os << "OPEN" << endl;
            break;
        case FMState::NARROW:
            os << "NARROW" << endl;
            break;
        case FMState::FROZEN:
            os << "FROZEN" << endl;
            break;
        }
    return os;
}

bool FMCell::isObstacle
() const {
    return occupancy_ == 0;
}

void FMCell::setDefault
() {
    Cell::setDefault();
    value_ = std::numeric_limits<double>::infinity();
    if (!occupancy_) velocity_ = 0;
    else             velocity_ = 1;
    state_ = FMState::OPEN;
}
