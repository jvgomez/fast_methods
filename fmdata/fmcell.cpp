#include "fmcell.h"

#include "../console/console.h"

using namespace std;

inline bool FMCell::isObstacle 
() const {
    if (velocity_ == 0)
        return true;
    else
        return false;
}

float FMCell::getValue
(const int whattosave) {
	float return_val;
	switch (whattosave) {
		case 0:
			return_val = value_;
			break;
		case 1:
			return_val = velocity_;
			break;
	}
	
}

ostream& operator << 
(ostream & os, FMCell & c) {
	os << console::str_info("Fast Marching cell information:");
	os << "\t" << "Index: " << c.index_ << endl;
	os << "\t" << "Value: " 	<< c.value_ << endl;
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
}

