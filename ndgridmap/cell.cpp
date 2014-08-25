#include "cell.h"
#include "../console/console.h"

using namespace std;

ostream& operator << 
(ostream & os, Cell & c) {
	os << console::str_info("Basic cell information:");
	os << "\t" << "Index: " 	<< c.index_ << endl;
	os << "\t" << "Value: " 	<< c.value_ << endl;
	return os;
}

