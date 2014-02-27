/* Assuming 4-connectivity */

#ifndef GRADIENTDESCENT_H_
#define GRADIENTDESCENT_H_

#include <iostream>
#include <vector>
#include <limits>
#include <cmath>
#include <array>

#include "../console/console.h"
#include "../ndgridmap/ndgridmap.hpp"
#include "../fmdata/fmcell.h"

// TODO: check if points fall in obstacles.

class GradientDescent {

    public: 
        GradientDescent() {}; // Default constructor not used.    
        virtual ~GradientDescent() {};  
        
        /*static void apply2D
        (nDGridMap<FMCell,2> & grid, const int & idx, std::vector<int> & path) {
			
			if (grid.getNDims() != 2)
				console::error("Not possible to apply 2D gradient descent. Grid dimensions != 2.");
				
			else {
				std::array<int,2> new_coords;
				std::array<int,2>  coords;
				path.push_back(idx);
				
				if (grid[idx].getValue() == 0) {
					console::warning("The final path point is equal to the initial point.");
					return;
				}
				
				std::array<int,8> neighs;
				int new_idx;
				int current_idx = idx;
				do {
					// Assuming that we are not in any border of the map. Not in the beginning nor the end.
					grid.getNeighbours8c2D(current_idx, neighs, false);

					double grad_x =-2*grid[neighs[0]].getValue()+2*grid[neighs[1]].getValue()
									 -grid[neighs[4]].getValue()+  grid[neighs[5]].getValue()
									 -grid[neighs[6]].getValue()+  grid[neighs[7]].getValue();
					double grad_y =-2*grid[neighs[2]].getValue()+2*grid[neighs[3]].getValue()
									 -grid[neighs[4]].getValue()+  grid[neighs[6]].getValue()
									 -grid[neighs[5]].getValue()+  grid[neighs[7]].getValue();
					double mod = sqrt(grad_x*grad_x+grad_y*grad_y);
					double alpha = atan2(-grad_y,-grad_x);
					
					std::cout << grad_x <<  "  " << grad_y << "  " << mod << "  " << alpha << std::endl;

					grid.idx2coord(current_idx,coords);
					new_coords[0] = round(coords[0]+mod*cos(alpha));
					new_coords[1] = round(coords[1]+mod*sin(alpha));
					grid.coord2idx(new_coords, new_idx);


					path.push_back(new_idx);
					current_idx = new_idx;
					std::cout << current_idx << std::endl;
				} while (grid[current_idx].getValue() != 0);
			}
		}*/
		
		
        
    protected:
    
    private:
       
};


#endif /* GRADIENTDESCENT_H_*/
