/* Assuming 4-connectivity */

#ifndef GRADIENTDESCENT_H_
#define GRADIENTDESCENT_H_

#include <iostream>
#include <vector>
#include <limits>
#include <cmath>
//#include <string>
//#include <fstream>
//#include <algorithm>

#include "../console/console.h"
#include "../ndgridmap/ndgridmap.hpp"
#include "../fmdata/fmcell.h"

template <class T> class GradientDescent {

    public: 
        GradientDescent<T>() {}; // Default constructor not used.    
        virtual ~GradientDescent<T>() {};  
        
        static void apply2D
        (nDGridMap<FMCell> & grid, const int & idx, std::vector<int> & path) {
			
			if (grid.getNDims() != 2)
				console::error("Not possible to apply 2D gradient descent. Grid dimensions != 2.");
				
			else {
				std::vector<int> c(2);
				grid.idx2coord(idx,c);
				std::cout << c[0] << "\t\t" << c[1] << std::endl;
				
				std::vector<int> new_coords(2);
				std::vector<int> coords(2);
				coords.resize(2);
				new_coords.resize(2);
				path.push_back(idx);
				
				if (grid[idx].getValue() == 0) {
					console::warning("The final path point is equal to the initial point.");
					return;
				}
				
				std::vector<int> neighs;
				int new_idx;
				int current_idx = idx;
				float minValue = std::numeric_limits<float>::infinity();
				do {
					grid.getNeighbours8c2D(current_idx, neighs, false);
					//for (int & i: neighs)
					//	grid.showCoords(i);

					float grad_x =-2*grid[neighs[0]].getValue()+2*grid[neighs[1]].getValue()
									-grid[neighs[4]].getValue()+  grid[neighs[5]].getValue()
									-grid[neighs[6]].getValue()+  grid[neighs[7]].getValue();
					float grad_y =-2*grid[neighs[2]].getValue()+2*grid[neighs[3]].getValue()
									-grid[neighs[4]].getValue()+  grid[neighs[5]].getValue()
									-grid[neighs[6]].getValue()+  grid[neighs[7]].getValue();
					//float mod = sqrt(grad_x*grad_x+grad_y*grad_y);
					float alpha = atan2(-grad_y,-grad_x);
					
					std::cout << grad_x << "\t" << grad_y << "\t" << alpha*180/3.141592 << std::endl;

					grid.idx2coord(current_idx,coords);
					new_coords[0] = round(coords[0]+cos(alpha));
					new_coords[1] = round(coords[1]+sin(alpha));
					grid.coord2idx(new_coords, new_idx);
					std::cout << new_coords[0] << "\t\t" << new_coords[1] << std::endl;

					
					/*for (int &i: neighs) {
						if (grid[i].getValue() < minValue) {
							minValue = grid[i].getValue();
							new_idx = i;
						}
					}*/
					path.push_back(new_idx);
					neighs.clear();
					current_idx = new_idx;
					std::cout << "Avance" << std::endl;
				} while (grid[current_idx].getValue() != 0);
			}
		}
		
		
        static void apply3D
        (nDGridMap<FMCell> & grid, const int & idx, std::vector<int> & path) {
			
			if (grid.getNDims() != 3)
				console::error("Not possible to apply 3D gradient descent. Grid dimensions != 3.");
				
			else {
				path.push_back(idx);
				
				if (grid[idx].getValue() == 0) {
					console::warning("The final path point is equal to the initial point.");
					return;
				}
				
				std::vector<int> neighs;
				int new_idx;
				int current_idx = idx;
				float minValue = std::numeric_limits<float>::infinity();
				do {
					grid.getNeighbours8c3D(current_idx, neighs);
					
					for (int &i: neighs) {
						if (grid[i].getValue() < minValue) {
							minValue = grid[i].getValue();
							new_idx = i;
						}
					}
					path.push_back(new_idx);
					neighs.clear();
					current_idx = new_idx;
				} while (grid[current_idx].getValue() != 0);
			}
		}
        
    protected:
       
};


#endif /* GRADIENTDESCENT_H_*/
