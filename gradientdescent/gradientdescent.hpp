/* Assuming 4-connectivity */

#ifndef GRADIENTDESCENT_H_
#define GRADIENTDESCENT_H_

#include <iostream>
#include <vector>
#include <array>
//#include <limits>
#include <cmath>


//#include "../console/console.h"
#include "../ndgridmap/ndgridmap.hpp"
#include "../fmdata/fmcell.h"

#include <fstream>

// TODO: check if points fall in obstacles.


typedef typename std::array<double, 2> Point2D;
typedef typename std::vector <Point2D> Path2D;

constexpr double PI  = 3.141592653589793238462643383;

class GradientDescent {

	typedef typename std::array<int, 2> Coord2D;

    public: 
        GradientDescent() {}; // Default constructor not used.    
        virtual ~GradientDescent() {};  
                 
         /** 
          * Computes the 2D path from the given index to a minimum (the one
          * gradient descent choses).
          * 
          * No checks are done (points in the borders, points in obstacles...).
          * 
          * The included scripts will parse the saved path.
          * 
          * IMPORTANT NOTE: both minimum and initial index cannot be in the
          * border of the map. This situation is not checked. We recommend to set a 1 pixel
          * black border around the map image.
          * 
          * @param grid the grid to apply gradient descent on.
          * @param idx index of the initial path point.
          * @param path the resulting path (output).
          */
        static void apply2D
        (nDGridMap<FMCell,2> & grid, int &  idx, Path2D & path) {
			
			Coord2D current_coord;
			grid.idx2coord(idx, current_coord);
			Point2D current_point = {	static_cast<double>(current_coord[0]), 
										static_cast<double>(current_coord[1]) };
			path.push_back(current_point);
			
			Coord2D d = grid.getDimSizes();

			while(grid[idx].getArrivalTime() != 0) {
				
				// Simplest approximated gradient
				double grad_x = - grid[idx-1].getValue()/2 + grid[idx+1].getValue()/2;
				double grad_y = - grid[idx-d[0]].getValue()/2 + grid[idx+d[0]].getValue()/2;
				
				double alpha = atan2(grad_y,grad_x);
				
				current_point = {	current_point[0] - cos(alpha),
									current_point[1] - sin(alpha) };		
				path.push_back(current_point);
				
				current_coord = {	static_cast<int>(current_point[0]), 
									static_cast<int>(current_point[1]) };
				grid.coord2idx(current_coord,idx);
			}
		}
		
        
    protected:
    
    private:
       
};


#endif /* GRADIENTDESCENT_H_*/
