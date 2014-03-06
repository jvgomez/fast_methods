#ifndef GRADIENTDESCENT_H_
#define GRADIENTDESCENT_H_

#include <iostream>
#include <vector>
#include <array>
#include <algorithm>
#include <cmath>

#include "../ndgridmap/ndgridmap.hpp"
#include "../fmdata/fmcell.h"

// TODO: check if points fall in obstacles, points in the borders, etc.

template <typename T> double sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

template <class T, size_t ndims> class GradientDescent {

	// Short-hand.
	typedef typename std::array<int, ndims> Coord;
	typedef typename std::array<double, ndims> Point;
	typedef typename std::vector <Point> Path;

    public: 
        GradientDescent() {}; // Default constructor not used.    
        virtual ~GradientDescent() {};  
                 
         /** 
          * Computes the path from the given index to a minimum (the one
          * gradient descent choses). The T class chosen must inherite from Cell class
          * or have a double getValue() function.
          * 
          * Simple gradient approximation is used: dimension 0: gx = f((x-1,y)+f(x+1,y))/2
          * dimension 1: gy = f((x,y-1)+f(x,y+1))/2
          * and so on.
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
          * @param the step size to be applied.
          */
        static void apply
        (nDGridMap<T,ndims> & grid, int &  idx, Path & path, const double step = 1) {
			
			Coord current_coord;
			Point current_point;
			Coord dimsize = grid.getDimSizes();
			double grad_i;
			
			std::array<int, ndims-1> d_; //  Same as nDGridMap class auxiliar array d_.
			d_[0] = dimsize[0];
			for (int i = 1; i < ndims; ++i)
				d_[i] = dimsize[i]*d_[i-1];
			
			grid.idx2coord(idx, current_coord);
			
			for (int i = 0; i < ndims; ++i)
				current_point[i] = static_cast<double>(current_coord[i]);
									
			path.push_back(current_point);

			while(grid[idx].getArrivalTime() != 0) {
				// Every iteration the gradient is computed for all dimensions. If is infinite, we convert it to 1 (keeping the sign).
				// The static_cast are necessary because the conversion between coordinate (we check the value in coordinates) and points
				// (the path is composed by continuous points).
				
				// First dimension done apart.
				grad_i = - grid[idx-1].getValue()/2 + grid[idx+1].getValue()/2;
				if (isinf(grad_i))
					grad_i = sgn<double>(grad_i);
					// Moving the point in dim 0.
				current_point[0] = current_point[0] - step*grad_i;
				current_coord[0] = static_cast<int>(current_point[0]);
				
				// Rest of dimensions.
				for (int i = 1; i < ndims; ++i) {
					grad_i = - grid[idx-d_[i-1]].getValue()/2 + grid[idx+d_[i-1]].getValue()/2;
					
					if (isinf(grad_i))
						grad_i = static_cast<double>( sgn<double>(grad_i) );
						// Moving the point in dim i.
					current_point[i] = current_point[i] - step*grad_i;
					current_coord[i] = static_cast<int>(current_point[i]);
				}
				
				path.push_back(current_point);
				grid.coord2idx(current_coord,idx);
			}
			
			//Adding exactly the last point at the end.
			grid.idx2coord(idx, current_coord);
			std::copy_n( current_coord.begin(), ndims, current_point.begin() );
			//for (int i = 0; i < ndims; ++i) 
			//	current_point[i] = static_cast<double>(current_coord[i]);

			path.push_back(current_point); 
		}
		
        
    protected:
    
    private:
       
};


#endif /* GRADIENTDESCENT_H_*/
