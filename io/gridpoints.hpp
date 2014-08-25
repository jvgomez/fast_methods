/*! \file gridpoints.hpp
    \brief Auxiliar class which helps to select initial and goal points in a 2d grid.
    
    It is based on the CImg library, therefore it has to be accessible.
    Copyright (C) 2014 Javier V. Gomez and Jose Pardeiro
    www.javiervgomez.com

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.
	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
	GNU General Public License for more details.
	You should have received a copy of the GNU General Public License
	along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef GRIDPOINTS_H_
#define GRIDPOINTS_H_

#include <string>
#include <array>

#include "../ndgridmap/ndgridmap.hpp"

#include <CImg.h>

using namespace cimg_library;

typedef typename std::array<int, 2> Coord2D;
typedef typename std::array<double, 2> Point2D;
typedef typename std::vector <Point2D> Path2D;
typedef typename std::vector <Path2D> Paths2D;

// TODO: include checks which ensure that the grids are adecuate for the functions used.
class GridPoints {
	public:
        GridPoints() {};
        virtual ~GridPoints() {};
	
	
		/**
         * Plots the initial binary map included in a given grid and extract coords of the
         * initial and goal point. It is based on the nDGridMap::getOccupancy() which has to
         * be bool valued. This function has to be overloaded in another occupancy type is being used.
		 * 
		 * Should be used only in 2D grids.
		 * 
		 *  The Y dimension flipping is because nDGridMap works in X-Y coordinates, not in image indices as CImg.
		 * 
		 * IMPORTANT NOTE: no type-checkings are done. T type has to be Cell or any class with bool getOccupancy() method.
		 * 
		 * @param grid 2D nDGridmap
         * @param coords of the initial point.
         * @param coords of the goal point.
		 * @param flipY true: flips the Y dimension. 0 does not flip.
		 */
		template<class T, size_t ndims> 
        static void selectMapPoints
        (nDGridMap<T, ndims> & grid, std::array<int,ndims> & coords_init, std::array<int,ndims> & coords_goal, const bool flipY = 1) {
            int y=0, x=0;
			// TODO: image checking: B/W, correct reading, etc.
			std::array<int,2> d = grid.getDimSizes();
			CImg<bool> img(d[0],d[1],1,1,0);
			if (flipY)
				// Filling the image flipping Y dim. We want now top left to be the (0,0).
				cimg_forXY(img,x,y) { img(x,y) = grid[img.width()*(img.height()-y-1)+x].getOccupancy(); }	
			else 
				cimg_forXY(img,x,y) { img(x,y) = grid[img.width()*y+x].getOccupancy(); }	
				
            CImgDisplay main_disp(img,"Click a point");

            // Detect click of the mouse
            while (x==0) {
                 main_disp.wait();
                 if (main_disp.button() && main_disp.mouse_y()>=0) {
                     if (flipY)
                        y = img.height()- 1- main_disp.mouse_y();
                     else
                        y = main_disp.mouse_y();
                   x = main_disp.mouse_x();
                }
            }

            coords_init = {x,y};

            x=0;

            // Detect click of the mouse
            while (x==0) {
                 main_disp.wait();
                 if (main_disp.button() && main_disp.mouse_y()>=0) {
                     if (flipY)
                        y = img.height()- 1- main_disp.mouse_y();
                     else
                        y = main_disp.mouse_y();
                   x = main_disp.mouse_x();
                }
            }

            coords_goal = {x,y};
        }
	
	
	protected:
	
};



#endif /* GRIDPLOTTER_H_ */
