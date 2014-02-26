/*! \file maploader.hpp
    \brief Auxiliar class which helps to load maps into nDGridMap
    
    It is based on the CImg library, therefore it has to be accessible.
    Copyright (C) 2014 Javier V. Gomez
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



#ifndef MAPLOADER_H_
#define MAPLOADER_H_


#include "../ndgridmap/ndgridmap.hpp"

#include <CImg.h>

using namespace cimg_library;

// TODO: include checks which ensure that the grids are adecuate for the functions used.
// TODO: include methods to read the txt saved by GridWriter.

class MapLoader {
	public:
		MapLoader() {};
		virtual ~MapLoader() {};
	
	   /**
		 *  Loads the initial binary map for a given grid. It is based on the
		 * nDGridMap::setOccupancy() which has to be bool valued. This function has to be
		 * overloaded in another occupancy type is being used.
		 * 
		 * The image should be monochromatic!
		 * 
		 * Should be used only in 2D grids.
		 * 
		 * The Y dimension flipping is because nDGridMap works in X-Y coordinates, not in image indices as CImg.
		 * 
		 * IMPORTANT NOTE: no type-checkings are done. T type has to be Cell or any class with bool setOccupancy() method.
		 * 
		 * @param filename file to be open
		 * @param grid 2D nDGridmap
		 * 
		 */
		template<class T, size_t ndims> 
		static void loadMapFromImg
		(const char * filename, nDGridMap<T, ndims> & grid) {
			CImg<bool> img(filename);
			std::array<int, ndims> dimsize = {img.width(), img.height()};
			grid.resize(dimsize);

			// Filling the grid flipping Y dim. We want bottom left to be the (0,0).
			cimg_forXY(img,x,y) {grid[img.width()*(img.height()-y-1)+x].setOccupancy(img(x,y)); }
		}
		
		
	    /**
		 * Loads the initial binary map for a given grid. It is based on the
		 * nDGridMap::setOccupancy() which has to be bool valued. This function has to be
		 * overloaded in another occupancy type is being used.
		 * 
		 * The image should be monochromatic!
		 * 
		 * In also stores all the false values to as initial points for a later computeFM function.
		 * 
		 * Should be used only in 2D grids.
		 * 
		 * The Y dimension flipping is because nDGridMap works in X-Y coordinates, not in image indices as CImg.
		 * 
		 * IMPORTANT NOTE: no type-checkings are done. T type has to be Cell or any class with bool setOccupancy() method.
		 * 
		 * @param filename file to be open
		 * @param grid 2D nDGridmap
		 * @param init_points stores the indices of all the values which are false.
		 * 
		 */
		template<class T, size_t ndims> 
		static void loadMapFromImg
		(const char * filename, nDGridMap<T, ndims> & grid, std::vector<int> & init_points) {
			CImg<bool> img(filename);
			std::array<int, ndims> dimsize = {img.width(), img.height()};
			grid.resize(dimsize);

			// Filling the grid flipping Y dim. We want bottom left to be the (0,0).
			cimg_forXY(img,x,y) {
				bool occupancy = img(x,y);
				int idx = img.width()*(img.height()-y-1)+x;
				grid[idx].setOccupancy(occupancy); 
				if (occupancy == 0)
					init_points.push_back(idx);				
				}
		}
		
	    /**
		 * Loads the velocities map for a given grid. It is based on the
		 * nDGridMap::seVelocity() which has to be float valued. This function has to be
		 * overloaded in another occupancy type is being used.
		 * 
		 * The image should be gray scale! It will be load as a single-channel, float values for pixels.
		 * 
		 * In also stores all the false values to as initial points for a later computeFM function.
		 * 
		 * Should be used only in 2D grids.
		 * 
		 * The Y dimension flipping is because nDGridMap works in X-Y coordinates, not in image indices as CImg.
		 * 
		 * IMPORTANT NOTE: no type-checkings are done. T type has to be Cell or any class with float setVelocity() method.
		 * 
		 * @param filename file to be open
		 * @param grid 2D nDGridmap
		 * 
		 */
		template<class T, size_t ndims> 
		static void loadVelocitiesFromImg
		(const char * filename, nDGridMap<T, ndims> & grid) {
			CImg<float> img(filename);
			std::array<int, ndims> dimsize = {img.width(), img.height()};
			grid.resize(dimsize);
			// Filling the grid flipping Y dim. We want bottom left to be the (0,0).
			cimg_forXY(img,x,y) {
				float vel = img(x,y); // First of the three channels of the image is taken.
				int idx = img.width()*(img.height()-y-1)+x;
				grid[idx].setVelocity(vel/255); 
			}
		}
		
		
	
	
	
	protected:
	
	
};



#endif /* MAPLOADER_H_ */
