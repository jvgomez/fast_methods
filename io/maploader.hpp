#ifndef MAPLOADER_H_
#define MAPLOADER_H_


#include "../ndgridmap/ndgridmap.hpp"

#include <CImg.h>

using namespace cimg_library;

class MapLoader {
	public:
		MapLoader() {};
		virtual ~MapLoader() {};
	
	
		// Restricted to 2D B/W images. class T has to be FMCell or any class with setVelocity(float) function.
		// TODO: image checking: B/W, correct reading, etc.
		template<class T, size_t ndims = 2> 
		void loadMapFromImg
		(const char * filename, nDGridMap<T, ndims> & grid) {
			CImg<bool> img(filename);
			std::array<int, ndims> dimsize = {img.width(), img.height()};
			grid.resize(dimsize);

			// Filling the grid flipping Y dim. We want bottom left to be the (0,0).
			cimg_forXY(img,x,y) {grid[img.width()*(img.height()-y-1)+x].setOccupancy(img(x,y)); }
		}
		
		// Restricted to 2D B/W images. class T has to be FMCell or any class with setVelocity(float) function.
		// TODO: image checking: B/W, correct reading, etc.
		template<class T, size_t ndims = 2> 
		void loadMapFromImg
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
		
		// Restricted to 2D grayscale images. class T has to be FMCell or any class with setVelocity(float) function.
		// TODO: image checking: B/W, correct reading, etc.
		template<class T, size_t ndims = 2> 
		void loadVelocitiesFromImg
		(const char * filename, nDGridMap<T, ndims> & grid, std::vector<int> & init_points) {
			CImg<float> img(filename);
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
		
		
	
	
	
	protected:
	
	
};



#endif /* MAPLOADER_H_ */
