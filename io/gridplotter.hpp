#ifndef GRIDPLOTTER_H_
#define GRIDPLOTTER_H_

#include <string>
#include <array>

#include "../ndgridmap/ndgridmap.hpp"

#include <CImg.h>

using namespace cimg_library;

class GridPlotter {
	public:
		GridPlotter() {};
		virtual ~GridPlotter() {};
	
	
		// Restricted to 2D images. class T has to be FMCell or any class with getVelocity(float) function.
		// TODO: image checking: B/W, correct reading, etc.
		template<class T, size_t ndims = 2> 
		void plotMap
		(nDGridMap<T, ndims> & grid, const bool flipY = 1) {
			std::array<int,2> d = grid.getDimSizes();
			CImg<bool> img(d[0],d[1],1,1,0);
			if (flipY)
				// Filling the image flipping Y dim. We want now top left to be the (0,0).
				cimg_forXY(img,x,y) { img(x,y) = grid[img.width()*(img.height()-y-1)+x].getOccupancy(); }	
			else 
				cimg_forXY(img,x,y) { img(x,y) = grid[img.width()*y+x].getOccupancy(); }	
				
			img.display("Grid map");
		}
	
	
		template<class T, size_t ndims = 2> 
		void plotArrivalTimes
		(nDGridMap<T, ndims> & grid, const bool flipY = 1) {
			std::array<int,2> d = grid.getDimSizes();
			float max_val = grid.getMaxValue();
			CImg<float> img(d[0],d[1],1,1,0);

			if (flipY) 
				// Filling the image flipping Y dim. We want now top left to be the (0,0).
				cimg_forXY(img,x,y) { img(x,y) = grid[img.width()*(img.height()-y-1)+x].getArrivalTime()/max_val*255; }
			else 
				cimg_forXY(img,x,y) { img(x,y) = grid[img.width()*y+x].getArrivalTime()/max_val*255; }	
				
			img.map( CImg<float>::jet_LUT256() );
			img.display("Arrival Times");

			
		}
	
	
	protected:
	
};



#endif /* GRIDPLOTTER_H_ */
