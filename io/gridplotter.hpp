/*! \file gridplotter.hpp
    \brief Auxiliar class which helps to visualise Fast Marching steps and results.
    
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

#ifndef GRIDPLOTTER_H_
#define GRIDPLOTTER_H_

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
class GridPlotter {
	public:
		GridPlotter() {};
		virtual ~GridPlotter() {};
	
	
		/**
		 * Plots the initial binary map included in a given grid. It is based on the
		 * nDGridMap::getOccupancy() which has to be bool valued. This function has to be
		 * overloaded in another occupancy type is being used.
		 * 
		 * Should be used only in 2D grids.
		 * 
		 *  The Y dimension flipping is because nDGridMap works in X-Y coordinates, not in image indices as CImg.
		 * 
		 * IMPORTANT NOTE: no type-checkings are done. T type has to be Cell or any class with bool getOccupancy() method.
		 * 
		 * @param grid 2D nDGridmap
		 * @param flipY true: flips the Y dimension. 0 does not flip.
		 */
		template<class T, size_t ndims> 
		static void plotMap
		(nDGridMap<T, ndims> & grid, const bool flipY = 1) {
			// TODO: image checking: B/W, correct reading, etc.
			std::array<int,2> d = grid.getDimSizes();
			CImg<bool> img(d[0],d[1],1,1,0);
			if (flipY)
				// Filling the image flipping Y dim. We want now top left to be the (0,0).
				cimg_forXY(img,x,y) { img(x,y) = grid[img.width()*(img.height()-y-1)+x].getOccupancy(); }	
			else 
				cimg_forXY(img,x,y) { img(x,y) = grid[img.width()*y+x].getOccupancy(); }	
				
			img.display("Grid map", false);
		}
	
	
	   /**
		 * Plots the value map included in a given grid. It is based on the
		 * nDGridMap::getValue() which has to be float valued. This function has to be
		 * overloaded in another value type is being used.
		 * 
		 * Should be used only in 2D grids.
		 * 
		 * The Y dimension flipping is because nDGridMap works in X-Y coordinates, not in image indices as CImg.
		 * 
		 * IMPORTANT NOTE: no type-checkings are done. T type has to be Cell or any class with bool getValue() method.
		 * 
		 * @param grid 2D nDGridmap
		 * @param flipY true: flips the Y dimension. 0 does not flip.
		 */
		template<class T, size_t ndims = 2> 
		static void plotArrivalTimes
		(nDGridMap<T, ndims> & grid, const bool flipY = true) {
			std::array<int,2> d = grid.getDimSizes();
			double max_val = grid.getMaxValue();
			CImg<double> img(d[0],d[1],1,1,0);

			if (flipY) 
				// Filling the image flipping Y dim. We want now top left to be the (0,0).
				cimg_forXY(img,x,y) { img(x,y) = grid[img.width()*(img.height()-y-1)+x].getValue()/max_val*255; }
			else 
				cimg_forXY(img,x,y) { img(x,y) = grid[img.width()*y+x].getValue()/max_val*255; }	
				
			img.map( CImg<float>::jet_LUT256() );
			img.display("Grid values", false);	
		}

        /**
         * Plots the initial binary map included in a given grid and the given path. It is based on the
         * nDGridMap::getOccupancy() which has to be bool valued. This function has to be
         * overloaded in another occupancy type is being used.
         *
         * Should be used only in 2D grids.
         *
         *  The Y dimension flipping is because nDGridMap works in X-Y coordinates, not in image indices as CImg.
         *
         * IMPORTANT NOTE: no type-checkings are done. T type has to be Cell or any class with bool getOccupancy() method.
         *
         * @param grid 2D nDGridmap
         * @param path 2D path to plot.
         * @param flipY true: flips the Y dimension. 0 does not flip.
         */
        template<class T, size_t ndims = 2>
        static void plotMapPath
        (nDGridMap<T, ndims> & grid, const Path2D & path, const bool flipY = true) {

            //std::vector <const Path2D *> & path    - (*path[i])[j]
            std::array<int,2> d = grid.getDimSizes();
            //double max_val = grid.getMaxValue();
            CImg<double> img(d[0],d[1],1,3,0);

            if (flipY)  {
                // Filling the image flipping Y dim. We want now top left to be the (0,0).
                cimg_forXYZC(img,x,y,z,c) { img(x,y,z,c) = grid[img.width()*(img.height()-y-1)+x].getOccupancy()*255; }

                for (int i = 0; i< path.size(); ++i)
                {
                    img(static_cast<int>(path[i][0]), (img.height()-static_cast<int>(path[i][1])-1), 0, 1) = 0;
                    img(static_cast<int>(path[i][0]), (img.height()-static_cast<int>(path[i][1])-1), 0, 2) = 0;
                }
            }
            else {
                cimg_forXY(img,x,y) { img(x,y) = grid[img.width()*y+x].getOccupancy()*255; }
                for (int i = 0; i< path.size(); ++i)
                    img(static_cast<int>(path[i][0]), static_cast<int>(path[i][1])) = 255;
                }

            img.display("Grid values", false);

        }

        /**
         * Plots the initial binary map included in a given grid and the given paths vector.
         * It is based on the nDGridMap::getOccupancy() which has to be bool valued. This function
         * has to be overloaded in another occupancy type is being used.
         *
         * Should be used only in 2D grids.
         *
         *  The Y dimension flipping is because nDGridMap works in X-Y coordinates, not in image indices as CImg.
         *
         * IMPORTANT NOTE: no type-checkings are done. T type has to be Cell or any class with bool getOccupancy() method.
         *
         * @param grid 2D nDGridmap
         * @param vector of path 2D path to plot.
         * @param flipY true: flips the Y dimension. 0 does not flip.
         */
        template<class T, size_t ndims = 2>
        static void plotMapPath
        (nDGridMap<T, ndims> & grid, const Paths2D & paths, const bool flipY = true) {

            //std::vector <const Path2D *> & path    - (*path[i])[j]
            std::array<int,2> d = grid.getDimSizes();
            //double max_val = grid.getMaxValue();
            CImg<double> img(d[0],d[1],1,3,0);

            if (flipY)  {
                // Filling the image flipping Y dim. We want now top left to be the (0,0).
                cimg_forXYZC(img,x,y,z,c) { img(x,y,z,c) = grid[img.width()*(img.height()-y-1)+x].getOccupancy()*255; }

                // Draw the path using different colours
                for (int j = 0; j < paths.size(); ++j)
                {
                    Path2D path = paths[j];
                    for (int i = 0; i< path.size(); ++i)
                    {
                        img(static_cast<int>(path[i][0]), (img.height()-static_cast<int>(path[i][1])-1), 0, j) = 0;
                        img(static_cast<int>(path[i][0]), (img.height()-static_cast<int>(path[i][1])-1), 0, j+1) = 0;
                    }
                }
            }
            else {
                cimg_forXY(img,x,y) { img(x,y) = grid[img.width()*y+x].getOccupancy()*255; }

                // Draw the path using different colours
                for (int j = 0; j < paths.size(); ++j)
                {
                    Path2D path = paths[j];

                    for (int i = 0; i< path.size(); ++i)
                        img(static_cast<int>(path[i][0]), static_cast<int>(path[i][1])) = 255;
                }
            }

            img.display("Grid values", false);

        }

       /**
       * Plots the value map included in a given grid and the given path. It is based on the
       * nDGridMap::getValue() which has to be float valued. This function has to be
       * overloaded in another value type is being used.
       *
       * Should be used only in 2D grids.
       *
       * The Y dimension flipping is because nDGridMap works in X-Y coordinates, not in image indices as CImg.
       *
       * IMPORTANT NOTE: no type-checkings are done. T type has to be Cell or any class with bool getValue() method.
       *
       * @param grid 2D nDGridmap
       * @param path 2D path to plot.
       * @param flipY true: flips the Y dimension. 0 does not flip.
       */
      template<class T, size_t ndims = 2>
      static void plotArrivalTimesPath
      (nDGridMap<T, ndims> & grid, const Path2D & path, const bool flipY = true) {
          std::array<int,2> d = grid.getDimSizes();
          double max_val = grid.getMaxValue();
          CImg<double> img(d[0],d[1],1,1,0);

          if (flipY)  {
              // Filling the image flipping Y dim. We want now top left to be the (0,0).
              cimg_forXY(img,x,y) { img(x,y) = grid[img.width()*(img.height()-y-1)+x].getValue()/max_val*255; }

              for (int i = 0; i< path.size(); ++i)
                  img(static_cast<int>(path[i][0]), (img.height()-static_cast<int>(path[i][1])-1)) = 255;
          }
          else {
              cimg_forXY(img,x,y) { img(x,y) = grid[img.width()*y+x].getValue()/max_val*255; }
              for (int i = 0; i< path.size(); ++i)
                  img(static_cast<int>(path[i][0]), static_cast<int>(path[i][1])) = 255;
              }


          img.map( CImg<double>::jet_LUT256() );


          img.display("Grid values", false);


      }
	
	
	protected:
	
};



#endif /* GRIDPLOTTER_H_ */
