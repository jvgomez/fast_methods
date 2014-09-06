/*! \file maploadertext.hpp
    \brief Auxiliar class which helps to load maps into nDGridMap
    
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

#ifndef MAPLOADERTEXT_H_
#define MAPLOADERTEXT_H_

#include "../ndgridmap/ndgridmap.hpp"
#include <fstream>

class MapLoaderText {
    public:
        MapLoaderText() {};
        virtual ~MapLoaderText() {};

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
         * @param filename text file to be open
         * @param grid 2D nDGridmap
         *
         */
        template<class T, size_t ndims>
        static void loadMapFromText
        (const char * filename, nDGridMap<T, ndims> & grid) {
            std::ifstream file;
            file.open(filename);

            if (file.is_open())
            {
                std::string val;
                std::getline(file, val);

                float leafsize;
                int width, height;
                size_t ndims_aux;

                file >> leafsize;
                file >> ndims_aux;
                file >> width;
                file >> height;

                std::array<int, ndims> dimsize = {width, height};
                grid.resize(dimsize);
                grid.setLeafSize(leafsize);

                for (int i = 0; i < width*height; ++i)
                {
                    bool occupancy;
                    file >> occupancy;

                    grid[i].setOccupancy(occupancy);
                }
            }
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
         * @param filename text file to be open
         * @param grid 2D nDGridmap
         * @param init_points stores the indices of all the values which are false.
         *
         */
        template<class T, size_t ndims>
        static void loadMapFromText
        (const char * filename, nDGridMap<T, ndims> & grid, std::vector<int> & init_points) {
            std::ifstream file;
            file.open(filename);

            if (file.is_open())
            {
                std::string val;
                std::getline(file, val);

                float leafsize;
                int width, height;
                size_t ndims_aux;

                file >> leafsize;
                file >> ndims_aux;
                file >> width;
                file >> height;

                std::array<int, ndims> dimsize = {width, height};
                grid.resize(dimsize);
                grid.setLeafSize(leafsize);

                for (int i = 0; i < width*height; ++i)
                {
                    bool occupancy;
                    file >> occupancy;

                    grid[i].setOccupancy(occupancy);

                    if (occupancy == 0)
                        init_points.push_back(i);
                }
            }
        }

    protected:

};

#endif /* MAPLOADER_H_ */
