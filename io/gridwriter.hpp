/*! \file gridwriter.hpp
    \brief Auxiliar class which helps to save nDGridMaps into text files.
    
    Additional Matlab scripts are provided to parse these grids.
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



#ifndef GRIDWRITER_H_
#define GRIDWRITER_H_

#include <fstream>

#include "../ndgridmap/ndgridmap.hpp"

// TODO: include checks which ensure that the grids are adecuate for the functions used.
// TODO: there should be a check when writing grid: it is written alread? erase and write. Something like that.
class GridWriter {
		
	public:
		GridWriter() {};
		virtual ~GridWriter() {};
		
		/**
		 * Saves grid values in ASCII format into the specified file.
		 * 
		 * Saved grid format:
		 * CellClass - info of the cell type\n  (string)
		 * leafsize_\n 							(float)
		 * ndims\n								(size_t)
		 * dimsize_[0]\n						(int)
		 * dimsize_[1]\n						(int)
		 * ...
		 * dimsize_[ndims_-1]\n					(int)
		 * getCell(0).getValue()\n 				(double)
		 * ...
		 * getCell(ncells_-1).getValue()\n 
		 * 
		 * Use the parsegrid.m Matlab script to parse the data.
		 * 
		 * @param filename name of the file to save.
		 * @param grid nDGridMap to be stored.
		 * */
		template <class T, size_t ndims> 
		static void saveGridValues
		(const char * filename, nDGridMap<T, ndims> & grid) {
			std::ofstream ofs;
			ofs.open (filename,  std::ofstream::out | std::ofstream::trunc);
			
			ofs << grid.getCell(0).type() << std::endl;
			ofs << grid.getLeafSize() << std::endl << ndims;
			
			std::array<int, ndims> dimsize = grid.getDimSizes();
			for (int i = 0; i < ndims; ++i)
				ofs << std::endl << dimsize[i] << "\t";
				   
			for (int i = 0; i < grid.size(); ++i)
				ofs << std::endl << grid.getCell(i).getValue();  
				
			ofs.close(); 
		}
		
		
		
		/**
		 * Saves grid velocities in ASCII format into the specified file.
		 * 
		 * Saved grid format:
		 * CellClass - info of the cell type\n  (string)
		 * leafsize_\n 							(float)
		 * ndims\n								(size_t)
		 * dimsize_[0]\n						(int)
		 * dimsize_[1]\n						(int)
		 * ...
		 * dimsize_[ndims_-1]\n					(int)
		 * getCell(0).getVelocity()\n 	
		 * ...
		 * getCell(ncells_-1).getVelocity()\n 	
		 * 		 * 
		 * Use the parsegrid.m Matlab script to parse the data.
		 * 
		 * @param filename name of the file to save.
		 * @param grid nDGridMap to be stored.
		 * */
		template <class T, size_t ndims> 
		static void saveVelocities
		(const char * filename, nDGridMap<T, ndims> & grid) {
			std::ofstream ofs;
			ofs.open (filename,  std::ofstream::out | std::ofstream::trunc);
			
			ofs << grid.getCell(0).type() << std::endl;
			ofs << grid.getLeafSize() << std::endl << ndims;
			
			std::array<int, ndims> dimsize = grid.getDimSizes();
			for (int i = 0; i < ndims; ++i)
				ofs << std::endl << dimsize[i] << "\t";
				   
			for (int i = 0; i < grid.size(); ++i)
				ofs << std::endl << grid.getCell(i).getVelocity();  
			
			ofs.close(); 
		}
		
	
		/** 
		 * Saves the 2D path in an ASCII file with the following format:
		 *
		 * leafsize_\n 							(float)
		 * ndims\n								(size_t)
		 * dimsize_[0]\n						(int)
		 * dimsize_[1]\n						(int)
		 * x1\ty1\tz1...\n 						(double)
		 * x2\ty2\tz2...\n 						(double)
		 * ...
		 * 		 * 
		 * Use the parsegrid.m and parsepath.m Matlab scripts to parse the data.
		 */
		template <class T, size_t ndims> 
		static void savePath
		(const char * filename, nDGridMap<T, ndims> & grid, std::vector< std::array<double,ndims> > & path) {
			std::ofstream ofs;
			ofs.open (filename,  std::ofstream::out | std::ofstream::trunc);
			
			ofs << grid.getLeafSize() << std::endl << ndims;
			
			std::array<int, ndims> dimsize = grid.getDimSizes();
			for (int i = 0; i < ndims; ++i)
				ofs << std::endl << dimsize[i] << "\t";
				   
			for (int i = 0; i < path.size(); ++i) {
				ofs << std::endl;
				for (int j = 0; j < ndims; ++j)	
					ofs << path[i][j] << "\t" ; 
			}
				
			ofs.close(); 
		}
		
		
		protected:
	
	   
};

#endif /* GRIDWRITER_H_ */
	   
