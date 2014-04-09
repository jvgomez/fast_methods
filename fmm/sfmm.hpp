/*! \file sfmm.hpp
    \brief Simplified Fast Marching Method
    
    This class keeps the narrow band in a standard priority queue and it is possible
    to have many instances of the same cell in the narrow band. Just the first cell popped is taken
    into account and the rest instances of the same cell are immediately ignored when popped.
    *
    It uses as a main container the nDGridMap class. The nDGridMap type T
    has to be an FMCell or something inherited from it.
    
    The leafsize of the grid map is ignored since it has to be >=1 and that 
    depends on the units employed.
    
    This method is introduced in the paper "3D Distance Fields: A Survey of Techniques and Applications"
    M.W. Jones, J.A. Baerentzen and M. Sramek, 2006.
    
    This method is faster than standard FMM for most common situations: small grids, 
    the narrow band is not big. However, to define "small grid or small narrow band" is highly dependent 
    on the problem.
    
    A binary-heap based version has been tested and it is much slower than this version.
    
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


#ifndef SFMM_H_
#define SFMM_H_

#include "fastmarching.hpp"
#include "../fmdata/fmpriorityqueue.hpp"

template <class T, size_t ndims> class SFMM : public FastMarching <T, ndims> {
	
	// Solving the two-phase name lookup issue.
	using FastMarching<T,ndims>::grid_;
	using FastMarching<T,ndims>::neighbors;
	using FastMarching<T,ndims>::solveEikonal;
	using FastMarching<T,ndims>::init_points_;
	using FastMarching<T,ndims>::leafsize_;

    public: 
        SFMM <T,ndims> () {};
        virtual ~SFMM <T,ndims>() {};
                          	
       	void init
		() {
			// TODO: neighbors computed twice for every cell. We can save time here.
			// TODO: check if the previous steps have been done (loading grid map and setting initial points.)
			int j = 0;
			int n_neighs = 0;
			for (int &i: init_points_) { // For each initial point
				n_neighs = grid_->getNeighbors(i, neighbors);
				for (int s = 0; s < n_neighs; ++s){  // For each neighbor
					j = neighbors[s];
					if (grid_->getCell(j).getState() != FMState::FROZEN) {
						double new_arrival_time = solveEikonal(j);
						grid_->getCell(j).setArrivalTime(new_arrival_time);
						narrow_band_.push( &(grid_->getCell(j))  ) ;
					}	
				} // For each neighbor.
			} // For each initial point.
		} // init()
       	
		/**
		 * Main Fast Marching Function. It requires to call first the setInitialPoints() function.
		 * 
		 * @see setInitialPoints()
		 */
		void computeFM
		() {
			int j = 0;
			int n_neighs = 0;
			// TODO: check if the previous steps have been done (initialization).
			while (narrow_band_.size() > 0) {
				int idxMin = narrow_band_.popMinIdx();
				
				if (grid_->getCell(idxMin).getState() != FMState::FROZEN) {
					grid_->getCell(idxMin).setState(FMState::FROZEN);
					n_neighs = grid_->getNeighbors(idxMin, neighbors);
					
					for (int s = 0; s < n_neighs; ++s) {
						j = neighbors[s];
						if (grid_->getCell(j).getState() != FMState::FROZEN) {
							double new_arrival_time = solveEikonal(j);
							// Specific in this implementation. This if is not in the paper but there is no
							// any advantages in this method if this condition is not included.
							if (new_arrival_time < grid_->getCell(j).getArrivalTime()) {
								grid_->getCell(j).setArrivalTime(new_arrival_time);
								narrow_band_.push( &(grid_->getCell(j)) );
							}
						}	
					}
				}
			}
		}
 
    protected:
    
		FMPriorityQueue narrow_band_;
};


#endif /* SFMM_H_*/
