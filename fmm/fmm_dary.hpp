/*! \file sfmm.hpp
    \brief Fast Marching Method using a Binary Heap
    
    It uses as a main container the nDGridMap class. The nDGridMap type T
    has to be an FMCell or something inherited from it.
    
    The leafsize of the grid map is ignored since it has to be >=1 and that 
    depends on the units employed.
    
    This method is introduced in the paper "3D Distance Fields: A Survey of Techniques and Applications"
    M.W. Jones, J.A. Baerentzen and M. Sramek, 2006.
    * 
    However, in this implementation, the results are not that satistactory as in their paper. In fact, 
    this method is much slower than the standard FMM.
    * 
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


#ifndef FMM_DARY_H_
#define FMM_DARY_H_

#include "fastmarching.hpp"
#include "../fmdata/fmdaryheap.hpp"

template <class T, size_t ndims> class FMM_Dary : public FastMarching <T, ndims> {
	
	// Solving the two-phase name lookup issue.
	using FastMarching<T,ndims>::grid_;
	using FastMarching<T,ndims>::neighbors;
	using FastMarching<T,ndims>::solveEikonal;
	using FastMarching<T,ndims>::init_points_;
	using FastMarching<T,ndims>::leafsize_;
	
    public: 
        FMM_Dary <T,ndims> () {};
        virtual ~FMM_Dary <T,ndims>() {};
        
          /**
          * Sets the input grid in which operations will be performed.
          * 
          * @param g input grid map.
          */   
        virtual void setEnvironment 
        (nDGridMap<T,ndims> * g) {
			// If this code is not taken from FastMarching, the narrow band is not properly initialized.
			grid_ = g;
			leafsize_ = grid_->getLeafSize();
			narrow_band_.setMaxSize(grid_->size());
		}
      
		/* * Internal function although it is set to public so it can be accessed if desired.
		 * 
		 * Computes the Fast Marching Method initialization from the initial points given. Programmed following the paper:
			A. Valero, J.V. Gómez, S. Garrido and L. Moreno, The Path to Efficiency: Fast Marching Method for Safer,
			More Efficient Mobile Robot Trajectories, IEEE Robotics and Automation Magazine, Vol. 20, No. 4, 2013.
		 * 
		 * @see setInitialPoints()
		 */	
		virtual void init
		() {
			// TODO: neighbors computed twice for every cell. We can save time here.
			// TODO: check if the previous steps have been done (loading grid map and setting initial points.)
			int j = 0;
			int n_neighs = 0;
			for (int &i: init_points_) { // For each initial point
				n_neighs = grid_->getNeighbors(i, neighbors);
				for (int s = 0; s < n_neighs; ++s){  // For each neighbor
					j = neighbors[s];
					if ((grid_->getCell(j).getState() == FMState::FROZEN) || grid_->getCell(j).isOccupied()) // If Frozen or obstacle
						continue;
					else {
						double new_arrival_time = solveEikonal(j);
						if (grid_->getCell(j).getState() == FMState::NARROW) { // Updating narrow band if necessary.
							if (new_arrival_time < grid_->getCell(j).getArrivalTime()) {
								grid_->getCell(j).setArrivalTime(new_arrival_time);
								narrow_band_.increase( &(grid_->getCell(j))  ) ;
							}
						}
						else {
							grid_->getCell(j).setState(FMState::NARROW);
							grid_->getCell(j).setArrivalTime(new_arrival_time);
							narrow_band_.push( &(grid_->getCell(j)) );
						} // neighbors open.
					} // neighbors not frozen.
				} // For each neighbor.
			} // For each initial point.
		} // init()
			
       	/**
		 * Main Fast Marching Function. It requires to call first the setInitialPoints() function.
		 * 
		 * @see setInitialPoints()
		 */
		virtual void computeFM
		() {
			// TODO: check if the previous steps have been done (initialization).
			int j= 0;
			int n_neighs = 0;
			while (narrow_band_.size() > 0) {
				int idxMin = narrow_band_.popMinIdx();
				n_neighs = grid_->getNeighbors(idxMin, neighbors);
				grid_->getCell(idxMin).setState(FMState::FROZEN);

				for (int s = 0; s < n_neighs; ++s) {
					j = neighbors[s];
					if ((grid_->getCell(j).getState() == FMState::FROZEN) || grid_->getCell(j).isOccupied()) // If Frozen or obstacle
						continue;
					else {
						double new_arrival_time = solveEikonal(j);
						if (grid_->getCell(j).getState() == FMState::NARROW) { // Updating narrow band if necessary.
							if (new_arrival_time < grid_->getCell(j).getArrivalTime()) {
								grid_->getCell(j).setArrivalTime(new_arrival_time);
								narrow_band_.increase( &(grid_->getCell(j)) );
							}
						}
						else {
							grid_->getCell(j).setState(FMState::NARROW);
							grid_->getCell(j).setArrivalTime(new_arrival_time);
							narrow_band_.push( &(grid_->getCell(j)) );
						} // neighbors open.
					} // neighbors not frozen.
				} // For each neighbor.
			} // while narrow band not empty
		}
 
    protected:
    
		FMDaryHeap narrow_band_;
    
};


#endif /* FMM_DARY_H_*/
