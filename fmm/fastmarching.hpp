/*! \file fastmarching.hpp
    \brief Templated class which computes the basic Fast Marching Method.
    
    It uses as a main container the nDGridMap class. The nDGridMap type T
    has to be an FMCell or something inherited from it.
    
    The leafsize of the grid map is ignored since it has to be >=1 and that 
    depends on the units employed.
    
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


#ifndef FASTMARCHING_H_
#define FASTMARCHING_H_

#include <iostream>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <fstream>
#include <array>

#include "../fmdata/fmcell.h"
#include "../ndgridmap/ndgridmap.hpp"
#include "../console/console.h"
#include "../fmdata/fmfibheap.hpp"

// TODO: when computing FM on the same grid twice it could fail. It should reset the grid in that case.
// TODO: check initial and goal points are not the same, not on obstacles, etc.

template <class T, size_t ndims> class FastMarching {
	
    public: 
        FastMarching <T,ndims> () {
			
		};
        virtual ~FastMarching <T,ndims>() {};
            
         /**
          * Sets the input grid in which operations will be performed.
          * 
          * @param g input grid map.
          */   
        virtual void setEnvironment 
        (nDGridMap<T,ndims> * g) {
			grid_ = g;
			leafsize_ = grid_->getLeafSize();
			narrow_band_.setMaxSize(grid_->size());
		}
        
		
		/**
		 * Set the initial points by the indices in the nDGridMap and 
		 * computes the initialization of the Fast Marching Method calling
		 * the init() function.
		 * 
		 * @param contains the indices of the init points. 
		 * 
		 * @see init()
		 */	
		virtual void setInitialPoints
		(const std::vector<int> & init_points) {
			init_points_ = init_points;
			for (const int &i: init_points) {
				grid_->getCell(i).setArrivalTime(0);
				grid_->getCell(i).setState(FMState::FROZEN);
			}
			
			init();
		}	
		
		 /**
		 * Internal function although it is set to public so it can be accessed if desired.
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
			
		
		//IMPORTANT NOTE: Assuming inc(1) = inc(y) =...= leafsize_
		// Possible improvement: If we include the neighbors in the cells information
		// this could be (most probably) speeded up.
		// This implementation is focused to be used with any number of dimensions.
		
		 /**
		 * Solves the Eikonal equation for a given cell. This function is generalized
		 * to any number of dimensions.
		 * 
		 * @param idx index of the cell to be evaluated.
		 * 
		 * @return the distance (or time of arrival) value.
		 */ 
		virtual double solveEikonal
		(const int & idx) {
			// TODO: Here neighbors are computed and then in the computeFM. There should be a way to avoid computing
			// neighbors twice.
			 
			
			int a = ndims; // a parameter of the Eikonal equation.
			
			double updatedT;
			sumT = 0;
			sumTT = 0;

			for (int dim = 0; dim < ndims; ++dim) {
				double minTInDim = grid_->getMinValueInDim(idx, dim);
				if (!isinf(minTInDim)) {
					Tvalues[dim] = minTInDim;
					sumT += Tvalues[dim];
					TTvalues[dim] = Tvalues[dim]*Tvalues[dim];
					sumTT += TTvalues[dim];
				}
				else {
					Tvalues[dim] = 0;
					TTvalues[dim] = 0;
					a -=1 ;
				}
			}
			
			double b = -2*sumT;
			double c = sumTT - 1/(grid_->getCell(idx).getVelocity()*grid_->getCell(idx).getVelocity()); // leafsize not taken into account here.
			double quad_term = b*b - 4*a*c;
			if (quad_term < 0) {
				double minT = *(std::min_element(Tvalues.begin(), Tvalues.end()));
				updatedT = 1/(grid_->getCell(idx).getVelocity()*grid_->getCell(idx).getVelocity()) + minT; // leafsize not taken into account here.
			}
			else 
				updatedT = (-b + sqrt(quad_term))/(2*a);
			
			return updatedT;
		}	
		
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
		nDGridMap<T, ndims> *  grid_; /*!< Main container.. */
		
		std::vector<int> init_points_;	/*!< Initial points for the Fast Marching Method. */
		FMFibHeap narrow_band_; /*!< Instance of the Fibonacci Heap used. */
		
		double leafsize_; /*!< Although it is on grid, it is stored here so that it has not to be accessed. */
		double sumT; /*!< Auxiliar value wich computes T1+T2+T3... Useful for generalizing the Eikonal solver. */
		double sumTT; /*!< Auxiliar value wich computes T1^2+T2^2+T3^2... Useful for generalizing the Eikonal solver. */
		
		std::array<double,ndims> Tvalues;  /*!< Auxiliar array with values T0,T1...Tn-1 variables in the Discretized Eikonal Equation. */
		std::array<double,ndims> TTvalues;  /*!< Auxiliar array with values T0^2,T1^2...Tn-1^2 variables in the Discretized Eikonal Equation. */
		std::array <int,2*ndims> neighbors;  /*!< Auxiliar array which stores the neighbor of each iteration of the computeFM() function. */
};


#endif /* FASTMARCHING_H_*/
