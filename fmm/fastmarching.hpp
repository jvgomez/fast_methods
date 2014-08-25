/*! \file fastmarching.hpp
    \brief Templated class which computes the basic Fast Marching Method (FMM).
    
    It uses as a main container the nDGridMap class. The nDGridMap type T
    has to be an FMCell or something inherited from it.
    
    The leafsize of the grid map is ignored since it has to be >=1 and that 
    depends on the units employed.
     
    The type of the heap introduced is very important for the behaviour of the 
    algorithm. The following heaps are provided:
   
	- FMDaryHeap wrap for the Boost D_ary heap (generalization of binary heaps). 
	* Set by default if no other heap is specified. The arity has been set to 2 
	* (binary heap) since it has been tested to be the more efficient in this algorithm.
	- FMFibHeap wrap for the Boost Fibonacci heap.
	- FMPriorityQueue wrap to the std::PriorityQueue class. This heap implies the implementation
	* of the Simplified FMM (SFMM) method, done automatically because of the FMPriorityQueue::increase implementation.
	* 
	@par External documentation:
		FMM: 
          A. Valero, J.V. Gómez, S. Garrido and L. Moreno, The Path to Efficiency: Fast Marching Method for Safer, More Efficient Mobile Robot Trajectories, IEEE Robotics and Automation Magazine, Vol. 20, No. 4, 2013. DOI: <a href="http://dx.doi.org/10.1109/MRA.2013.2248309">10.1109/MRA.2013.2248309></a><br>
           <a href="http://ieeexplore.ieee.org/xpl/articleDetails.jsp?arnumber=6582543">[PDF]</a>
        
        SFMM: 
          M.W. Jones, J.A. Baerentzen, M. Sramek, 3D Distance Fields: A Survey of Techniques and Applications, IEEE Transactions on Visualization and Computer Graphics, Vol. 12, No. 4, 2006. DOI <a href=http://dx.doi.org/10.1109/TVCG.2006.56">110.1109/TVCG.2006.56</a><br>
          <a href="http://ieeexplore.ieee.org/xpl/articleDetails.jsp?arnumber=1634323">[PDF]</a>
    
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
#include "../fmdata/fmstarcell.h"
#include "../fmdata/fmdirectionalcell.h"
#include "../ndgridmap/ndgridmap.hpp"
#include "../console/console.h"
#include "../fmdata/fmdaryheap.hpp"
#include "../fmdata/fmdaryheapstar.hpp"

// TODO: when computing FM on the same grid twice it could fail. It should reset the grid in that case.
// TODO: check initial and goal points are not the same, not on obstacles, etc.
// TODO: implement the heap type as a policy, since in derived classes setEnvironment 
//		 function has to be copied (FMM_Dary) in order to properly start the heap.

// IMPORTANT TODO: substitute grid_->getCell(j).isOccupied() by grid_->getCell(j).getVelocity() == 0 (conceptually is not the same).

template < class grid_t, class heap_t = FMDaryHeap<FMCell> >  class FastMarching {
	
    public: 
        FastMarching <grid_t, heap_t> () {
			
		};
        virtual ~FastMarching <grid_t, heap_t>() {};
            
         /**
          * Sets the input grid in which operations will be performed.
          * 
          * @param g input grid map.
          */   
        virtual void setEnvironment 
        (grid_t * g) {
			grid_ = g;
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
        (const std::vector<int> & init_points, int goal) {
			init_points_ = init_points;
            goal_idx_ = goal;
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
					if ((grid_->getCell(j).getState() == FMState::FROZEN) || grid_->getCell(j).isOccupied() || grid_->getCell(j).getVelocity() == 0) // If Frozen or obstacle
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
			 
			
			int a = grid_t::getNDims(); // a parameter of the Eikonal equation.
			
			double updatedT;
			sumT = 0;
			sumTT = 0;

			for (int dim = 0; dim < grid_t::getNDims(); ++dim) {
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
            double c = sumTT - grid_->getLeafSize()/(grid_->getCell(idx).getVelocity()*grid_->getCell(idx).getVelocity()); // leafsize not taken into account here.
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
         * @param select if the wave has to stop when it arrives to the goal point
         *
		 * @see setInitialPoints()
		 */
		virtual void computeFM
        (bool stop = true) {
			// TODO: check if the previous steps have been done (initialization).
			int j= 0;
			int n_neighs = 0;
            bool stopWavePropagation = 0;

            while (narrow_band_.size() > 0 && stopWavePropagation == 0) {
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
                    if (idxMin == goal_idx_ && stop)
                        stopWavePropagation = 1;
                } // For each neighbor.
			} // while narrow band not empty
		}
 
		
    protected:
		grid_t* grid_; /*!< Main container. */
		heap_t narrow_band_; /*!< Instance of the heap used. */
		
        std::vector<int> init_points_;	/*!< Initial points for the Fast Marching Method. */
        int goal_idx_;
		
		double sumT; /*!< Auxiliar value wich computes T1+T2+T3... Useful for generalizing the Eikonal solver. */
		double sumTT; /*!< Auxiliar value wich computes T1^2+T2^2+T3^2... Useful for generalizing the Eikonal solver. */
		
		std::array<double, grid_t::getNDims()> Tvalues;  /*!< Auxiliar array with values T0,T1...Tn-1 variables in the Discretized Eikonal Equation. */
		std::array<double, grid_t::getNDims()> TTvalues;  /*!< Auxiliar array with values T0^2,T1^2...Tn-1^2 variables in the Discretized Eikonal Equation. */
		std::array <int, 2*grid_t::getNDims()> neighbors;  /*!< Auxiliar array which stores the neighbor of each iteration of the computeFM() function. */
};


#endif /* FASTMARCHING_H_*/
