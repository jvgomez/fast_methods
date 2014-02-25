/*! \file fastmarching.hpp
    \brief Templated class which computes the basic Fast Marching Method.
    
    It uses as a main container the nDGridMap class. The nDGridMap type T
    has to be an FMCell or something inherited from it.
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

// TODO: compute geodesic method.

template <class T, size_t ndims> class FastMarching {
	
    public: 
        FastMarching <T,ndims> () {};
        virtual ~FastMarching <T,ndims>() {};
            
         /**
          * Sets the input grid in which operations will be performed.
          * 
          * @param g input grid map.
          */   
        void setEnvironment 
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
		void setInitialPoints
		(const std::vector<int> & init_points) {
			init_points_ = init_points;
			for (const int &i: init_points) {
				grid_->cells_[i].setArrivalTime(0);
				grid_->cells_[i].setState(FMState::FROZEN);
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
		void init
		() {
			// TODO: neighbours computed twice for every cell. We can save time here.
			// TODO: check if the previous steps have been done (loading grid map and setting initial points.)
			int j = 0;
			int n_neighs = 0;
			for (int &i: init_points_) { // For each initial point
				n_neighs = grid_->getNeighbours(i, neighbours);
				for (int s = 0; s < n_neighs; ++s){  // For each neighbour
					j = neighbours[s];
					if (grid_->cells_[j].getState() == FMState::FROZEN)
						continue;
					else {
						float new_arrival_time = solveEikonal(j);
						if (grid_->cells_[j].getState() == FMState::NARROW) { // Updating narrow band if necessary.
							if (new_arrival_time < grid_->cells_[j].getArrivalTime()) {
								//console::warning("Narrow band value updated!");
								grid_->cells_[j].setArrivalTime(new_arrival_time);
								narrow_band_.increase(&grid_->cells_[j]);
							}
						}
						else {
							grid_->cells_[j].setState(FMState::NARROW);
							grid_->cells_[j].setArrivalTime(new_arrival_time);
							narrow_band_.push(&grid_->cells_[j]);
						} // Neighbours open.
					} // Neighbours not frozen.
				} // For each neighbour.
			} // For each initial point.
		} // init()
			
		/*void saveGrid
        (const std::string & filename, const int whattosave = 0) {	
			grid_->saveGrid(filename,whattosave);
		}*/
		
		//IMPORTANT NOTE: Assuming inc(1) = inc(y) =...= leafsize_
		// Possible improvement: If we include the neighbours in the cells information
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
		float solveEikonal
		(const int & idx) {
			// TODO: Here neighbours are computed and then in the computeFM. There should be a way to avoid computing
			// neighbours twice.
			float a = ndims; // a parameter of the Eikonal equation.
			
			float updatedT;
			sumT = 0;
			sumTT = 0;

			for (int dim = 0; dim < ndims; ++dim) {
				float minTInDim = grid_->getMinValueInDim(idx, dim);
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
					
			
			float b = -2*sumT;
			float c = sumTT - (leafsize_*leafsize_)/(grid_->cells_[idx].getVelocity()*grid_->cells_[idx].getVelocity());
			float quad_term = b*b - 4*a*c;
			if (quad_term < 0) {
				//console::warning("Quad term < 0");
				float minT = *(std::min_element(Tvalues.begin(), Tvalues.end()));
				updatedT = (leafsize_*leafsize_)/(grid_->cells_[idx].getVelocity()*grid_->cells_[idx].getVelocity()) + minT;
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
		void computeFM
		() {
			// TODO: check if the previous steps have been done (initialization).
			int j= 0;
			int n_neighs = 0;
			while (narrow_band_.size() > 0) {
				int idxMin = narrow_band_.popMinIdx();
				n_neighs = grid_->getNeighbours(idxMin, neighbours);
				grid_->cells_[idxMin].setState(FMState::FROZEN);

				for (int s = 0; s < n_neighs; ++s) {
					j = neighbours[s];
					if (grid_->cells_[j].getState() == FMState::FROZEN)
						continue;
					else {
						float new_arrival_time = solveEikonal(j);
						if (grid_->cells_[j].getState() == FMState::NARROW) { // Updating narrow band if necessary.
							if (new_arrival_time < grid_->cells_[j].getArrivalTime()) {
								grid_->cells_[j].setArrivalTime(new_arrival_time);
								narrow_band_.increase(&grid_->cells_[j]);
							}
						}
						else {
							grid_->cells_[j].setState(FMState::NARROW);
							grid_->cells_[j].setArrivalTime(new_arrival_time);
							narrow_band_.push(&grid_->cells_[j]);
						} // Neighbours open.
					} // Neighbours not frozen.
				} // For each neighbour.
			} // while narrow band not empty
		}
        
        /*void computeGeodesic
        (const int & idx, std::vector<int> & path_indices) {
			if (ndims_ == 2)
				GradientDescent<nDGridMap<T>>::apply2D(grid_,idx, path_indices);
			else if (ndims_ == 3)
				GradientDescent<nDGridMap<T>>::apply3D(grid_,idx, path_indices);
			else
				console::error("Grimap dimensions > 3. Geodesic extraction only implemented for 2D and 3D.");
		}
		
		void saveGeodesic
		(const std::string filename, const std::vector<int> & path) {
			
			std::ofstream ofs;
			ofs.open (filename,  std::ofstream::out | std::ofstream::trunc);
			
			ofs << "Fast Marching geodesic" << std::endl;
			ofs << grid_->getLeafSize() << std::endl << grid_->getNDims() ;
			std::vector<int> dimsize = grid_->getDimSizes();
			for (int i = 0; i < grid_->getNDims(); ++i)
				ofs << std::endl << dimsize[i] << "\t";
				   
			for (int i = 0; i < path.size(); ++i)
				ofs << std::endl << path[i];  
		}*/
		
    private:
		nDGridMap<T, ndims> *  grid_; /*!< Main container.. */
		std::vector<int> init_points_;	/*!< Initial points for the Fast Marching Method. */
		FMFibHeap narrow_band_; /*!< Instance of the Fibonacci Heap used. */
		
		float leafsize_; /*!< Although it is on grid, it is stored here so that it has not to be accessed. */
		float sumT; /*!< Auxiliar value wich computes T1+T2+T3... Useful for generalizing the Eikonal solver. */
		float sumTT; /*!< Auxiliar value wich computes T1^2+T2^2+T3^2... Useful for generalizing the Eikonal solver. */
		
		std::array<float,ndims> Tvalues;  /*!< Auxiliar array with values T0,T1...Tn-1 variables in the Discretized Eikonal Equation. */
		std::array<float,ndims> TTvalues;  /*!< Auxiliar array with values T0^2,T1^2...Tn-1^2 variables in the Discretized Eikonal Equation. */
		std::array <int,2*ndims> neighbours;  /*!< Auxiliar array which stores the neighbour of each iteration of the computeFM() function. */
};


#endif /* FASTMARCHING_H_*/
