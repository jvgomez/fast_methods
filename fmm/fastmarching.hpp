#ifndef FASTMARCHING_H_
#define FASTMARCHING_H_

#include <iostream>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <fstream>

#include "../fmdata/fmcell.h"
#include "../ndgridmap/ndgridmap.hpp"
#include "../console/console.h"
#include "../fmdata/fmfibheap.hpp"
#include "../gradientdescent/gradientdescent.hpp"


template <class T> class FastMarching {
	
    public: 
		
        FastMarching <T> () {};
        virtual ~FastMarching <T>() {};
            
        void setEnvironment 
        (nDGridMap<T> * g) {
			grid_ = g;
			ndims_ = grid_->getNDims();
			Tvalues.resize(ndims_);
			TTvalues.resize(ndims_);
			leafsize_ = grid_->getLeafSize();
			narrow_band_.setMaxSize(grid_->size());
			
			neighbours.resize(2*ndims_);
		}
        
			
		void setInitialPoints
		(const std::vector<int> & init_points) {
			init_points_ = init_points;
			for (const int &i: init_points) {
				grid_->cells_[i].setArrivalTime(0);
				grid_->cells_[i].setState(FMState::FROZEN);
			}
		}	
		
		// Do the rest of the FMM initialization.
		void init
		() {
			// Programmed following the paper:
			// A. Valero, J.V. Gómez, S. Garrido and L. Moreno, The Path to Efficiency: Fast Marching Method for Safer,
			// More Efficient Mobile Robot Trajectories, IEEE Robotics and Automation Magazine, Vol. 20, No. 4, 2013.
			
			// TODO: neighbours computed twice for every cell. We can save time here.
			// TODO: check if the previous steps have been done (loading grid map and setting initial points.)
			int j = 0;
			int n_neighs = 0;
			for (int &i: init_points_) { // For each initial point
				n_neighs = grid_->getNeighbours(i, neighbours);
				for (int s = 0; s < n_neighs; ++s){ 
					j = neighbours[s];
					if (grid_->cells_[j].getState() == FMState::FROZEN)
						continue;
					else {
						float new_arrival_time = solveEikonal(j);
						if (grid_->cells_[j].getState() == FMState::NARROW) { // Updating narrow band if necessary.
							if (new_arrival_time < grid_->cells_[j].getArrivalTime()) {
								console::warning("Narrow band value updated!");
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
			
		void saveGrid
        (const std::string & filename, const int whattosave = 0) {	
			grid_->saveGrid(filename,whattosave);
		}
		
		//IMPORTANT NOTE: Assuming inc(1) = inc(y) =...= leafsize_
		// Possible improvement: If we include the neighbours in the cells information
		// this could be (most probably) speeded up.
		// This implementation is focused to be used with any number of dimensions.
		inline float solveEikonal
		(const int & idx) {
			float a = ndims_; // a parameter of the Eikonal equation.
			
			float updatedT;
			sumT = 0;
			sumTT = 0;

			for (int dim = 0; dim < ndims_; ++dim) {
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
			
			Tvalues.clear();
			TTvalues.clear();
			return updatedT;
		}	
		
		void computeFM
		() {
			int j= 0;
			int n_neighs = 0;
			while (narrow_band_.size() > 0) {
				int idxMin = narrow_band_.popMinIdx();
				// TODO: check if the previous steps have been done (initialization).
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
		nDGridMap<T> *  grid_;
		std::vector<int> init_points_;	
		FMFibHeap narrow_band_;
		
		
		//Aux vectors and variables declared here to avoid reallocating everytime.
		std::vector<float> Tvalues;  //Tvalues are the T0,T1...Tn-1 variables in the Discretized Eikonal Equation.
		std::vector<float> TTvalues; //Tvalues are the T0^2,T1^2...Tn-1^2 variables in the Discretized Eikonal Equation.
		//neighbours4c neighbours;     //std::array cannot be used since it has to be size 2*ndims_
		std::vector <int> neighbours;
		
		
		int ndims_;
		float leafsize_;
		float sumT;
		float sumTT;
};


#endif /* FASTMARCHING_H_*/
