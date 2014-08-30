/*! \file fm2star.hpp
    \brief Templated class which computes the Fast Marching Square Star (FM2*).
    
    It uses as a main container the nDGridMap class. The nDGridMap type T
    has to be an FMStarCell or something inherited from it.
    
    The leafsize of the grid map is ignored since it has to be > = 1 and that
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
          A. Valero, J.V. Gómez, S. Garrido and L. Moreno, Fast Marching Method for Safer, More Efficient Mobile Robot Trajectories.
    
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
    along with this program. If not, see  < http://www.gnu.org/licenses/>.
*/


#ifndef FASTMARCHING2STAR_H_
#define FASTMARCHING2STAR_H_

#include  <iostream>
#include  <cmath>
#include  <algorithm>
#include  <numeric>
#include  <fstream>
#include  <array>
#include  <limits>

#include "../fmdata/fmcell.h"
#include "../fmm/fastmarching.hpp"
#include "../gradientdescent/gradientdescent.hpp"

template  <  class grid_t, class heap_t = FMDaryHeap <FMCell> >  class FastMarching2Star : public FastMarching  < grid_t, heap_t> {
	
    public:
        typedef std::vector< std::array< double, grid_t::getNDims() > > path_t;

        FastMarching2Star  < grid_t, heap_t> () {
			
		};
        virtual ~FastMarching2Star  < grid_t, heap_t> () {};

        using FastMarching < grid_t, heap_t>::grid_;
        using FastMarching < grid_t, heap_t>::neighbors;
        using FastMarching < grid_t, heap_t>::init_points_;
        using FastMarching < grid_t, heap_t>::Tvalues;
        using FastMarching < grid_t, heap_t>::TTvalues;
        using FastMarching < grid_t, heap_t>::narrow_band_;

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
         * computes the initialization of the Fast Marching Square calling
         * the init() function.
         *
         * @param contains the indice of the init point.
         *
         * @param contains the indices of the init points corresponding to all black cells.
         *
         * @param contains the indice of the goal point.
         *
         * @see init()
         */

        virtual void setInitialAndGoalPoints
        (const std::vector <int> & initial_point, const std::vector <int> & fmm2_sources, const int goal_point) {
            initial_point_ = initial_point;
            fmm2_sources_ = fmm2_sources;
            goal_idx_ = goal_point;
            grid_->idx2coord(goal_idx_, goal);
        }

        /**
         * Set the initial points by the indices in the nDGridMap,
         * computes the initialization of the Fast Marching Method calling
         * the init() function and the euclidean distance between every pixel
         * calling the precomputeDistances() function.
         *
         * @param contains the indices of the init points.
         *
         * @see init()
         *
         * @see precomputeDistances()
         */
        virtual void setInitialPoints
        (const std::vector <int> & init_points) {
            init_points_ = init_points;
            for (const int &i: init_points) {
                grid_->getCell(i).setArrivalTime(0);
                grid_->getCell(i).setState(FMState::FROZEN);
            }

            if (init_points.size()>1)
                init(false, false);
            else
            {
                precomputeDistances();
                init(true, true);
            }
        }

        /**
         * Calculate the euclidean distance between every pixel filling an
         * array with all of them. This method has been generalized to be used
         * on n-dimensional grids.
         */
        virtual void precomputeDistances
        () {
            distances = new double[grid_->size()];
            std::array < int,grid_->getNDims()> coords;
            double dist = 0;

            for (size_t i = 0; i  <  grid_->size(); ++i)
            {
                dist = 0;
                grid_->idx2coord(i, coords);

                for (size_t j = 0; j  <  coords.size(); ++j)
                    dist += pow(coords[j], 2);

                distances[i] = std::sqrt(dist);
            }
        }

        /**
         * Extract the euclidean distance calculated from precomputeDistances
         * function distance between two positions. It obtain the difference vector
         * coordinates in every dimension between a given and a goal point and convert
         * it into an index of the distances array.
         *
         * @param idx index of the cell to be evaluated.
         *
         * @see precomputeDistances()
         */
        virtual double getPrecomputedDistance
        (const int idx) {
            std::array < int,grid_->getNDims()> position, distance;

            grid_->idx2coord(idx, position);

            for (int i = 0; i < grid_->getNDims(); i++)
               distance[i] = std::abs(position[i]-goal[i]);

            int idx_dist;
            grid_->coord2idx(distance, idx_dist);

            return distances[idx_dist];
        }

         /**
         * Internal function although it is set to public so it can be accessed if desired.
         *
         * Computes the Fast Marching Method initialization from the initial points given. Programmed following the paper:
            A. Valero, J.V. Gómez, S. Garrido and L. Moreno, The Path to Efficiency: Fast Marching Method for Safer,
            More Efficient Mobile Robot Trajectories, IEEE Robotics and Automation Magazine, Vol. 20, No. 4, 2013.
         *
         * @param selection of mode to expand the wave.
         *
         * @see setInitialPoints()
         */
        virtual void init
        (const bool stop = true, const bool star = true) {
            // TODO: neighbors computed twice for every cell. We can save time here.
            // TODO: check if the previous steps have been done (loading grid map and setting initial points.)
            int j = 0;
            int n_neighs = 0;
            for (int &i: init_points_) { // For each initial point
                n_neighs = grid_->getNeighbors(i, neighbors);
                for (int s = 0; s  <  n_neighs; ++s){  // For each neighbor
                    j = neighbors[s];
                    if ((grid_->getCell(j).getState() == FMState::FROZEN) || grid_->getCell(j).isOccupied() || grid_->getCell(j).getVelocity() == 0) // If Frozen or obstacle
                        continue;
                    else {
                        double new_arrival_time = 0;

                        new_arrival_time = solveEikonal(j, star);

                        if (grid_->getCell(j).getState() == FMState::NARROW) { // Updating narrow band if necessary.
                            if (new_arrival_time  <  grid_->getCell(j).getArrivalTime()) {
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
            } // For each initial point.
        } // init()


        //IMPORTANT NOTE: Assuming inc(1) = inc(y)  = ... =  leafsize_
        // Possible improvement: If we include the neighbors in the cells information
        // this could be (most probably) speeded up.
        // This implementation is focused to be used with any number of dimensions.

        /**
        * Solves the Eikonal equation for a given cell. This function is generalized
        * to any number of dimensions.
        *
        * @param idx index of the cell to be evaluated.
        *
        * @param selection of mode to resolve Eikonal equation depending of the heuristic
        *
        * @return the distance (or time of arrival) value.
        */
       virtual double solveEikonal
       (const int & idx, const bool star = false) {
           // TODO: Here neighbors are computed and then in the computeFM. There should be a way to avoid computing
           // neighbors twice.
           int a = grid_t::getNDims(); // a parameter of the Eikonal equation.

           double updatedT;
           sumT = 0;
           sumTT = 0;

           for (int dim = 0; dim  <  grid_t::getNDims(); ++dim) {
               double minTInDim = grid_->getMinValueInDim(idx, dim);
               if (!isinf(minTInDim)) {
                   Tvalues[dim] = minTInDim;
                   sumT +=  Tvalues[dim];
                   TTvalues[dim] = Tvalues[dim]*Tvalues[dim];
                   sumTT +=  TTvalues[dim];
               }
               else {
                   Tvalues[dim] = 0;
                   TTvalues[dim] = 0;
                   a -= 1 ;
               }
           }

           double b = -2*sumT;
           double c = sumTT - grid_->getLeafSize() * grid_->getLeafSize()/(grid_->getCell(idx).getVelocity()*grid_->getCell(idx).getVelocity()); // leafsize not taken into account here.
           double quad_term = b*b - 4*a*c;
           if (quad_term  <  0) {
               double minT = *(std::min_element(Tvalues.begin(), Tvalues.end()));
               updatedT = 1/(grid_->getCell(idx).getVelocity()*grid_->getCell(idx).getVelocity()) + minT; // leafsize not taken into account here.
           }
           else
               updatedT = (-b + sqrt(quad_term))/(2*a);

           // Apply the FM2* heuristic
           if (star) {
               double distance = getPrecomputedDistance(idx);

               updatedT +=  distance/grid_->getCell(idx).getVelocity();
           }

           return updatedT;
       }

        /**
         * Main Fast Marching Function. It requires to call first the setInitialPoints() function.
         *
         * @param selection of mode to expand the wave.
         *
         * @see setInitialPoints()
         */
        virtual void computeFM
        (const bool stop = true, const bool star = true) {
            // TODO: check if the previous steps have been done (initialization).
            int j =  0;
            int n_neighs = 0;
            bool stopWavePropagation = 0;

            while (narrow_band_.size() > 0 && stopWavePropagation == 0) {
                int idxMin = narrow_band_.popMinIdx();
                n_neighs = grid_->getNeighbors(idxMin, neighbors);
                grid_->getCell(idxMin).setState(FMState::FROZEN);

                for (int s = 0; s  <  n_neighs; ++s) {
                    j = neighbors[s];
                    if ((grid_->getCell(j).getState() == FMState::FROZEN) || grid_->getCell(j).isOccupied()) // If Frozen or obstacle
                        continue;
                    else {
                        double new_arrival_time = solveEikonal(j, star);

                        if (grid_->getCell(j).getState() == FMState::NARROW) { // Updating narrow band if necessary.
                            if (new_arrival_time  <  grid_->getCell(j).getArrivalTime()) {
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

        /**
         * Main Fast Marching Square Star Function with velocity saturation. It requires to call first the setInitialPoints() function.
         *
         * @param saturation distance
         *
         * @param max velocity value of the robot.
         *
         * @see setInitialPoints()
         */

        virtual void computeFM2Star
        (const float maxDistance = -1) {
            if (maxDistance != -1) {
                maxDistance_ = maxDistance;
                computeFirstPotential(true);
            } else
                computeFirstPotential();

            setInitialPoints(initial_point_);
            computeFM(true, true);
        }

        /**
         * Computes the path from the given index to a minimum (the one
         * gradient descent choses) and returns the velocity.
         *
         * No checks are done (points in the borders, points in obstacles...).
         *
         * The included scripts will parse the saved path.
         *
         * @param path the resulting path (output).
         *
         * @param velocity the resulting path (output).
         */

        virtual void computePath
        (path_t * p, std::vector <double> * path_velocity) {
            path_t* path_ = p;
            constexpr int ndims = grid_->getNDims();

            GradientDescent < nDGridMap < FMCell, ndims > > grad;
            grad.apply(*grid_,goal_idx_,*path_, *path_velocity);
        }
		
private:

    /**
     * Computes the first potential expansion of the wave to perform the velocity map.
     *
     * @param select if the potential is saturated
     */

    void computeFirstPotential
    (bool saturate = false) {
        setInitialPoints(fmm2_sources_);
        computeFM(false, false);

        //Rescaling and saturating to relative velocities: [0-1]
        float maxValue = grid_->getMaxValue();
        double maxVelocity = 0;

        if (saturate)
            maxVelocity = maxDistance_/grid_->getLeafSize(); // Calculate max velocity using the max distance and the leaf size of the cell

        for (int i = 0; i < grid_->size(); i++) {
            double vel = grid_->getCell(i).getValue()/maxValue;

            if (saturate)
                if (vel < maxVelocity)
                    grid_->getCell(i).setVelocity(vel/maxVelocity);
                else
                    grid_->getCell(i).setVelocity(1);
            else
                grid_->getCell(i).setVelocity(vel);

            grid_->getCell(i).setValue(inf_);
            grid_->getCell(i).setState(FMState::OPEN);
        }
    }

    protected:
        double inf_ = std::numeric_limits < double>::infinity();

        double sumT; /*! <  Auxiliar value wich computes T1+T2+T3... Useful for generalizing the Eikonal solver. */
        double sumTT; /*! <  Auxiliar value wich computes T1^2+T2^2+T3^2... Useful for generalizing the Eikonal solver. */
		
        int goal_idx_; /*! <  Goal point for the Fast Marching Square Star. */
        std::vector <int> fmm2_sources_;	/*! <  Wave propagation sources for the Fast Marching Square Star. */
        std::vector <int> initial_point_;	/*! <  Initial point for the Fast Marching Square Star. */
        std::array <int,grid_t::getNDims()> goal; /*! <  Goal coord for the Fast Marching Square Star. */
        double *distances; /*! <  Auxiliar container of euclidean distances for the Fast Marching Square Star heuristic. */
        double maxDistance_; /*!< Distance value to saturate the first potential. */
};


#endif /* FASTMARCHING2STAR_H_*/
