/*! \file fm2directional.hpp
    \brief Templated class which computes the Fast Marching Directional (FM2Directional).

    It uses as a main container the nDGridMap class. The nDGridMap type T
    has to be an FMDirectionalCell or something inherited from it.

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


#ifndef FASTMARCHING2DIRECTIONAL_H_
#define FASTMARCHING2DIRECTIONAL_H_

#include <iostream>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <fstream>
#include <array>
#include <limits>

#include <fstream>

#include "../fmdata/fmdirectionalcell.h"
#include "../fmm/fastmarching.hpp"
#include "../gradientdescent/gradientdescent.hpp"

#define PI 4 * std::atan(1)

template < class grid_t, class heap_t = FMDaryHeap<FMDirectionalCell> >  class FastMarching2Directional : public FastMarching <grid_t, heap_t> {

    public:
        typedef std::vector< std::array< double, grid_t::getNDims() > > path_t;

        FastMarching2Directional <grid_t, heap_t> () {

		};
        virtual ~FastMarching2Directional <grid_t, heap_t> () {};

        using FastMarching<grid_t, heap_t>::grid_;
        using FastMarching<grid_t, heap_t>::neighbors;
        using FastMarching<grid_t, heap_t>::init_points_;
        using FastMarching<grid_t, heap_t>::Tvalues;
        using FastMarching<grid_t, heap_t>::TTvalues;
        using FastMarching<grid_t, heap_t>::narrow_band_;

         /**
          * Sets the input grid in which operations will be performed.
          *
          * @param g input grid map.
          */
        virtual void setEnvironment
        (grid_t * g) {
            grid_ = g;
            narrow_band_.setMaxSize(grid_->size());
            ndims_ = grid_->getNDims();
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

        }

        /**
         * Set the initial points by the indices in the nDGridMap and
         * computes the initialization of the Fast Marching Method calling
         * the init() function.
         *
         * @param contains the indices of the init points.
         *
         * @param param to select if the velocity profile must be saved
         *
         * @see init()
         */
        virtual void setInitialPoints
        (const std::vector<int> & init_points, const bool save_velocity = false) {
            init_points_ = init_points;
            for (const int &i: init_points) {
                grid_->getCell(i).setArrivalTime(0);
                grid_->getCell(i).setDirectionalTime(0);
                grid_->getCell(i).setState(FMState::FROZEN);
            }

            if (init_points.size()>1)
                init();
            else
                init(save_velocity, true);
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
        (const bool save_velocity = false, const bool directional = false) {
            // TODO: neighbors computed twice for every cell. We can save time here.
            // TODO: check if the previous steps have been done (loading grid map and setting initial points.)
            int j = 0;
            int n_neighs = 0;
            for (int &i: init_points_) { // For each initial point
                n_neighs = grid_->getNeighbors(i, neighbors);
                for (int s = 0; s < n_neighs; ++s){  // For each neighbor
                    j = neighbors[s];
                    if ((grid_->getCell(j).getState() ==  FMState::FROZEN) || grid_->getCell(j).isOccupied() || grid_->getCell(j).getVelocity() ==  0) // If Frozen or obstacle
                        continue;
                    else {
                        double new_arrival_time = solveEikonal(j);
                        double dir_time = 0;

                        if (directional ==  true)
                            dir_time = new_arrival_time;

                        if (grid_->getCell(j).getState() ==  FMState::NARROW) { // Updating narrow band if necessary.
                            if (new_arrival_time < grid_->getCell(j).getArrivalTime()) {
                                grid_->getCell(j).setArrivalTime(new_arrival_time);

                                if (save_velocity)
                                    velocity_map_[j] = vel;

                                if (directional)
                                    grid_->getCell(j).setDirectionalTime(dir_time);
                                narrow_band_.increase( &(grid_->getCell(j))  ) ;
                            }
                        }
                        else {
                            grid_->getCell(j).setState(FMState::NARROW);
                            grid_->getCell(j).setArrivalTime(new_arrival_time);

                            if (save_velocity)
                                velocity_map_[j] = vel;

                            if (directional ==  true)
                                grid_->getCell(j).setDirectionalTime(dir_time);
                            narrow_band_.push( &(grid_->getCell(j)) );
                        } // neighbors open.
                    } // neighbors not frozen.
                } // For each neighbor.
            } // For each initial point.
        } // init2()

        /**
        * Solves the gradient value for a given cell. This function is generalized
        * to any number of dimensions.
        *
        * @param gradient results vector.
        *
        * @param idx index of the cell to be evaluated.
        *
        * @param selection of attribute to calculate the gradient
        *
        * @return the distance (or time of arrival) value.
        */
        virtual void solveGradient
        (std::array<double, grid_t::getNDims()> & grads, const int idx, const bool velocity = false)
        {
            dimsize_ = grid_->getDimSizes();

            d_[0] = dimsize_[0];
            for (int i = 1; i < ndims_; ++i)
                d_[i] = dimsize_[i]*d_[i-1];

            // Apply gradient to velocity value
            if (velocity ==  true)
            {
                // First dimension done apart.
                grads[0] = - grid_->getCell(idx-1).getVelocity()/2 + grid_->getCell(idx+1).getVelocity()/2;

                if (isinf(grads[0]))
                    grads[0] = sgn<double>(grads[0]);

                for (int i = 1; i < ndims_; ++i) {
                    grads[i] = - grid_->getCell(idx-d_[i-1]).getVelocity()/2 + grid_->getCell(idx+d_[i-1]).getVelocity()/2;
                    if (isinf(grads[i]))
                        grads[i] = sgn<double>(grads[i]);
                }
            }
            // Apply gradient to wave expansione
            else
            {
                double min_dim;

                if (grid_->getCell(idx-1).getValue() > grid_->getCell(idx+1).getValue())
                    min_dim = grid_->getCell(idx+1).getValue();
                else
                    min_dim = grid_->getCell(idx-1).getValue();

                // First dimension done apart.
                grads[0] = - (grid_->getCell(idx).getValue() - min_dim);

                if (isinf(grads[0]))
                    grads[0] = 0;

                for (int i = 1; i < ndims_; ++i) {
                    if (grid_->getCell(idx-d_[i-1]).getValue() > grid_->getCell(idx+d_[i-1]).getValue())
                        min_dim = grid_->getCell(idx+d_[i-1]).getValue();
                    else
                        min_dim = grid_->getCell(idx-d_[i-1]).getValue();

                    grads[i] = - (grid_->getCell(idx).getValue() - min_dim);

                    if (isinf(grads[i]))
                        grads[i] = 0;
                }
            }
        }

        //IMPORTANT NOTE: Assuming inc(1) = inc(y)  = ... =  leafsize_
        // Possible improvement: If we include the neighbors in the cells information
        // this could be (most probably) speeded up.
        // This implementation is focused to be used with any number of dimensions.

        /**
        * Solves the Eikonal equation for a given cell using the heuristic criteria of the FM2*.
        * This function is generalized to any number of dimensions.
        *
        * @param idx index of the cell to be evaluated.
        *
        * @param idx index of the source cell of the wave. If this value is -1 the heuristic is not applied
        *
        * @return the distance (or time of arrival) value.
        */
       virtual double solveEikonal
       (const int & idx, const int & idx_source = -1) {
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

           vel = 0;

           if (idx_source == -1)
               vel = grid_->getCell(idx).getVelocity();
           else {
               // Calculate velocity and wave gradient
               solveGradient(grads_velocity, idx, true);

               solveGradient(grads_wave, idx, false);

               std::array<int,grid_->getNDims()> coords_source, coords_wave;

               grid_->idx2coord(idx_source, coords_source);
               grid_->idx2coord(idx, coords_wave);

               // Calculate the gradients angle
               angle_velocity = std::atan2(grads_velocity[1], grads_velocity[0]);
               angle_wave = std::atan2(grads_wave[1], grads_wave[0]);

               if (angle_velocity<0)
                   angle_velocity +=  2 * PI;

               if (angle_wave<0)
                   angle_wave +=  2 * PI;

               // Calculate the angle between the gradients
               diff_angle = std::abs(angle_velocity-angle_wave);

               if (diff_angle > PI )
                   diff_angle = 2 * PI - diff_angle;

              vel = 0;

               // Apply the heuristic
               if (std::abs(diff_angle) <=  (0.5 * PI) && grid_->getCell(idx).getVelocity() > 0.05)
                   vel = 1;
               else
                   vel = grid_->getCell(idx).getVelocity();
           }

           double b = -2*sumT;
           double c = sumTT - grid_->getLeafSize() * grid_->getLeafSize()/(vel * vel); // leafsize not taken into account here.
           double quad_term = b*b - 4*a*c;
           if (quad_term < 0) {
               double minT = *(std::min_element(Tvalues.begin(), Tvalues.end()));
               updatedT = 1/(vel * vel) + minT; // leafsize not taken into account here.
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
         * @param select directionally
         *
         * @see setInitialPoints()
         */

        virtual void computeFM
        (const bool stop = true, const bool directional = false) {
            // TODO: check if the previous steps have been done (initialization).
            int j =  0;
            int n_neighs = 0;
            bool stopWavePropagation = 0;

            while (narrow_band_.size() > 0 && stopWavePropagation ==  0) {
                int idxMin = narrow_band_.popMinIdx();
                n_neighs = grid_->getNeighbors(idxMin, neighbors);
                grid_->getCell(idxMin).setState(FMState::FROZEN);

                for (int s = 0; s < n_neighs; ++s) {
                    j = neighbors[s];
                    if ((grid_->getCell(j).getState() ==  FMState::FROZEN) || grid_->getCell(j).isOccupied()) // If Frozen or obstacle
                        continue;
                    else {
                        double new_arrival_time = solveEikonal(j);

                        double dir_time = 0;

                        if (directional ==  true)
                            dir_time = solveEikonal(j, idxMin);

                        if (grid_->getCell(j).getState() ==  FMState::NARROW) { // Updating narrow band if necessary.
                            if (new_arrival_time < grid_->getCell(j).getArrivalTime()) {
                                grid_->getCell(j).setArrivalTime(new_arrival_time);
                                narrow_band_.increase( &(grid_->getCell(j)) );
                                velocity_map_[j] = vel;
                            }
                            if (directional)
                                if (dir_time < grid_->getCell(j).getDirectionalTime())
                                    grid_->getCell(j).setDirectionalTime(dir_time);
                        }
                        else {
                            grid_->getCell(j).setState(FMState::NARROW);
                            grid_->getCell(j).setArrivalTime(new_arrival_time);
                            velocity_map_[j] = vel;

                            if (directional)
                                grid_->getCell(j).setDirectionalTime(dir_time);
                            narrow_band_.push( &(grid_->getCell(j)) );
                        } // neighbors open.
                    } // neighbors not frozen.
                    if (idxMin ==  initial_point_[0] && stop)
                        stopWavePropagation = 1;
                } // For each neighbor.
            } // while narrow band not empty
        }

        /**
         * Main Fast Marching Square Directional Function with velocity saturation. It requires to call first the setInitialPoints() function.
         *
         * @param saturation distance. If this value is -1 the potential is not saturated
         *
         * @see setInitialPoints()
         */

        virtual void computeFM2Directional
        (const float maxDistance = -1) {
            velocity_map_.resize(grid_->size());
            if (maxDistance != -1) {
                maxDistance_ = maxDistance;
                computeFirstPotential(true);
            } else
                computeFirstPotential();

            // The wave has to be expanded from the goal to the intial point
            std::vector<int> goals;
            goals.push_back(goal_idx_);
            setInitialPoints(goals, true);
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
            constexpr int ndims=grid_->getNDims();

            GradientDescent< nDGridMap<FMDirectionalCell, ndims> > grad;
            grad.apply_directional(*grid_,initial_point_[0],*path_, velocity_map_, *path_velocity);
        }

        /**
        * For a cell with index idx, obtains the minimum value of the neigbours in dimension dim looking the
        * DirectionalTime value.
        *
        * @param idx index of the cell accessed.
        *
        * @param dim dimension in which the minimum is examinated (0: x, 1: y, 2: z, etc).
        *
        * @return the corresponding minimum value.
        * */

        double getMinValueInDimDirectional
        (const int idx, const int dim)   {
           // n_neighs = 0; // How many neighbors obtained in that dimension.
            constexpr int ndims = grid_->getNDims();
            std::array<int, ndims> n;
            int n_neighs = grid_->getNumberNeighborsInDim(idx,n,dim);

            if (n_neighs ==  1)
                return grid_->getCell(n[0]).getDirectionalTime();
            else
                return (grid_->getCell(n[0]).getDirectionalTime()<grid_->getCell(n[1]).getDirectionalTime()) ? grid_->getCell(n[0]).getDirectionalTime() : grid_->getCell(n[1]).getDirectionalTime();

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
            computeFM(false);

            //Rescaling and saturating to relative velocities: [0-1]
            float maxValue = grid_->getMaxValue();
            double maxVelocity = 0;

            if (saturate)
                maxVelocity = maxDistance_/grid_->getLeafSize(); // Calculate max velocity using the max distance and the leaf size of the cell

            for (int i = 0; i < grid_->size(); i++) {
                double velocity = grid_->getCell(i).getValue()/maxValue;

                if (saturate)
                    if (velocity < maxVelocity)
                        grid_->getCell(i).setVelocity(velocity/maxVelocity);
                    else
                        grid_->getCell(i).setVelocity(1);
                else
                    grid_->getCell(i).setVelocity(velocity/maxVelocity);

                grid_->getCell(i).setValue(inf_);
                grid_->getCell(i).setState(FMState::OPEN);
            }
        }

    protected:
        double inf_ = std::numeric_limits<double>::infinity();

        double sumT; /*!< Auxiliar value wich computes T1+T2+T3... Useful for generalizing the Eikonal solver. */
        double sumTT; /*!< Auxiliar value wich computes T1^2+T2^2+T3^2... Useful for generalizing the Eikonal solver. */
        double sumDistance; /*!< Auxiliar value wich computes euclidean distance between narrow band and goal point. Useful for generalizing the Eikonal solver with euristic. */

        double vel; /*!< Auxiliar value wich contains the velocity of the cell used on the Eikonal solver. */
        double maxDistance_; /*!< Distance value to saturate the first potential. */

        int goal_idx_; /*!< Goal point for the Fast Marching Square Star. */
        std::vector<int> fmm2_sources_;	/*!< Wave propagation sources for the Fast Marching Square Star. */
        std::vector<int> initial_point_;	/*!< Initial point for the Fast Marching Square Star. */

        std::array<int, grid_t::getNDims()-1> d_;
        std::array<int, grid_t::getNDims()> dimsize_;
        std::array<double, grid_t::getNDims()> grads_velocity, grads_wave;
        std::vector<double> velocity_map_; /*!< Auxiliar vector which contains the final velocity map of the environment. */

        int ndims_;
        double angle_wave, angle_velocity;
        double diff_angle;
};


#endif /* FASTMARCHING2STAR_H_*/
