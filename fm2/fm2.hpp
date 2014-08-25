/*! \file fm2.hpp
    \brief Templated class which computes the Fast Marching Square (FM2).

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
        FMM2:
          J.V. Gómez, A. Lumbier, S. Garrido and L. Moreno, The Path to Efficiency: Fast Marching Methods in Path Planning.

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


#ifndef FASTMARCHING2_H_
#define FASTMARCHING2_H_

#include <iostream>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <fstream>
#include <array>
#include <limits>

#include "../fmm/fastmarching.hpp"
#include "../gradientdescent/gradientdescent.hpp"

using namespace std::chrono;

template < class grid_t, class path_t, class heap_t = FMDaryHeap<FMCell>  >  class FastMarching2 {

    public:
        FastMarching2 <grid_t, path_t, heap_t> () {

        };
        virtual ~FastMarching2 <grid_t, path_t, heap_t> () {};

         /**
          * Sets the input grid in which operations will be performed.
          *
          * @param g input grid map.
          */
        virtual void setEnvironment
        (grid_t * g) {
            grid_ = g;
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
        (const std::vector<int> & initial_point, const std::vector<int> & fmm2_sources, const int goal_point) {
            initial_point_ = initial_point;
            fmm2_sources_ = fmm2_sources;
            goal_idx_ = goal_point;
        }

        /**
         * Main Fast Marching Square Function. It requires to call first the setInitialPoints() function.
         *
         * @see setInitialPoints()
         */

        virtual void computeFM2
        () {
            FastMarching< grid_t, heap_t> fmm;

            fmm.setEnvironment(grid_);
            fmm.setInitialPoints(fmm2_sources_, goal_idx_);
            fmm.computeFM(false);

            float max_value=grid_->getMaxValue();


            for (int i=0; i<grid_->size(); i++) {
                grid_->getCell(i).setVelocity(grid_->getCell(i).getValue()/max_value);
                grid_->getCell(i).setValue(inf_);
                grid_->getCell(i).setState(FMState::OPEN);
            }

            fmm.setEnvironment(grid_);
            start = system_clock::now();
            fmm.setInitialPoints(initial_point_, goal_idx_);
            fmm.computeFM(true);
            end = system_clock::now();
            time_elapsed = duration_cast<milliseconds>(end-start).count();
            std::cout << "\tFM2 second wave time: " << time_elapsed << " ms" << std::endl;
        }

        /**
         * Main Fast Marching Square Function with velocity saturation. It requires to call first the setInitialPoints() function.
         *
         * @param Saturation distance
         *
         * @see setInitialPoints()
         */

        virtual void computeFM2
        (const float maxDistance) {
            FastMarching< grid_t, heap_t> fmm;

            fmm.setEnvironment(grid_);
            fmm.setInitialPoints(fmm2_sources_, goal_idx_);
            fmm.computeFM();

            float maxValue=grid_->getMaxValue();
            double maxVelocity=maxDistance/grid_->getLeafSize(); // Calculate max velocity using the max distance and the leaf size of the cell

            for (int i=0; i<grid_->size(); i++) {

                double vel=grid_->getCell(i).getValue()/maxValue;

                if (vel<maxVelocity)
                    grid_->getCell(i).setVelocity(vel/maxVelocity);
                else
                    grid_->getCell(i).setVelocity(1);

                grid_->getCell(i).setValue(inf_);
                grid_->getCell(i).setState(FMState::OPEN);
            }

            fmm.setEnvironment(grid_);
            fmm.setInitialPoints(initial_point_, goal_idx_);
            start = system_clock::now();
            fmm.setInitialPoints(initial_point_, goal_idx_);
            fmm.computeFM(true);
            end = system_clock::now();
            time_elapsed = duration_cast<milliseconds>(end-start).count();
            std::cout << "\tFM2 second wave time: " << time_elapsed << " ms" << std::endl;
        }

        /**
         * Computes the path from the given index to a minimum (the one
         * gradient descent choses).
         *
         * No checks are done (points in the borders, points in obstacles...).
         *
         * The included scripts will parse the saved path.
         *
         * @param path the resulting path (output).
         */

        virtual void computePath
        (path_t * p) {
            path_ = p;
            constexpr int ndims=grid_->getNDims();

            GradientDescent< nDGridMap<FMCell, ndims> > grad;
            grad.apply(*grid_,goal_idx_,*path_);
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
            path_ = p;
            constexpr int ndims=grid_->getNDims();

            GradientDescent< nDGridMap<FMCell, ndims> > grad;
            grad.apply(*grid_,goal_idx_,*path_, *path_velocity);
        }


    protected:
        grid_t* grid_; /*!< Main container. */
        path_t* path_; /*!< Path container. */

        double inf_=std::numeric_limits<double>::infinity();

        std::vector<int> fmm2_sources_;	/*!< Wave propagation sources for the Fast Marching Square. */
        std::vector<int> initial_point_;	/*!< Initial point for the Fast Marching Square. */
        int goal_idx_; /*!< Goal point for the Fast Marching Square. */

        time_point<std::chrono::system_clock> start, end; // Time measuring.
        double time_elapsed;
};


#endif /* FASTMARCHING2_H_*/
