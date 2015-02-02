/*! \file FMM.hpp
    \brief Templated class which computes the basic Fast Marching Method (FMM).

    It uses as a main container the nDGridMap class. The nDGridMap type T
    has to be an FMCell or something inherited from it.

    The grid is assumed to be squared, that is Delta(x) = Delta(y) = leafsize_

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

#ifndef FMM_HPP_
#define FMM_HPP_

#include <iostream>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <fstream>
#include <array>

#include "solver.hpp"

#include "fmdata/fmcell.h"
#include "fmdata/fmdaryheap.hpp"
#include "fmdata/fmfibheap.hpp"
#include "../ndgridmap/ndgridmap.hpp"
#include "../console/console.h"

// TODO: check initial and goal points are not the same, not on obstacles, etc.
// IMPORTANT TODO: substitute grid_->getCell(j).isOccupied() by grid_->getCell(j).getVelocity() == 0 (conceptually is not the same).

template < class grid_t, class heap_t = FMDaryHeap<FMCell> >  class FMM : public Solver<grid_t> {

    public:
        FMM(const std::string& name = "FMMDary") : Solver<grid_t>(name) {
            // TODO: try to automate this.
            //if (static_cast<FMFibHeap>(heap_t))
             //   name_ = "FMMFib";
        }

        /**
        * Internal function although it is set to public so it can be accessed if desired.
        *
        * Computes the Fast Marching Method initialization from the initial points given. Programmed following the paper:
          A. Valero, J.V. Gómez, S. Garrido and L. Moreno, The Path to Efficiency: Fast Marching Method for Safer,
          More Efficient Mobile Robot Trajectories, IEEE Robotics and Automation Magazine, Vol. 20, No. 4, 2013.
        *
        * @see setInitialAndGoalPoints()
        */
        virtual void setup
        () {
            Solver<grid_t>::setup();
            narrow_band_.setMaxSize(grid_->size());
        }

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
            // TODO: neighbors computed twice for every cell. We can save time here.
            unsigned int a = grid_t::getNDims(); // a parameter of the Eikonal equation.

            double updatedT;
            sumT = 0;
            sumTT = 0;

            for (unsigned int dim = 0; dim < grid_t::getNDims(); ++dim) {
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
            double c = sumTT - grid_->getLeafSize() * grid_->getLeafSize()/(grid_->getCell(idx).getVelocity()*grid_->getCell(idx).getVelocity()); // leafsize not taken into account here.
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
         * Main Fast Marching Function. It requires to call first the setInitialAndGoalPoints() function.
         * If a goal_idx_ was set through setInitialAndGoalPoints(), the wave will stop expanding once
         * that cell is reached.
         *
         * @see setInitialAndGoalPoints()
         */
        virtual void compute
        () {
            if (!setup_)
                setup();

            unsigned int j = 0;
            unsigned int n_neighs = 0;
            bool stopWavePropagation = 0;

            // Algorithm initialization
            for (unsigned int &i: init_points_) { // For each initial point
                grid_->getCell(i).setArrivalTime(0);
                grid_->getCell(i).setState(FMState::FROZEN);
                narrow_band_.push( &(grid_->getCell(i)) );
            }

            // Main loop.
            while (!stopWavePropagation && !narrow_band_.empty()) {
                unsigned int idxMin = narrow_band_.popMinIdx();
                n_neighs = grid_->getNeighbors(idxMin, neighbors);
                grid_->getCell(idxMin).setState(FMState::FROZEN);

                for (unsigned int s = 0; s < n_neighs; ++s) {
                    j = neighbors[s];
                    // If Frozen or obstacle
                    if ((grid_->getCell(j).getState() == FMState::FROZEN) || grid_->getCell(j).isOccupied())
                        continue;
                    else {
                        double new_arrival_time = solveEikonal(j);
                        // Updating narrow band if necessary.
                        if (grid_->getCell(j).getState() == FMState::NARROW) {
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
                    if (idxMin == goal_idx_)
                        stopWavePropagation = true;
                } // For each neighbor.
            } // while narrow band not empty
        }

        virtual void clear
        () {
            Solver<grid_t>::clear();
            narrow_band_.clear();
        }

        virtual void reset
        () {
            Solver<grid_t>::reset();
            narrow_band_.clear();
        }

    protected:
        using Solver<grid_t>::grid_;
        using Solver<grid_t>::init_points_;
        using Solver<grid_t>::goal_idx_;
        using Solver<grid_t>::setup_;

        std::array <unsigned int, 2*grid_t::getNDims()> neighbors;  /*!< Auxiliar array which stores the neighbor of each iteration of the computeFM() function. */

    private:
        double sumT; /*!< Auxiliar value wich computes T1+T2+T3... Useful for generalizing the Eikonal solver. */
        double sumTT; /*!< Auxiliar value wich computes T1^2+T2^2+T3^2... Useful for generalizing the Eikonal solver. */

        std::array<double, grid_t::getNDims()> Tvalues;  /*!< Auxiliar array with values T0,T1...Tn-1 variables in the Discretized Eikonal Equation. */
        std::array<double, grid_t::getNDims()> TTvalues;  /*!< Auxiliar array with values T0^2,T1^2...Tn-1^2 variables in the Discretized Eikonal Equation. */

        heap_t narrow_band_; /*!< Instance of the heap used. */
};

#endif /* FMM_HPP_*/
