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

#include "../ndgridmap/ndgridmap.hpp"
#include "../console/console.h"

unsigned int absUI
(int a) {
    return (a>0) ? (a) : (-a);
}

enum HeurStrategy {NOHEUR = 0, TIME, DISTANCE};

template < class grid_t, class heap_t = FMDaryHeap<FMCell> >  class FMM : public Solver<grid_t> {

    public:
        FMM(HeurStrategy h = NOHEUR) : Solver<grid_t>("FMM"), heurStrategy_(h), precomputed_(false) {
            // TODO: try to automate this.
            //if (static_cast<FMFibHeap>(heap_t))
             //   name_ = "FMMFib";
        }

        FMM(const char * name, HeurStrategy h = NOHEUR) : Solver<grid_t>(name), heurStrategy_(h), precomputed_(false) {}

        virtual ~FMM() { clear(); }

        /** Executes Solver setup and sets maximum size for the narrow band. */
        virtual void setup
        () {
            Solver<grid_t>::setup();
            narrow_band_.setMaxSize(grid_->size());
            setHeuristics(heurStrategy_); // Redundant, but safe.
        }

        /** \brief Solves nD Eikonal equation for cell idx. If heuristics are activated, it will add
            the estimated travel time to goal with current velocity. */
        virtual double solveEikonal
        (const int & idx) {
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
            double c = sumTT - grid_->getLeafSize() * grid_->getLeafSize()/(grid_->getCell(idx).getVelocity()*grid_->getCell(idx).getVelocity());
            double quad_term = b*b - 4*a*c;
            if (quad_term < 0) {
                double minT = *(std::min_element(Tvalues.begin(), Tvalues.end()));
                updatedT = grid_->getLeafSize() * grid_->getLeafSize()/(grid_->getCell(idx).getVelocity()*grid_->getCell(idx).getVelocity()) + minT;
            }
            else
                updatedT = (-b + sqrt(quad_term))/(2*a);

            if (heurStrategy_ == TIME)
                grid_->getCell(idx).setHeuristicTime( getPrecomputedDistance(idx)/grid_->getCell(idx).getVelocity() );
            else if (heurStrategy_ == DISTANCE)
                grid_->getCell(idx).setHeuristicTime( getPrecomputedDistance(idx) );

            return updatedT;
        }

        virtual void computeInternal
        () {
            if (!setup_)
                setup();

            unsigned int j = 0;
            unsigned int n_neighs = 0;
            bool stopWavePropagation = false;

            // Algorithm initialization
            for (unsigned int &i: init_points_) { // For each initial point
                grid_->getCell(i).setArrivalTime(0);
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

        /** Set heuristics flag. True is activated. It will precompute distances
            if not done already. */
        void setHeuristics
        (HeurStrategy h) {
            if (h && int(goal_idx_)!=-1) {
                heurStrategy_ = h;
                grid_->idx2coord(goal_idx_, heur_coord_);
                if (!precomputed_)
                    precomputeDistances();
            }
        }

        /** \brief Returns heuristics flag. */
        HeurStrategy getHeuristics
        () const {
            return heurStrategy_;
        }

        virtual void clear
        () {
            narrow_band_.clear();
            distances_.clear();
            precomputed_ = false;
        }

        virtual void reset
        () {
            Solver<grid_t>::reset();
            narrow_band_.clear();
        }

        /** Computes euclidean distance between goal and rest of cells. */
        virtual void precomputeDistances
        () {
            distances_.reserve(grid_->size());
            std::array <unsigned int, grid_t::getNDims()> coords;
            double dist = 0;

            for (size_t i = 0; i < grid_->size(); ++i)
            {
                dist = 0;
                grid_->idx2coord(i, coords);

                for (size_t j = 0; j < coords.size(); ++j)
                    dist += coords[j]*coords[j];

                distances_[i] = std::sqrt(dist);
            }
            precomputed_ = true;
        }

        /** Extract the euclidean distance calculated from precomputeDistances
            function distance between two positions. */
        virtual double getPrecomputedDistance
        (const unsigned int idx) {
            std::array <unsigned int, grid_t::getNDims()> position, distance;
            grid_->idx2coord(idx, position);

            for (unsigned int i = 0; i < grid_t::getNDims(); ++i)
                distance[i] = absUI(position[i] - heur_coord_[i]);

            unsigned int idx_dist;
            grid_->coord2idx(distance, idx_dist);

            return distances_[idx_dist];
        }

    protected:
        using Solver<grid_t>::grid_;
        using Solver<grid_t>::init_points_;
        using Solver<grid_t>::goal_idx_;
        using Solver<grid_t>::setup_;
        using Solver<grid_t>::name_;

        std::array <unsigned int, 2*grid_t::getNDims()> neighbors; /*!< Auxiliar array which stores the neighbor of each iteration of the computeFM() function. */

    private:
        double sumT; /*!< Auxiliar value wich computes T1+T2+T3... Useful for generalizing the Eikonal solver. */
        double sumTT; /*!< Auxiliar value wich computes T1^2+T2^2+T3^2... Useful for generalizing the Eikonal solver. */

        std::array<double, grid_t::getNDims()> Tvalues;  /*!< Auxiliar array with values T0,T1...Tn-1 variables in the Discretized Eikonal Equation. */
        std::array<double, grid_t::getNDims()> TTvalues;  /*!< Auxiliar array with values T0^2,T1^2...Tn-1^2 variables in the Discretized Eikonal Equation. */

        heap_t narrow_band_; /*!< Instance of the heap used. */

        HeurStrategy heurStrategy_;/*!< Flag to activate heuristics and corresponding strategy. */
        std::vector<double> distances_;  /*!< Stores the precomputed heuristic distances. */
        bool precomputed_;  /*!< Flag to indicate if distances_ is already computed. */
        std::array <unsigned int, grid_t::getNDims()> heur_coord_; /*!< Goal coord, goal of the second wave propagation (actually the initial point of the path). */
};

#endif /* FMM_HPP_*/
