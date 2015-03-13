/*! \class FMM
    \brief Implements the Fast Marching Method (FMM).

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

/** \brief Heuristic strategy to be used. TIME = DISTANCE/local velocity. */
enum HeurStrategy {NOHEUR = 0, TIME, DISTANCE};

template < class grid_t, class heap_t = FMDaryHeap<FMCell> >  class FMM : public Solver<grid_t> {

    public:
        FMM(HeurStrategy h = NOHEUR) : Solver<grid_t>("FMM"), heurStrategy_(h), precomputed_(false) {
            /// \todo automate the naming depending on the heap.
            //if (static_cast<FMFibHeap>(heap_t))
             //   name_ = "FMMFib";
        }

        FMM(const char * name, HeurStrategy h = NOHEUR) : Solver<grid_t>(name), heurStrategy_(h), precomputed_(false) {}

        virtual ~FMM() { clear(); }

        /** \brief Executes Solver setup and sets maximum size for the narrow band. */
        virtual void setup
        () {
            Solver<grid_t>::setup();
            narrow_band_.setMaxSize(grid_->size());
            setHeuristics(heurStrategy_); // Redundant, but safe.

            if (int(goal_idx_) == -1 && heurStrategy_ != NOHEUR) {
                console::warning("FMM/SFMM: Heuristics set with no goal point. Deactivating heuristics.");
                heurStrategy_ = NOHEUR;
            }
        }

        /** \brief Actual method that implements FMM. */
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
                n_neighs = grid_->getNeighbors(idxMin, neighbors_);
                grid_->getCell(idxMin).setState(FMState::FROZEN);
                for (unsigned int s = 0; s < n_neighs; ++s) {
                    j = neighbors_[s];
                    if ((grid_->getCell(j).getState() == FMState::FROZEN) || grid_->getCell(j).isOccupied())
                        continue;
                    else {
                        double new_arrival_time = solveEikonal(j);
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
                        } // neighbors_ open.
                    } // neighbors_ not frozen.
                } // For each neighbor.
                if (idxMin == goal_idx_)
                    stopWavePropagation = true;
            } // while narrow band not empty
        }

        /** \brief Solves nD Eikonal equation for cell idx. If heuristics are activated, it will add
            the estimated travel time to goal with current velocity. */
        virtual double solveEikonal
        (const int & idx) {
            unsigned int a = grid_t::getNDims(); // a parameter of the Eikonal equation.
            Tvalues_.clear();

            for (unsigned int dim = 0; dim < grid_t::getNDims(); ++dim) {
                double minTInDim = grid_->getMinValueInDim(idx, dim);
                if (!isinf(minTInDim) && minTInDim < grid_->getCell(idx).getArrivalTime())
                    Tvalues_.push_back(minTInDim);
                else
                    a -=1;
            }

            if (a == 0)
                return std::numeric_limits<double>::infinity();

            // Sort the neighbor values to make easy the following code.
            /// \todo given that this sorts a small vector, a n^2 methods could be better. Test it.
            std::sort(Tvalues_.begin(), Tvalues_.end());
            double updatedT;
            for (unsigned i = 1; i <= a; ++i) {
                updatedT = solveEikonalNDims(idx, i);
                // If no more dimensions or increasing one dimension will not improve time.
                if (i == a || (updatedT - Tvalues_[i]) < utils::COMP_MARGIN)
                    break;
            }

            // Include heuristics if necessary.
            if (heurStrategy_ == TIME)
                grid_->getCell(idx).setHeuristicTime( getPrecomputedDistance(idx)/grid_->getCell(idx).getVelocity() );
            else if (heurStrategy_ == DISTANCE)
                grid_->getCell(idx).setHeuristicTime( getPrecomputedDistance(idx) );

            return updatedT;
        }


        /** \brief Set heuristics flag. True is activated. It will precompute distances
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

        /** \brief Computes euclidean distance between goal and rest of cells. */
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

        /** \brief Extracts the euclidean distance calculated from precomputeDistances
            function distance between two positions. */
        virtual double getPrecomputedDistance
        (const unsigned int idx) {
            std::array <unsigned int, grid_t::getNDims()> position, distance;
            grid_->idx2coord(idx, position);

            for (unsigned int i = 0; i < grid_t::getNDims(); ++i)
                distance[i] = utils::absUI(position[i] - heur_coord_[i]);

            unsigned int idx_dist;
            grid_->coord2idx(distance, idx_dist);

            return distances_[idx_dist];
        }

        virtual void printRunInfo
        () const {
            console::info("Fast Marching Method");
            std::cout << '\t' << name_ << '\n'
                      << '\t' << "Heuristic type: " << heurStrategy_ << '\n'
                      << '\t' << "Elapsed time: " << time_ << " ms\n";
        }


    /// \note These accessing levels may need to be modified (and other solvers).
    protected:
        /** \brief Solves the Eikonal equation assuming that Tvalues_
            is sorted. */
        double solveEikonalNDims
        (unsigned int idx, unsigned int dim) {
            // Solve for 1 dimension.
            if (dim == 1)
                return Tvalues_[0] + grid_->getLeafSize() / grid_->getCell(idx).getVelocity();

            // Solve for any number > 1 of dimensions.
            double sumT = 0;
            double sumTT = 0;
            for (unsigned i = 0; i < dim; ++i) {
                sumT += Tvalues_[i];
                sumTT += Tvalues_[i]*Tvalues_[i];
            }
            double a = dim;
            double b = -2*sumT;
            double c = sumTT - grid_->getLeafSize() * grid_->getLeafSize() / (grid_->getCell(idx).getVelocity()*grid_->getCell(idx).getVelocity());
            double quad_term = b*b - 4*a*c;

            if (quad_term < 0)
                return std::numeric_limits<double>::infinity();
            else
                return (-b + sqrt(quad_term))/(2*a);
        }

        using Solver<grid_t>::grid_;
        using Solver<grid_t>::init_points_;
        using Solver<grid_t>::goal_idx_;
        using Solver<grid_t>::setup_;
        using Solver<grid_t>::name_;
        using Solver<grid_t>::time_;

        /** \brief Auxiliar array which stores the neighbor of each iteration of the computeFM() function. */
        std::array <unsigned int, 2*grid_t::getNDims()> neighbors_;

        /** \brief Auxiliar vector with values T0,T1...Tn-1 variables in the Discretized Eikonal Equation. */
        std::vector<double>          Tvalues_;

    private:
        /** \brief Instance of the heap used. */
        heap_t                                          narrow_band_;

        /** \brief Flag to activate heuristics and corresponding strategy. */
        HeurStrategy                                    heurStrategy_;

        /** \brief Stores the precomputed heuristic distances. */
        std::vector<double>                             distances_;
        
        /** \brief Flag to indicate if distances_ is already computed. */
        bool                                            precomputed_;

        /** \brief Goal coord, goal of the second wave propagation (actually the initial point of the path). */
        std::array <unsigned int, grid_t::getNDims()>   heur_coord_;
};

#endif /* FMM_HPP_*/
