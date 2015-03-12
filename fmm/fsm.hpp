/*! \class FSM
    \brief Implements Fast Sweeping Method up to 3D.

    IMPORTANT NOTE: A maximum number of dimensions is set because of the sweeps implementation
    which require nested loops. The alternative is to do it recursively, but that would make the
    algorithm slower and FSM is rarely used above 3D.

    The FSM code contains information about how to set the maximum number of dimensions. Look for
    "// Dimension:" comments.

    It uses as a main container the nDGridMap class. The nDGridMap type T
    has to use an FMCell or derived.

    The grid is assumed to be squared, that is Delta(x) = Delta(y) = leafsize_

    @par External documentation:
        H. Zhao, A fast sweeping method for Eikonal equations, Math. Comp. 74 (2005), 603-627.
        <a href="http://www.ams.org/journals/mcom/2005-74-250/S0025-5718-04-01678-3/S0025-5718-04-01678-3.pdf">[PDF]</a>

    NOTE: The sweeping directions are inverted with respect to the paper to make implementation easier.

    Copyright (C) 2015 Javier V. Gomez
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

#ifndef FSM_HPP_
#define FSM_HPP_

#include "fmm.hpp"
#include "../utils/utils.h"

#include <algorithm>

// Dimension: change this value to the number of dimensions required.
#define MAXDIMS 3

/// \todo implement a more robust goal point stopping criterion.
template < class grid_t > class FSM : public FMM<grid_t> {

    public:
        FSM(unsigned maxSweeps = std::numeric_limits<unsigned>::max()) : FMM<grid_t>("FSM"),
            sweeps_(0),
            maxSweeps_(maxSweeps) {
            initializeSweepArrays();
            Tvalues_.reserve(grid_t::getNDims());
        }

        FSM(const char * name, unsigned maxSweeps = std::numeric_limits<unsigned>::max()) : FMM<grid_t>(name),
            sweeps_(0),
            maxSweeps_(maxSweeps) {
            initializeSweepArrays();
            Tvalues_.reserve(grid_t::getNDims());
        }

        /** \brief Sets and cleans the grid in which operations will be performed.
             Since a maximum number of dimensions is assumed, fills the rest with size 1. */
        virtual void setEnvironment
        (grid_t * g) {
            FMM<grid_t>::setEnvironment(g);
            // Filling the size of the dimensions...
            std::array<unsigned, grid_t::getNDims()> dimsize = g->getDimSizes();
            for (size_t i = 0; i < grid_t::getNDims(); ++i)
                dimsize_[i] = dimsize[i];
            // ... and the extended dimensions.
            for (size_t i = grid_t::getNDims(); i < MAXDIMS; ++i)
                dimsize_[i] = 1;
        }

        /** \brief Executes Solver setup (instead of FMM setup) and other checks. */
        virtual void setup
        () {
            Solver<grid_t>::setup();

            if (int(goal_idx_) != -1) {
                console::warning("Setting a goal point in FSM is experimental. It may lead to wrong results.");
            }
        }

        /** \brief Actual method that implements FSM. */
        virtual void computeInternal
        () {
            if (!setup_)
                setup();

            // Initialization
            for (unsigned int i: init_points_) // For each initial point
                grid_->getCell(i).setArrivalTime(0);

            // Getting dimsizes and filling the other dimensions.
            bool keepSweeping = true;
            bool stopPropagation = false;
            unsigned idx = 0;

            while (keepSweeping && !stopPropagation && sweeps_ < maxSweeps_) {
                keepSweeping = false;
                setSweep();
                ++sweeps_;

                // Dimension: nest as many for loops as MAXDIMS.
                for (int k = inits_[2]; k != ends_[2]; k+=incs_[2])
                    for (int j = inits_[1]; j != ends_[1]; j+=incs_[1])
                        for (int i = inits_[0]; i != ends_[0]; i+=incs_[0])
                        {
                            // Dimension: update the index computation (for 4D it is
                            // l*dimsize_[0]*dimsize_[1]*dimsize_[2] + k*...
                            // and so on.
                            idx = k*dimsize_[0]*dimsize_[1] + j *dimsize_[0] + i;
                            const double prevTime = grid_->getCell(idx).getArrivalTime();
                            const double newTime = solveEikonal(idx);
                            if(utils::isTimeBetterThan(newTime, prevTime)) {
                                grid_->getCell(idx).setArrivalTime(newTime);
                                keepSweeping = true;
                                if (idx == 3242)
                                    std::cout << "Updated: " << idx << "  " << newTime << '\n';
                            }
                            // EXPERIMENTAL - Value not updated, it has converged
                            else if(!isnan(newTime) && !isinf(newTime) && (idx == goal_idx_)) {
                                stopPropagation = true;
                            }
                        }
            }
        }

        /** \brief FSM-specific solver. Solves recursively nD Eikonal equation for cell idx for
            1D, 2D, etc.. until increasing dimensions does not imrpove the time.
            If the neighbor value is higher than the current value is treated as
            an infinite. */
        virtual double solveEikonal
        (const int & idx) {
            unsigned int a = grid_t::getNDims(); // a parameter of the Eikonal equation.
            Tvalues_.clear();
            double updatedT;

            if (idx == 3242)
                std::cout << idx << "  " << grid_->getCell(idx).getVelocity() << "  ";

            // Getting neigbhor values.
            for (unsigned int dim = 0; dim < grid_t::getNDims(); ++dim) {
                double minTInDim = grid_->getMinValueInDim(idx, dim);
                if (idx == 3242)
                    std::cout << minTInDim << "  ";
                if (!isinf(minTInDim) && minTInDim < grid_->getCell(idx).getArrivalTime())
                    Tvalues_.push_back(minTInDim);
                else
                    a -=1;
            }

            if (idx == 3242)
                std::cout << '\n';

            // FSM will try to solve for indices with no valid neighbor values.
            if (a == 0)
                return std::numeric_limits<double>::infinity();

            // Sort the neighbor values to make easy the following code.
            std::sort(Tvalues_.begin(), Tvalues_.end());

            // Solve from 1dim to n-dim until incrementing 1 more dimension
            // cannot improve the time.
            for (unsigned i = 1; i <= a; ++i) {
                updatedT = solveForNDims(idx, i);
                if (i == a)
                    break;
                else if ((updatedT - Tvalues_[i]) < utils::COMP_MARGIN)
                    break;
            }
            return updatedT;
        }

        virtual void reset
        () {
            FMM<grid_t>::reset();
            sweeps_ = 0;
            initializeSweepArrays();
        }

        virtual void printRunInfo
        () const {
            console::info("Fast Sweeping Method");
            std::cout << '\t' << name_ << '\n'
                      << '\t' << "Maximum sweeps: " << maxSweeps_ << '\n'
                      << '\t' << "Sweeps performed: " << sweeps_ << '\n'
                      << '\t' << "Elapsed time: " << time_ << " ms\n";
        }

    protected:
        /** \brief Set the sweep variables: initial and final indices for iterations,
             and the increment of each iteration in every dimension.

             Generates a periodical pattern for incs_ (example for 3D):
            [-1-1-1, 1-1-1, -11-1, 11-1, -1-11, 1-11, -111, 111]

             Stablishes inits_ and ends_ accordingly. */
        virtual void setSweep
        () {
            // Inspired in http://stackoverflow.com/a/17758788/2283531
            for (size_t i = 0; i < grid_t::getNDims(); ++i)
            {
                if((incs_[i] += 2) <=1)
                    break;
                else
                    incs_[i] = -1;
            }

            // Setting inits and ends.
            for (size_t i = 0; i < grid_t::getNDims(); ++i)
            {
                if (incs_[i] == 1)
                {
                    inits_[i] = 0;
                    ends_[i] = dimsize_[i];
                }
                else
                {
                    inits_[i] = dimsize_[i]-1;
                    ends_[i] = -1;
                }
            }
        }

        /** \brief Initializes the internal arrays employed. */
        virtual void initializeSweepArrays
        () {
            for (size_t i = 0; i < MAXDIMS; ++i) {
                incs_[i] = 1;
                inits_[i] = 0;
                ends_[i] = 1;
            }
        }

        using FMM<grid_t>::grid_;
        using FMM<grid_t>::init_points_;
        using FMM<grid_t>::goal_idx_;
        using FMM<grid_t>::setup_;
        using FMM<grid_t>::name_;
        using FMM<grid_t>::time_;
        using FMM<grid_t>::solveForNDims;
        using FMM<grid_t>::Tvalues_;

        /** \brief Number of sweeps performed. */
        unsigned int sweeps_;

        /** \brief Number of maximum sweeps to perform. */
        unsigned maxSweeps_;

    private:
        /** \brief Sweep directions {-1,1} for each dimension. Extended dimensions always 1. */
        std::array<int, MAXDIMS> incs_;

        /** \brief Initial indices for each dimension. Extended dimensions always 0. */
        std::array<int, MAXDIMS> inits_;

        /** \brief Final indices for each dimension. Extended dimensions always 1. */
        std::array<int, MAXDIMS> ends_;

        /** \brief Size of each dimension, extended to the maximum size. Extended dimensions always 1. */
        std::array<int, MAXDIMS> dimsize_;
};

#endif /* FSM_HPP_*/
