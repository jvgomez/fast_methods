/*! \class LSM
    \brief Implements Lock Sweeping Method.

    It uses as a main container the nDGridMap class. The nDGridMap type T
    has to use an FMCell or derived.

    The grid is assumed to be squared, that is Delta(x) = Delta(y) = leafsize_

    @par External documentation:
        H. Zhao, A fast sweeping method for Eikonal equations, Math. Comp. 74 (2005), 603-627.
        <a href="http://www.ams.org/journals/mcom/2005-74-250/S0025-5718-04-01678-3/S0025-5718-04-01678-3.pdf">[PDF]</a>

    NOTE: The sweeping directions are inverted with respect to the paper to make implementation easier. And sweeping
    is implemented recursively (undetermined number of nested for loops) to achieve n-dimensional behaviour.

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

#ifndef LSM_HPP_
#define LSM_HPP_

#include "fsm.hpp"
#include "../utils/utils.h"

#include <algorithm>

/// \todo implement a more robust goal point stopping criterion.
template < class grid_t > class LSM : public FSM<grid_t> {

    public:
        LSM(unsigned maxSweeps = std::numeric_limits<unsigned>::max()) : FSM<grid_t>("LSM", maxSweeps) {}

        LSM(const char * name, unsigned maxSweeps = std::numeric_limits<unsigned>::max()) : FSM<grid_t>(name, maxSweeps) {}

        /** \brief Actual method that implements LSM. */
        virtual void computeInternal
        () {
            if (!setup_)
                setup();

            // Initialization
            for (unsigned int i: init_points_) // For each initial point
                grid_->getCell(i).setArrivalTime(0);

            // Getting dimsizes and filling the other dimensions.
            keepSweeping_ = true;
            stopPropagation_ = false;

            while (keepSweeping_ && !stopPropagation_ && sweeps_ < maxSweeps_) {
                keepSweeping_ = false;
                setSweep();
                ++sweeps_;

                recursiveIteration(grid_t::getNDims()-1);
            }
        }

        virtual void reset
        () {
            FSM<grid_t>::reset();
            sweeps_ = 0;
        }

        virtual void printRunInfo
        () const {
            console::info("Lock Sweeping Method");
            std::cout << '\t' << name_ << '\n'
                      << '\t' << "Maximum sweeps: " << maxSweeps_ << '\n'
                      << '\t' << "Sweeps performed: " << sweeps_ << '\n'
                      << '\t' << "Elapsed time: " << time_ << " ms\n";
        }

    protected:
        void recursiveIteration
        (size_t depth, int it = 0) {
            if (depth > 0) {
                for(int i = inits_[depth]; i != ends_[depth]; i += incs_[depth])
                    recursiveIteration(depth-1, it + i*d_[depth-1]);
            }
            else {
                for(int i = inits_[0]; i != ends_[0]; i += incs_[0]) {
                    unsigned idx = it + i;
                    const double prevTime = grid_->getCell(idx).getArrivalTime();
                    const double newTime = solveEikonal(idx);
                    if(utils::isTimeBetterThan(newTime, prevTime)) {
                        grid_->getCell(idx).setArrivalTime(newTime);
                        keepSweeping_ = true;
                    }
                    // EXPERIMENTAL - Value not updated, it has converged
                    else if(!isnan(newTime) && !isinf(newTime) && (idx == goal_idx_))
                        stopPropagation_ = true;

                }
            }
        }

        // Inherited members from FSM.
        using FSM<grid_t>::grid_;
        using FSM<grid_t>::init_points_;
        using FSM<grid_t>::goal_idx_;
        using FSM<grid_t>::setup_;
        using FSM<grid_t>::setup;
        using FSM<grid_t>::name_;
        using FSM<grid_t>::time_;
        using FSM<grid_t>::solveEikonal;
        using FSM<grid_t>::setSweep;
        using FSM<grid_t>::sweeps_;
        using FSM<grid_t>::maxSweeps_;
        using FSM<grid_t>::keepSweeping_;
        using FSM<grid_t>::stopPropagation_;
        using FSM<grid_t>::incs_;
        using FSM<grid_t>::inits_;
        using FSM<grid_t>::ends_;
        using FSM<grid_t>::d_;
};

#endif /* LSM_HPP_*/
