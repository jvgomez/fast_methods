/*! \class DDQM
    \brief Implements Double Dynamic Queue Method.

    It uses as a main container the nDGridMap class. The nDGridMap type T
    has to use an FMCell or derived.

    The grid is assumed to be squared, that is Delta(x) = Delta(y) = leafsize_

    @par External documentation:
        S. Bak, J. McLaughlin, D. Renzi, Some Improvements for the Fast Sweeping Method,
        SIAM J. Sci. Comput., 32(5), 2853–2874.
        <a href="http://epubs.siam.org/doi/abs/10.1137/090749645">[More Info]</a>

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

#ifndef DDQM_HPP_
#define DDQM_HPP_

#include "fmm.hpp"
#include "../utils/utils.h"

/// \todo implement a more robust goal point stopping criterion.
template < class grid_t > class DDQM : public FMM<grid_t> {

    public:
        //DDQM(unsigned maxSweeps = std::numeric_limits<unsigned>::max()) : FMM<grid_t>("DDQM", maxSweeps) {}

        //DDQM(const char * name, unsigned maxSweeps = std::numeric_limits<unsigned>::max()) : FMM<grid_t>(name, maxSweeps) {}

        DDQM(const char * name = "DDQM") : FMM<grid_t>(name) {}

        /** \brief Calls FMM::setEnvironment() and --------. */
        virtual void setEnvironment
        (grid_t * g) {
            FMM<grid_t>::setEnvironment(g);
            // FMState::FROZEN - locked and FMState::OPEN - unlocked.
            /*for(size_t i = 0; i < grid_->size(); ++i)
                grid_->getCell(i).setState(FMState::FROZEN);*/
        }

        /** \brief Actual method that implements DDQM. */
        virtual void computeInternal
        () {
            if (!setup_)
                setup();

            // Initialization
            for (unsigned int i: init_points_) {
                grid_->getCell(i).setArrivalTime(0);
                unsigned int n_neighs = grid_->getNeighbors(i, neighbors_);
                for (unsigned int j = 0; j < n_neighs; ++j)
                    grid_->getCell(neighbors_[j]).setState(FMState::OPEN);
            }
        }

        virtual void reset
        () {
            Solver<grid_t>::reset();
        }

        virtual void printRunInfo
        () const {
            console::info("Double Dynamic Queue Method");
            std::cout << '\t' << name_ << '\n'
                      << '\t' << "Elapsed time: " << time_ << " ms\n";
        }

    protected:
        using FMM<grid_t>::grid_;
        using FMM<grid_t>::init_points_;
        using FMM<grid_t>::goal_idx_;
        using FMM<grid_t>::setup_;
        using FMM<grid_t>::setup;
        using FMM<grid_t>::setEnvironment;
        using FMM<grid_t>::name_;
        using FMM<grid_t>::time_;
        using FMM<grid_t>::solveEikonal;
        //using FSM<grid_t>::d_;
        using FMM<grid_t>::neighbors_;
};

#endif /* DDQM_HPP_*/
