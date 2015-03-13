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

#include "eikonalsolver.hpp"
#include "fmdata/fmcell.h"

#include "../utils/utils.h"

#include <queue>

/// \todo implement a more robust goal point stopping criterion.
template < class grid_t > class DDQM : public EikonalSolver<grid_t> {

    public:
        //DDQM(unsigned maxSweeps = std::numeric_limits<unsigned>::max()) : EikonalSolver<grid_t>("DDQM", maxSweeps) {}

        //DDQM(const char * name, unsigned maxSweeps = std::numeric_limits<unsigned>::max()) : EikonalSolver<grid_t>(name, maxSweeps) {}

        DDQM(const char * name = "DDQM") : EikonalSolver<grid_t>(name) {}

        /** \brief Calls EikonalSolver::setEnvironment() and sets the initial threshold. */
        virtual void setEnvironment
        (grid_t * g) {
            EikonalSolver<grid_t>::setEnvironment(g);
            double avgSpeed = 2*grid_->getLeafSize(); /// \todo compute it properly.
            threshold_ =1.5 / avgSpeed;
            thStep_ = 2;
        }

        /** \brief Actual method that implements DDQM. */
        virtual void computeInternal
        () {
            if (!setup_)
                setup();

            // FMState::FROZEN - locked and FMState::OPEN - unlocked.
            // The time this takes is negligible and if done in setup or
            // setEnvironment it can affect other planners run in the same
            // grid.
            for(size_t i = 0; i < grid_->size(); ++i)
                grid_->getCell(i).setState(FMState::FROZEN);

            // Initialization
            unsigned int n_neighs = 0;
            for (unsigned int i: init_points_) {
                grid_->getCell(i).setArrivalTime(0);
                n_neighs = grid_->getNeighbors(i, neighbors_);
                for (unsigned int j = 0; j < n_neighs; ++j) {
                    grid_->getCell(neighbors_[j]).setState(FMState::OPEN);
                    queues_[0].push(j);
                }
            }

            // percentages_[0] is in lower queue. [1] is in total.
            std::array<size_t, 2> percents = {0,0};

            // lq is the index of the lower queue (to avoid swapping and copying).
            unsigned int lq = 0;
            while (!queues_[0].empty() || !queues_[1].empty()) {
                while (!queues_[lq].empty()) {
                    unsigned int idx = queues_[lq].front();
                    queues_[lq].pop();
                    double newT = solveEikonal(idx);
                    if (utils::isTimeBetterThan(newT, grid_->getCell(idx).getArrivalTime())) {
                        grid_->getCell(idx).setArrivalTime(newT);
                        n_neighs = grid_->getNeighbors(idx, neighbors_);
                        for (unsigned int j = 0; j < n_neighs; ++j)
                            if (grid_->getCell(j).getState() == FMState::FROZEN) // In the paper they say unlocked here, but makes no sense!!
                                if(utils::isTimeBetterThan(newT, grid_->getCell(j).getArrivalTime())) {
                                    grid_->getCell(j).setState(FMState::OPEN);
                                    percents[1] += 1;
                                    if (utils::isTimeBetterThan(newT, threshold_)) {
                                        queues_[lq].push(j); // Insert in lower queue.
                                        percents[0] += 1;
                                    }
                                    else
                                        queues_[(lq+1)%2].push(j); // Insert in higher queue.
                                }
                    } // If time is improved.
                    grid_->getCell(idx).setState(FMState::FROZEN);
                } // While lower queue is not empty.

                lq = (lq+1)%2;
                increaseThreshold(percents);
            }
        }

        void increaseThreshold
        (std::array<size_t, 2> & percents) {
            double minPercent = 0.65;
            double maxPercent = 0.7;
            double currentPercent = percents[0]/percents[1];
            if (currentPercent < minPercent)
                thStep_ *= 1.5;
            if (currentPercent > maxPercent)
                thStep_ /= 2.0;
            threshold_ += thStep_;
            percents = {0,0};
        }

        virtual void reset
        () {
            EikonalSolver<grid_t>::reset();
        }

        virtual void printRunInfo
        () const {
            console::info("Double Dynamic Queue Method");
            std::cout << '\t' << name_ << '\n'
                      << '\t' << "Elapsed time: " << time_ << " ms\n";
        }

    protected:
        using EikonalSolver<grid_t>::grid_;
        using EikonalSolver<grid_t>::init_points_;
        using EikonalSolver<grid_t>::goal_idx_;
        using EikonalSolver<grid_t>::setup_;
        using EikonalSolver<grid_t>::setup;
        using EikonalSolver<grid_t>::name_;
        using EikonalSolver<grid_t>::time_;
        using EikonalSolver<grid_t>::solveEikonal;

        /** \brief Auxiliar array which stores the neighbor of each iteration of the computeFM() function. */
        std::array <unsigned int, 2*grid_t::getNDims()> neighbors_;

        std::array<std::queue<unsigned int>, 2> queues_;

        double threshold_;
        double thStep_;

};

#endif /* DDQM_HPP_*/
