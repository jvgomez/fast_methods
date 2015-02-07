/*! \file UFMM.hpp
    \brief Fast Marching Method using a untidy priority queue (UFMM).
    
    It uses as a main container the nDGridMap class. The nDGridMap type T
    has to be an FMUntidyCell or something inherited from it.
    
    The leafsize of the grid map is ignored since it has to be >=1 and that 
    depends on the units employed.
    
    @par External documentation:
        L. Yatziv, A.Bartesaghi and G. Sapiro, O(n) implementation of the fast marching algorithm, Journal of Computational Physics 
        <a href="http://www.sciencedirect.com/science/article/pii/S0021999105003736">[PDF]</a>
    
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

#ifndef UFMM_HPP_
#define UFMM_HPP_

#include "fmm.hpp"
#include "fmdata/fmuntidyqueue.hpp"

template <class grid_t, class cell_t = FMCell> class UFMM : public FMM <grid_t> {

    public:
        UFMM
        (unsigned s = 1000, double inc = 2) :  FMM<grid_t>("UFMM"), heap_s_(s), heap_inc_(inc) {
            narrow_band_ = new FMUntidyQueue<cell_t> (heap_s_, heap_inc_);
        }
        UFMM
        (const std::string& name, unsigned s = 1000, double inc = 2) : FMM<grid_t>(name), heap_s_(s), heap_inc_(inc) {
            narrow_band_ = new FMUntidyQueue<cell_t> (heap_s_, heap_inc_);
        }

        /**
         * Main Untidy Fast Marching Function. It requires to call first the setInitialPoints() function inherited from Fast Marching.
         *
         * @see setInitialPoints()
         */
        virtual void compute
        () {
            if (!setup_)
                setup();

            unsigned int j= 0;
            unsigned int n_neighs = 0;
            bool stopWavePropagation = false;

            // Algorithm initialization
            for (unsigned int &i : init_points_) { // For each initial point
                grid_->getCell(i).setArrivalTime(0);
                grid_->getCell(i).setState(FMState::FROZEN);
                narrow_band_->push( &(grid_->getCell(i)) );
            }

            // Main loop
            while (!stopWavePropagation && !narrow_band_->empty()) {
                unsigned int idxMin = narrow_band_->popMinIdx();
                n_neighs = grid_->getNeighbors(idxMin, neighbors);
                grid_->getCell(idxMin).setState(FMState::FROZEN);
                for (unsigned int s = 0; s < n_neighs; ++s) { // For each neighbor
                    j = neighbors[s];
                    if ( (grid_->getCell(j).getState() == FMState::FROZEN) || grid_->getCell(j).isOccupied()|| (grid_->getCell(j).getVelocity() == 0) ) // If Frozen,obstacle or velocity = 0
                        continue;
                    else {
                        double new_arrival_time = solveEikonal(j);
                        if (grid_->getCell(j).getState() == FMState::NARROW) { // Updating narrow band if necessary.
                            if (new_arrival_time < grid_->getCell(j).getArrivalTime() ) {
                                grid_->getCell(j).setArrivalTime(new_arrival_time);
                                narrow_band_->increase( &(grid_->getCell(j)) );
                            }
                        }
                        else {
                            grid_->getCell(j).setState(FMState::NARROW);
                            grid_->getCell(j).setArrivalTime(new_arrival_time);
                            narrow_band_->push( &(grid_->getCell(j)) );
                        } // neighbors open.
                    } // neighbors not frozen.
                } // For each neighbor.
                if (idxMin == goal_idx_)
                    stopWavePropagation = true;
            } // while narrow band is not empty
        }

        virtual void clear
        () {
            FMM<grid_t>::clear();
            narrow_band_->clear();
            delete narrow_band_;
        }

        virtual void reset
        () {
            FMM<grid_t>::reset();
            narrow_band_->clear();
        }

    protected:
        using FMM<grid_t>::grid_;
        using FMM<grid_t>::neighbors;
        using FMM<grid_t>::solveEikonal;
        using FMM<grid_t>::init_points_;
        using FMM<grid_t>::goal_idx_;
        using Solver<grid_t>::setup;
        using FMM<grid_t>::setup_;

    private:
        unsigned heap_s_;
        double heap_inc_;
        FMUntidyQueue<cell_t> * narrow_band_; /*!< Instance of the priority queue used. */
};

#endif /* UFMM_HPP_*/
