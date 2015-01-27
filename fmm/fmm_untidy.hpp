/*! \file fmm_untidy.hpp
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

#ifndef FMM_UNTIDY_H_
#define FMM_UNTIDY_H_

#include "fastmarching.hpp"
#include "fmdata/fmuntidyqueue.hpp"

template < class grid_t > class FMM_Untidy : public FastMarching <grid_t> {

    using FastMarching<grid_t>::grid_;
    using FastMarching<grid_t>::neighbors;
    using FastMarching<grid_t>::solveEikonal;
    using FastMarching<grid_t>::init_points_;
    //using FastMarching<grid_t>::leafsize_;
    using FastMarching<grid_t>::Tvalues;
    using FastMarching<grid_t>::TTvalues;
    using FastMarching<grid_t>::sumT;
    using FastMarching<grid_t>::sumTT;

    public:
        FMM_Untidy <grid_t> () {
            Solver<grid_t>::Solver("UFMM");
        }
        virtual ~FMM_Untidy <grid_t>() {}

          /**
          * Sets the input grid in which operations will be performed.
          *
          * @param g input grid map.
          */
        virtual void setEnvironment
        (grid_t * g) {
            grid_ = g;
            //leafsize_ = grid_->getLeafSize();
        }

        /**
         * Internal function although it is set to public so it can be accessed if desired.
         *
         * Computes the Fast Iterative Method initialization from the initial points given. Programmed following the paper:
            L. Yatziv, A.Bartesaghi and G. Sapiro, O(n) implementation of the fast marching algorithm, Journal of Computational Physics

         * @see setInitialPoints()
         */
        virtual void init
        () {
            int j = 0;
            int n_neighs = 0;
            for (int &i: init_points_) { // For each initial point
                n_neighs = grid_->getNeighbors(i, neighbors);
                for (int s = 0; s < n_neighs; ++s){  // For each neighbor
                    j = neighbors[s];
                    if ( (grid_->getCell(j).getState() == FMState::FROZEN) || grid_->getCell(j).isOccupied() || (grid_->getCell(j).getVelocity() == 0) ) // If Frozen,obstacle or velocity = 0
                        continue;
                    else {
                        double new_arrival_time = solveEikonal(j);
                        if (grid_->getCell(j).getState() == FMState::NARROW) { // Updating narrow band if necessary.
                            if (new_arrival_time < grid_->getCell(j).getArrivalTime()) {
                                grid_->getCell(j).setArrivalTime(new_arrival_time);
                                narrow_band_.increase( &(grid_->getCell(j))) ;
                            }
                        }
                        else {
                            grid_->getCell(j).setState(FMState::NARROW);
                            grid_->getCell(j).setArrivalTime(new_arrival_time);
                            narrow_band_.push( &(grid_->getCell(j)) );
                        }
                    }
                } // For each neighbor.
            } // For each initial point.
        } // init()

        /**
         * Main Untidy Fast Marching Function. It requires to call first the setInitialPoints() function inherited from Fast Marching.
         *
         * @see setInitialPoints()
         */
        virtual void computeFM
        () {
            int j= 0;
            int n_neighs = 0;
            while (narrow_band_.size() > 0) {
                int idxMin = narrow_band_.index_min();
                narrow_band_.popMinIdx();
                n_neighs = grid_->getNeighbors(idxMin, neighbors);
                grid_->getCell(idxMin).setState(FMState::FROZEN);
                for (int s = 0; s < n_neighs; ++s) { // For each neighbor
                    j = neighbors[s];
                    if ( (grid_->getCell(j).getState() == FMState::FROZEN) || grid_->getCell(j).isOccupied()|| (grid_->getCell(j).getVelocity() == 0) ) // If Frozen,obstacle or velocity = 0
                        continue;
                    else {
                        double new_arrival_time = solveEikonal(j);
                        if (grid_->getCell(j).getState() == FMState::NARROW) { // Updating narrow band if necessary.
                            if (new_arrival_time < grid_->getCell(j).getArrivalTime() ) {
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
                } // For each neighbor.
            } // while narrow band is not empty
        }

    protected:
        FMUntidyqueue narrow_band_; /*!< Instance of the priority queue used. */
};

#endif /* FMM_UNTIDY_H_*/
