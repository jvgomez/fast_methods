/*! \file fim.hpp
    \brief Templated class which computes the basic Fast Iterative Method.
    
    It uses as a main container the nDGridMap class. The nDGridMap type T
    has to be an FMCell or something inherited from it.
    
    The leafsize of the grid map is ignored since it has to be >=1 and that 
    depends on the units employed.
    
    @par External documentation:
        W. Jeong and R. Whitaker, A Fast Iterative Method for Eiknal Equations, SIAM J. Sci. Comput., 30(5), 2512–2534.
        <a href="http://epubs.siam.org/doi/abs/10.1137/060670298">[PDF]</a>
    
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

#ifndef FASTITERATIVEMETHOD_H_
#define FASTITERATIVEMETHOD_H_

#include "../fmm/fastmarching.hpp"

template < class grid_t > class FastIterativeMethod : public FastMarching <grid_t> {

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
        FastIterativeMethod <grid_t> () {}

        virtual ~FastIterativeMethod <grid_t>() {}

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
            W. Jeong and R. Whitaker, A Fast Iterative Method for Eiknal Equations, SIAM J. Sci. Comput., 30(5), 2512–2534.
         *
         * @see setInitialPoints()
         */
        virtual void init
        () {
            int j = 0;
            int n_neighs = 0;
            for (int &i: init_points_) {// For each initial point
                n_neighs = grid_->getNeighbors(i, neighbors);
                for (int s = 0; s < n_neighs; ++s) {// For each neighbor
                    j = neighbors[s];
                    if ( (grid_->getCell(j).getState() == FMState::OPEN) && (grid_->getCell(j).getVelocity() != 0)) {// Only OPEN neighbors and with velocity
                        active_list_.push_back(j);
                        grid_->getCell(j).setState(FMState::NARROW);
                    }
                }
            }
        } // init()

        /**
         * Main Fast Iterative Method Function. It requires to call first the setInitialPoints() function.
         *
         * @see setInitialPoints()
         */
        virtual void computeFM
        () {
            double q=-1;
            double p=-1;
            int n_neighs;
            int j = 0;

            while(active_list_.size() != 0){ //while active list is not zero
                for (std::list<int>::iterator i=active_list_.begin(); i!=active_list_.end(); ++i) {// for each cell of active_list
                    p = grid_->getCell(*i).getArrivalTime();
                    q = solveEikonal(*i);
                    if(p>q)
                        grid_->getCell(*i).setArrivalTime(q);
                    if (fabs(p - q) <= E_) {// if the cell has converged
                        grid_->getCell(*i).setState(FMState::FROZEN);
                        n_neighs = grid_->getNeighbors(*i, neighbors);
                        for (int s = 0; s < n_neighs; ++s){  // For each neighbor of converged cells of active_list
                            j = neighbors[s];
                            if ((grid_->getCell(j).getState() == FMState::OPEN) && (grid_->getCell(j).getVelocity() != 0)) {
                                active_list_.insert(i,j);
                                grid_->getCell(j).setState(FMState::NARROW);
                            }
                        }// For each neighbor of converged cells of active_list
                    grid_->getCell(*i).setState(FMState::FROZEN);
                    i = active_list_.erase(i);
                    i--;
                    }// if the cell has converged
                }// for each cell of active_list
            }//while active_list is not zero
        }//compute fm

    protected:
        std::list<int> active_list_; /*!< List wich stores the narrow band of each iteration. */
        double E_= 0; /*!< Error threshold value that reveals if a cell has converged. */
};

#endif /* FASTITERATIVEMETHOD_H_*/
