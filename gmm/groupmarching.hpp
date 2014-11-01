/*! \file groupmarching.hpp
    \brief Templated class which computes Group Marching Method (GMM).
    
    It uses as a main container the nDGridMap class. The nDGridMap type T
    has to be an FMCell or something inherited from it.
    
    The leafsize of the grid map is ignored since it has to be >=1 and that 
    depends on the units employed.
    
    @par External documentation:
        S. Kim, An O(N) Level Set Method for Eikonal Equations, SIAM J. Sci. Comput., 22(6), 2178–2193.
        <a href="http://epubs.siam.org/doi/abs/10.1137/S1064827500367130">[PDF]</a>
    
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

#ifndef GROUPMARCHING_H_
#define GROUPMARCHING_H_

#include "../fmm/fastmarching.hpp"

template < class grid_t > class GroupMarching : public FastMarching <grid_t> {

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
        GroupMarching <grid_t> () {}

        virtual ~GroupMarching <grid_t>() {}

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
         * Computes the Group Marching Method initialization from the initial points given. Programmed following the paper:
            S. Kim, An O(N) Level Set Method for Eikonal Equations, SIAM J. Sci. Comput., 22(6), 2178–2193.
         *
         * @see setInitialPoints()
         */
        virtual void init
        () {
            deltau_= 1;
            int j = 0;
            int n_neighs = 0;
            tm_= std::numeric_limits<float>::infinity();
            for (int &i: init_points_) { // For each initial point
                n_neighs = grid_->getNeighbors(i, neighbors);
                for (int s = 0; s < n_neighs; ++s){  // For each neighbor
                    j = neighbors[s];
                    if ((grid_->getCell(j).getState() == FMState::FROZEN) || grid_->getCell(j).isOccupied() || (grid_->getCell(j).getVelocity() == 0)) // If Frozen,obstacle or velocity = 0
                        continue;
                    else {
                        double new_arrival_time = solveEikonal(j);
                        if (new_arrival_time < tm_){
                            tm_ = new_arrival_time;
                        }
                        grid_->getCell(j).setArrivalTime(new_arrival_time);
                        grid_->getCell(j).setState(FMState::NARROW);
                        gamma_.push_back(j);
                    } // neighbors open.
                } // For each neighbor.
            } // For each initial point.
        }//init

        /**
         * Main Group Marching Function. It requires to call first the setInitialPoints() function.
         *
         * @see setInitialPoints()
         */
        virtual void computeFM
        () {
            while(gamma_.size() != 0) {
                int n_neighs;
                int j = 0;
    //M1
                tm_ += deltau_;
    //M2
                std::list<int>::reverse_iterator k = gamma_.rbegin();
                std::list<int>::iterator i = k.base();//iterator points to the next element the reverse_iterator is currently pointing to
                i--;
                k = gamma_.rend();
                std::list<int>::iterator q = k.base();
                q--;//the end of a reverse list is the first element of that list
                //This is needed because some functions, like std::list::erase, do not work with reverse iterators

                for( ; i!=q; --i) {//for each gamma in the reverse order
                    if( grid_->getCell(*i).getArrivalTime() <= tm_) {
                        n_neighs = grid_->getNeighbors(*i, neighbors);
                        for (int s = 0; s < n_neighs; ++s){  // For each neighbor of gamma
                            j = neighbors[s];
                            if ( (grid_->getCell(j).getState() == FMState::FROZEN) || grid_->getCell(j).isOccupied() || (grid_->getCell(j).getVelocity() == 0)) // If Frozen,obstacle or velocity = 0
                                continue;
                            else {
                                double new_arrival_time = solveEikonal(j);
                                if (new_arrival_time < grid_->getCell(j).getArrivalTime()) // Updating narrow band if necessary.
                                    grid_->getCell(j).setArrivalTime(new_arrival_time);
                            }
                        }//for each neighbor of gamma
                    }
                }//for each gamma in the reverse order
    //M3
                double narrow_size= gamma_.size();
                i = gamma_.begin();
                for(int z = 0; z < narrow_size; ++z) {//for each gamma in the forward order
                    if( grid_->getCell(*i).getArrivalTime()<= tm_) {
                        n_neighs = grid_->getNeighbors(*i, neighbors);
                        for (int s = 0; s < n_neighs; ++s) {// for each neighbor of gamma
                            j = neighbors[s];
                            if ((grid_->getCell(j).getState() == FMState::FROZEN) || grid_->getCell(j).isOccupied() || (grid_->getCell(j).getVelocity() == 0)) // If Frozen,obstacle or velocity = 0
                                continue;
                            else {
                                double new_arrival_time = solveEikonal(j);
                                if (new_arrival_time < grid_->getCell(j).getArrivalTime()) {
                                        grid_->getCell(j).setArrivalTime(new_arrival_time);
                                }
                                if (grid_->getCell(j).getState() == FMState::OPEN){
                                    gamma_.push_back(j);
                                    grid_->getCell(j).setState(FMState::NARROW);
                                }
                            }
                        }//for each neighbor of gamma
                    grid_->getCell(*i).setState(FMState::FROZEN);
                    i = gamma_.erase(i);
                    }
                    else
                        ++i;
                }//for each gamma in the forward order
            }//while gamma is not zero
        }//compute fm

    protected:
        double tm_; /*!< Global bound that determines the group of cells of gamma that will be updated in each step. */
        double deltau_; /*!< For each updating step, tm_ is increased by this value. */
        std::list<int> gamma_; /*!< List wich stores the narrow band of each iteration. */
};

#endif /* GROUPMARCHING_H_*/
