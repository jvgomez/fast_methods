/*! \file fmuntidyqueue.hpp
    \brief Wrap for untidyqueue priority queue class.
    
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

#ifndef FMUNTIDYQUEUE_H_
#define FMUNTIDYQUEUE_H_

#include "../../thirdparty/untidy_queue.hpp"
#include "fmuntidycell.h"

class FMUntidyqueue {

    public:
        FMUntidyqueue () {}
        virtual ~ FMUntidyqueue() {}

        /**
         * Set the maximum number of cells the heap will contain.
         *
         * @param maximum number of cells.
         */
        void setMaxSize
        (const int & n) {
        }

        void push
        ( FMUntidyCell * c) {
            int i = queue_.push(c,c->getArrivalTime());
            c->setBucket(i);
        }

        /**
         * pops index of the element with lowest value and removes it from the heap.
         *
         */
        void popMinIdx
        () {
            queue_.pop();
        }

        size_t size
        () const {
            return queue_.size();
        }

        void update
        (const FMUntidyCell * c) {
        }

        /**
         * Updates the position of the cell in the priority queue. Its priority can only increase.
         * Also updates the bucket of the cell.
         *
         * @param c FMUntidyCell to be updated.
         *
         */
        void increase
        (FMUntidyCell * c) {
            int i = queue_.increase_priority(c,c->getbucket(),c->getArrivalTime());
            c->setBucket(i);
        }

        /**
         * pops index of the element with lowest value of the priority queue.
         *
         */
        int index_min
        (){
            const FMCell * c = queue_.top();
            int index_pop = c->getIndex();
            return index_pop;
        }

        bool empty
        () const {
            return queue_.empty();
        }

        void clear
        () {
            queue_.clear();
        }

    protected:
        levelset::PriorityQueue<const FMCell * > queue_; /*!< Priority queue that stores cells of the narrow band. */
};

#endif /* FMUNTIDYQUEUE_H_ */
