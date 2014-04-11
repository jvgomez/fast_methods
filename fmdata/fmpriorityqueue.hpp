/*! \file fmpriorityqueue.hpp
    \brief Wrap for the Boost Priority Queue class.
    
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


#ifndef FMPRIORITYQUEUE_H_
#define FMPRIORITYQUEUE_H_


#include <boost/heap/priority_queue.hpp>

#include "fmcell.h"


/** 
 * This struct is used a comparator for the heap. Since a minimum-heap
 * is desired the operation checked is param1 > param2 as seen in this
 * [Stack Overflow post](http://stackoverflow.com/a/16706002/2283531)
 * */
template <class cell_t>struct compare_cells_pq {
	inline bool operator()
	(const cell_t * c1 , const cell_t * c2) const {

		return c1->getArrivalTime() > c2->getArrivalTime();			 
	}
};

// TODO: Template this class.
template <class cell_t = FMCell> class FMPriorityQueue{
	
	public:
		FMPriorityQueue () {};
		FMPriorityQueue (const int & n) { heap_.reserve(n); }
		virtual ~ FMPriorityQueue() {};
		
		/**
		 * Set the maximum number of cells the heap will contain.
		 * 
		 * @param maximum number of cells.
		 */
		void setMaxSize
		(const int & n) {
			heap_.reserve(n);
		}
		
		/**
		 * Adds an element to the heap.
		 * 
		 * @param cell to add.
		 */
		void push 
		(const cell_t * c) {
			heap_.push(c);
		}
		
		/**
		 * Priority queues do not allow key increasing. Therefore, it pushes the element again.
		 * This is done for FMM-SFMM compatibility.
		 * 
		 * @param cell to add.
		 */
		void increase 
		(const FMCell * c) {
			heap_.push(c);
		}
		
		/**
		 * pops index of the element with lowest value and removes it from the heap.
		 * 
		 * @return index of the cell with lowest value.
		 */ 
		int popMinIdx
		() {
			const int idx = heap_.top()->getIndex();
			heap_.pop();
			return idx;	
		}
		
		size_t size
		() const {
			return heap_.size();
		}
			
		
	protected:
		boost::heap::priority_queue<const cell_t *, boost::heap::compare<compare_cells_pq<cell_t> > > heap_;  /*!< The actual heap for FMCells. */
};


#endif /* FMPRIORITYQUEUE_H_ */

