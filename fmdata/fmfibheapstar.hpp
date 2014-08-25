/*! \file fmfibheapstar.hpp
    \brief Wrap for the Boost Fibonacci Heap class.
    
    Copyright (C) 2014 Javier V. Gomez and Jose Pardeiro
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


#ifndef FMFIBHEAPSTAR_H_
#define FMFIBHEAPSTAR_H_


#include <boost/heap/fibonacci_heap.hpp>

#include "fmcell.h"
#include "fmstarcell.h"

/** 
 * This struct is used a comparator for the heap. Since a minimum-heap
 * is desired the operation checked is param1 > param2 as seen in this
 * [Stack Overflow post](http://stackoverflow.com/a/16706002/2283531)
 * */
template <class cell_t, const bool FMStar_t> struct compare_cells_star {
	inline bool operator()
	(const cell_t * c1 , const cell_t * c2) const {

        if (FMStar_t==false)
            return c1->getArrivalTime() > c2->getArrivalTime();
        else if (FMStar_t==true)
            return (c1->getArrivalTime() + c1->getHeuristic() > c2->getArrivalTime() + c2->getHeuristic());
	}
};

// TODO: Template this class.
template <class cell_t = FMCell, const bool FMStar_t = false> class FMFibHeapStar {
	
    typedef boost::heap::fibonacci_heap<const cell_t *, boost::heap::compare<compare_cells_star<cell_t, FMStar_t> > > fib_heap_t;
	typedef typename fib_heap_t::handle_type handle_t;
	
	public:
        FMFibHeapStar () {};
        FMFibHeapStar (const int & n) {	handles_.resize(n);}
        virtual ~ FMFibHeapStar() {};
		
		/**
		 * Set the maximum number of cells the heap will contain.
		 * 
		 * @param maximum number of cells.
		 */
		void setMaxSize
		(const int & n) {
			handles_.resize(n);
		}
		
		void push 
		(const cell_t * c) {
			handles_[c->getIndex()] = heap_.push(c);
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
		
		/**
		 * Updates the position of the cell in the heap. Its priority can increase or decrease.
		 * 
		 * @param c cell_t to be updated.
		 * 
		 * @see increase()
		 */
		void update
		(const cell_t * c) {
			heap_.update(handles_[c->getIndex()], c);
		}
		
		/**
		 * Updates the position of the cell in the heap. Its priority can only increase.
		 * It is more efficient than the update() function if it is ensured that the priority
		 * will increase.
		 * 
		 * @param c cell_t to be updated.
		 * 
		 * @see update()
		 */
		void increase
		(const cell_t * c) {
			heap_.increase(handles_[c->getIndex()],c);
		}
			
		
	protected:
		fib_heap_t heap_;  /*!< The actual heap for cell_t. */
		std::vector<handle_t> handles_;  /*!< Stores the handles of each cell by keeping the indices: handles_(0) is the handle for
											the cell with index 0 in the grid. Makes possible to update the heap.*/
};


#endif /* FMFIBHEAPSTAR_H_ */

