#ifndef MYHEAP_H_
#define MYHEAP_H_

#include "../ndgridmap/ndgridmap.hpp"
#include "fmcell.h"

#include <list>

static nDGridMap<FMCell> * grid_ ;

// Heap solution seen in: http://stackoverflow.com/a/16706002/2283531
/*struct compare_cells {
	inline bool operator()
	(const int & idx1, const int & idx2) const {
		//std::cout << grid_->cells_[idx1].getArrivalTime() << "  " << grid_->cells_[idx2].getArrivalTime() << std::endl;
		return grid_->cells_[idx1].getArrivalTime() > grid_->cells_[idx2].getArrivalTime();			 
	}
};*/

//typedef typename boost::heap::fibonacci_heap<int, boost::heap::compare<compare_cells>>::handle_type handle_t;

//TODO: improve heap with a handles system.
class MyHeap {
	
	public:
	MyHeap () {};
	//MyHeap (const int & n) { heap_.reserve(n);}
	virtual ~ MyHeap() {};
	
	/*void setMaxSize
	(const int & n) {
		heap_.reserve(n);
	}*/
	
	/*void setEnvironment
	(nDGridMap<FMCell> & grid) {
		grid_ = &grid;
		setMaxSize(grid_->size());
	}*/
	
	void push 
	(FMCell * pD) {
		if (heap.size()==0) heap.push_back(pD);

		// check if it must be placed at the beginning
		std::list<FMCell *>::iterator it = heap.begin();
		if (heap.size()>0 &&  (*it)->getArrivalTime() >= pD->getArrivalTime()){
			heap.insert(heap.begin(), pD);
			return;
		}
		it = heap.end();
		// check if it must be placed at the end
		if (heap.size()>0 && (*it)->getArrivalTime() <= pD->getArrivalTime()){
			heap.push_back(pD);
			return;
		}

		
		// insert in the middle
		//std::list<FMCell *>::iterator it;
		for (it=heap.begin(); it!= heap.end(); it++){
			FMCell* current = *it;
			//if current is smaller, insert there
			if(pD->getArrivalTime() <= current->getArrivalTime()){
				it = heap.insert(it,pD);
				return;
			}
		}
		
		heap.push_back(pD);
	}
	
	int popMinIdx
	() {
		FMCell* current = heap.front();
		heap.erase(heap.begin());
		return current->getIndex();	
	}
	
	size_t size
	() const {
		heap.size();
	}
	
	
	void increase
	(FMCell * pD, const float & new_time) {
		std::list<FMCell *>::iterator it = findElement(pD);
		heap.erase(it);
		pD->setArrivalTime(new_time);
		push(pD);
	}
	
	
	std::list<FMCell *>::iterator findElement
	(FMCell * pD) 	{
		
		unsigned int init = 0;
		unsigned int end = heap.size();
		unsigned int middle = (end + init) / 2;
		
		std::list<FMCell *>::iterator it = std::next(heap.begin(), middle);
		std::list<FMCell *>::iterator it1 = std::next(heap.begin(), middle+1);
		while ( pD != (*it) && (*it)->getArrivalTime() != pD->getArrivalTime() && (*it1)->getArrivalTime() != pD->getArrivalTime() ){
			if ( (*it)->getArrivalTime() > pD->getArrivalTime() ){ // it is before
				end = middle;
				middle = (end + init) / 2;
			} else{ //it is after
				init = middle;
				middle = (end + init) / 2;
			}
			it = std::next(heap.begin(), middle);
			it1 = std::next(heap.begin(), middle+1);
		}
		
		/*it must be just before or after */

		int aux=0;
		for (int i=0; i<heap.size(); i++)
		{
			aux = middle + i;
			it = std::next(heap.begin(), aux);
			if ( *it == pD ){
				return it;
			}


			aux = middle - i;
			it = std::next(heap.begin(), aux);
			if ( (*it) == pD ){
				return it;
			}

		}
	}
			
	
	protected:
		std::list<FMCell *> heap;
};


#endif /* FMFIBHEAP_H_ */

