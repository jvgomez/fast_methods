
#ifndef FMFIBHEAP_H_
#define FMFIBHEAP_H_


#include <boost/heap/fibonacci_heap.hpp>

#include "fmcell.h"


// Heap solution seen in: http://stackoverflow.com/a/16706002/2283531
struct compare_cells {
	inline bool operator()
	(const FMCell * c1 , const FMCell * c2) const {

		return c1->getArrivalTime() > c2->getArrivalTime();			 
	}
};

typedef typename boost::heap::fibonacci_heap<const FMCell *, boost::heap::compare<compare_cells>>::handle_type handle_t;

class FMFibHeap {
	
	public:
	FMFibHeap () {};
	FMFibHeap (const int & n) {	handles_.resize(n);}
	virtual ~ FMFibHeap() {};
	
	void setMaxSize
	(const int & n) {
		handles_.resize(n);
	}
	
	void push 
	(const FMCell * c) {
		handles_[c->getIndex()] = heap_.push(c);
	}
	
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
	
	void update
	(const FMCell * c) {
		heap_.update(handles_[c->getIndex()], c);
	}
	
	void increase
	(const FMCell * c) {
		heap_.increase(handles_[c->getIndex()],c);
	}
		
	
	protected:
		boost::heap::fibonacci_heap<const FMCell *, boost::heap::compare<compare_cells>> heap_;
		std::vector<handle_t> handles_;
};


#endif /* FMFIBHEAP_H_ */

