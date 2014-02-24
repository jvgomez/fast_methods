
#ifndef CELL_H_
#define CELL_H_

#include <iostream>
#include <string>

class Cell {
	friend std::ostream& operator << (std::ostream & os, Cell & c);
	
    public: 
        Cell() : value_(-1), occupancy_(true) {};
        Cell(float v, bool o = true) : value_(v), occupancy_(o) {};
        virtual ~Cell() {};
        
        // NOTE: no checks are done (out of bounds, correct states, etc) no improve efficienty.
        // TODO: overload functions to add the option of input checking.
        virtual void setValue(const float v)        	{value_ = v;}
        virtual void setOccupancy (const bool o)		{occupancy_ = o;}
        virtual std::string type ()						{return std::string("Cell - Basic cell");}
        virtual void setIndex(const int i)				{index_ = i;}
		
        // This function gets no arguments in this case, but in the derivated classes it could. The ... 
		// says that parameters could be given (or not).
        virtual float getValue() const        		{return value_;}
        virtual bool getOccupancy() const			{return occupancy_;}
		virtual int getIndex() const				{return index_;}
        

    protected:
        float value_;
        bool occupancy_; // 1 means clear, 0 means occupied.
        int index_; // Required because of the use of the heap.       
};


#endif /* CELL_H_*/
