#ifndef FMCELL_H_
#define FMCELL_H_

#include <iostream>
#include <string>
#include <limits>

#include "../ndgridmap/cell.h"


enum class FMState {OPEN, NARROW, FROZEN};

class FMCell : public Cell{
	friend std::ostream& operator << (std::ostream & os, const FMCell & c);
	
    public: 
    // Implicit Fast Marching method initialization.
       FMCell() : Cell(std::numeric_limits<float>::infinity(), false), state_(FMState::OPEN), velocity_(1)  {};
        virtual ~FMCell() {};
        
        // NOTE: no checks are done (out of bounds, correct states, etc) no improve efficienty.
        // TODO: overload functions to add the option of input checking.
        virtual void setVelocity (const float v)    		{velocity_ = v;}
        virtual void setArrivalTime (const float at)    	{value_= at;}
        virtual void setState (const FMState state)			{state_ = state;}
        // Occupied means velocity 0 but not vice-versa.
        virtual void setOccupancy(const bool o)				{occupancy_ = o;
															 if (o == 0) setVelocity(0);}
															 
        std::string type () {return std::string("FMCell - Fast Marching cell");}
           
        virtual float getArrivalTime () const				{return value_;}
        virtual float getVelocity () const					{return velocity_;}
        virtual FMState getState () const					{return state_;}
        
        inline bool isObstacle() const;

    private:
		//value_ is in this case the time of arrival.
		FMState state_;
        float velocity_;    
};


#endif /* CELL_H_*/
