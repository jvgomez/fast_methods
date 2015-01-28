/*! \file fmcell.h
    \brief Header of the FMCell class

    A stand-alone, standard C++ class which represents each one of the cells
    of a gridmap and its typical members. Inherited from Cell class, in this
    case the value_ member represents the distance value (or time of arrival).
   
    IMPORTANT NOTE: no checks are done in the set functions.
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

#ifndef FMCELL_H_
#define FMCELL_H_

#include <iostream>
#include <string>
#include <limits>

#include "../../ndgridmap/cell.h"

/**
  * Possible states of the FMCells
  * */
enum class FMState {OPEN, NARROW, FROZEN};

class FMCell : public Cell{
    /**
      * ostream operator << overloaded for this class.
      */
    friend std::ostream& operator << (std::ostream & os, const FMCell & c);

    public:
        /**
          * Default constructor which performs and implicit Fast Marching-like initialization of the grid,
          */
        FMCell() : Cell(std::numeric_limits<double>::infinity(), true), state_(FMState::OPEN), velocity_(1)  {}

        virtual ~FMCell() {}

        // NOTE: no checks are done (out of bounds, correct states, etc) no improve efficienty.
        // TODO: overload functions to add the option of input checking.
        virtual void setVelocity (const float v)            {velocity_ = v;}
        virtual void setArrivalTime (const double at)       {value_= at;}
        virtual void setState (const FMState state)         {state_ = state;}
        virtual void setDefault();

        /**
         * Set the occupancy_. Occupied means velocity 0 but not vice-versa.
         *
         * IMPORTANT NOTE: a false occupancy sets the velocity also to 0.
         */
        virtual void setOccupancy(const bool o) {
            occupancy_ = o;
            if (o == false)
                setVelocity(0);
        }

        std::string type () {return std::string("FMCell - Fast Marching cell");}

        virtual double getArrivalTime () const              {return value_;}
        virtual float getVelocity () const                  {return velocity_;}
        virtual FMState getState () const                   {return state_;}

        virtual bool isObstacle() const;

    protected:
        FMState state_;   /*!< State of the cell */
        float velocity_;  /*!< Wave propagation velocity through this cell */
};

#endif /* FMCELL_H_*/
