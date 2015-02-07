/*! \file cell.h
    \brief Header of the Cell class

    A stand-alone, standard C++ class which represents each one of the cells
    of a gridmap and its typical members.

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

#ifndef CELL_H_
#define CELL_H_

#include <iostream>
#include <string>

class Cell {
      /**
       * ostream operator << overloaded for this class.
       */
    friend std::ostream& operator << (std::ostream & os, Cell & c);

    public:
        /**
          * Default constructor: sets value_ to -1 and occupancy_ to true (clear cell, not occupied).
          */
        Cell() : value_(-1), occupancy_(true) {}

        Cell(double v, bool o = true) : value_(v), occupancy_(o) {}

        /**
          * Destructor not used.
         */
        virtual ~Cell() {}

        // NOTE: no checks are done (out of bounds, correct states, etc) no improve efficienty.
        // TODO: overload functions to add the option of input checking.

        virtual void setValue(double v)            {value_ = v;}
        virtual void setOccupancy(bool o)          {occupancy_ = o;}
        virtual std::string type()                 {return std::string("Cell - Basic cell");}
        virtual void setIndex(int i)               {index_ = i;}
        virtual void setDefault();

        // This function gets no arguments in this case, but in the derivated classes it could. The ...
        // says that parameters could be given (or not).
        virtual double getValue() const             {return value_;}
        virtual bool getOccupancy() const           {return occupancy_;}
        virtual unsigned int getIndex() const       {return index_;}

        virtual bool isOccupied() const             {return !occupancy_;}

    protected:
        double value_; /*!< Value of the cell. */
        bool occupancy_; /*!< Binary occupanxy, true means clear, false occupied. */
        unsigned int index_; /*!< By design, each cell does not require to know its index within the grid
                        however, it is very useful when used in heaps*/
};

#endif /* CELL_H_*/
