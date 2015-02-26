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
    along with this program. If not, see <http://www.gnu.org/licenses/>. */

#ifndef CELL_H_
#define CELL_H_

#include <iostream>
#include <string>
#include <limits>

// NOTE: no checks are done (out of bounds, correct states, etc) no improve efficienty.
// TODO: overload functions to add optional input checking.
class Cell {

    friend std::ostream& operator << (std::ostream & os, Cell & c);

    public:
        /** Default constructor: sets value_ to -1 and occupancy_ to true (clear cell, not occupied). */
        Cell() : value_(-1), occupancy_(1) {}

        Cell(double v, double o = 1) : value_(v), occupancy_(o) {}

        virtual inline void setValue(double v)            {value_ = v;}
        virtual inline void setOccupancy(double o)        {occupancy_ = o;}
        virtual std::string type()                        {return std::string("Cell - Basic cell");}
        virtual inline void setIndex(int i)               {index_ = i;}

        /** Sets default values for the cell. Concretely, restarts value_ = -1 but
            occupancy_ is not modified. */
        virtual void setDefault();

        virtual inline double getValue() const             {return value_;}
        virtual inline double getOccupancy() const         {return occupancy_;}
        virtual inline unsigned int getIndex() const       {return index_;}

        virtual inline bool isOccupied() const {
            if (occupancy_ < std::numeric_limits<double>::epsilon() * 1e3)
                return true;
            return false;
        }

    protected:
        double value_; /*!< Value of the cell. */
        double occupancy_; /*!< Binary occupancy, true means clear, false occupied. */
        unsigned int index_; /*!< Index within the grid. Useful when used in heaps. */
};

#endif /* CELL_H_*/
